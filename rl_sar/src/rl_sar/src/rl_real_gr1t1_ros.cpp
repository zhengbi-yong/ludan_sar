/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_gr1t1_ros.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#if defined(PLOT)
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

namespace
{
template <typename T>
std::vector<T> ReadVector(const YAML::Node &node)
{
    std::vector<T> values;
    if (!node)
    {
        return values;
    }
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}
}

RL_Real::RL_Real()
{
    this->ang_vel_type = "ang_vel_body";
    this->robot_name = "gr1t1";
    this->config_name = "robot_hw";
    this->ReadYamlBase(this->robot_name);

    // load hardware specific config
    this->LoadHardwareConfig();

    // ROS interface
    this->cmd_vel_subscriber_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
    for (const auto &topic : joint_state_topics_)
    {
        joint_state_subs_.push_back(nh_.subscribe<sensor_msgs::JointState>(topic, 10, &RL_Real::JointStateCallback, this));
    }
    if (!imu_topic_.empty())
    {
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 10, &RL_Real::ImuCallback, this);
    }
    command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(command_topic_, 10);

    spinner_ = std::make_unique<ros::AsyncSpinner>(2);
    spinner_->start();

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "No FSM registered for robot: " << this->robot_name << std::endl;
    }

    this->InitOutputs();
    this->InitControl();

    mapped_joint_positions_.assign(this->params.num_of_dofs, 0.0);
    mapped_joint_velocities_.assign(this->params.num_of_dofs, 0.0);

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#if defined(PLOT)
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos)
    {
        vector = std::vector<double>(this->plot_size, 0.0);
    }
    for (auto &vector : this->plot_target_joint_pos)
    {
        vector = std::vector<double>(this->plot_size, 0.0);
    }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.05, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#if defined(CSV_LOGGER)
    this->CSVInit(this->robot_name + "/" + this->config_name);
#endif
}

RL_Real::~RL_Real()
{
    if (this->loop_keyboard)
    {
        this->loop_keyboard->shutdown();
    }
    if (this->loop_control)
    {
        this->loop_control->shutdown();
    }
    if (this->loop_rl)
    {
        this->loop_rl->shutdown();
    }
#if defined(PLOT)
    if (this->loop_plot)
    {
        this->loop_plot->shutdown();
    }
#endif
    if (spinner_)
    {
        spinner_->stop();
    }
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::LoadHardwareConfig()
{
    std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + this->robot_name + "/robot_hw/hardware.yaml";
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)["hardware"];
    }
    catch (const YAML::BadFile &e)
    {
        throw std::runtime_error("Failed to load hardware config: " + config_path);
    }

    joint_state_topics_ = ReadVector<std::string>(config["joint_state_topics"]);
    command_topic_ = config["command_topic"].as<std::string>("");
    if (command_topic_.empty())
    {
        throw std::runtime_error("command_topic must not be empty in hardware config");
    }
    imu_topic_ = config["imu_topic"].as<std::string>("");
    hardware_joint_names_ = ReadVector<std::string>(config["command_joint_names"]);
    if (hardware_joint_names_.empty())
    {
        throw std::runtime_error("command_joint_names must not be empty in hardware config");
    }

    joint_directions_ = ReadVector<double>(config["joint_directions"]);
    if (joint_directions_.empty())
    {
        joint_directions_.assign(hardware_joint_names_.size(), 1.0);
    }
    else if (joint_directions_.size() != hardware_joint_names_.size())
    {
        throw std::runtime_error("joint_directions size mismatch with command_joint_names");
    }

    joint_name_to_index_.clear();
    for (size_t i = 0; i < hardware_joint_names_.size(); ++i)
    {
        joint_name_to_index_[hardware_joint_names_[i]] = i;
    }

    joint_positions_.assign(hardware_joint_names_.size(), 0.0);
    joint_velocities_.assign(hardware_joint_names_.size(), 0.0);
    joint_efforts_.assign(hardware_joint_names_.size(), 0.0);

    command_cache_.assign(hardware_joint_names_.size(), {0.0, 0.0, 0.0, 0.0, 0.0});

    if (config["default_command"])
    {
        const YAML::Node default_cmd = config["default_command"];
        auto default_pos = ReadVector<double>(default_cmd["position"]);
        auto default_vel = ReadVector<double>(default_cmd["velocity"]);
        auto default_kp = ReadVector<double>(default_cmd["kp"]);
        auto default_kd = ReadVector<double>(default_cmd["kd"]);
        auto default_ff = ReadVector<double>(default_cmd["ff"]);

        for (size_t i = 0; i < command_cache_.size(); ++i)
        {
            if (i < default_pos.size()) command_cache_[i][0] = default_pos[i];
            if (i < default_vel.size()) command_cache_[i][1] = default_vel[i];
            if (i < default_kp.size())  command_cache_[i][2] = default_kp[i];
            if (i < default_kd.size())  command_cache_[i][3] = default_kd[i];
            if (i < default_ff.size())  command_cache_[i][4] = default_ff[i];
        }
    }
}

void RL_Real::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        auto it = joint_name_to_index_.find(msg->name[i]);
        if (it == joint_name_to_index_.end())
        {
            continue;
        }
        const size_t index = it->second;
        if (index >= joint_positions_.size())
        {
            continue;
        }
        if (i < msg->position.size())
        {
            joint_positions_[index] = msg->position[i];
        }
        if (i < msg->velocity.size())
        {
            joint_velocities_[index] = msg->velocity[i];
        }
        if (i < msg->effort.size())
        {
            joint_efforts_[index] = msg->effort[i];
        }
    }
}

void RL_Real::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_quaternion_[0] = msg->orientation.w;
    imu_quaternion_[1] = msg->orientation.x;
    imu_quaternion_[2] = msg->orientation.y;
    imu_quaternion_[3] = msg->orientation.z;

    imu_gyro_[0] = msg->angular_velocity.x;
    imu_gyro_[1] = msg->angular_velocity.y;
    imu_gyro_[2] = msg->angular_velocity.z;
}

void RL_Real::GetState(RobotState<double> *state)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    state->imu.quaternion[0] = imu_quaternion_[0];
    state->imu.quaternion[1] = imu_quaternion_[1];
    state->imu.quaternion[2] = imu_quaternion_[2];
    state->imu.quaternion[3] = imu_quaternion_[3];

    state->imu.gyroscope[0] = imu_gyro_[0];
    state->imu.gyroscope[1] = imu_gyro_[1];
    state->imu.gyroscope[2] = imu_gyro_[2];

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        size_t hardware_index = static_cast<size_t>(this->params.joint_mapping[i]);
        if (hardware_index >= joint_positions_.size())
        {
            state->motor_state.q[i] = 0.0;
            state->motor_state.dq[i] = 0.0;
            state->motor_state.tau_est[i] = 0.0;
            continue;
        }
        double direction = joint_directions_[hardware_index];
        state->motor_state.q[i] = joint_positions_[hardware_index] * direction;
        state->motor_state.dq[i] = joint_velocities_[hardware_index] * direction;
        state->motor_state.tau_est[i] = joint_efforts_[hardware_index] * direction;

        mapped_joint_positions_[i] = state->motor_state.q[i];
        mapped_joint_velocities_[i] = state->motor_state.dq[i];
    }
}

void RL_Real::PublishCommand()
{
    const size_t joint_count = command_cache_.size();
    if (joint_count == 0 || !command_pub_)
    {
        return;
    }

    std_msgs::Float64MultiArray msg;
    msg.data.resize(joint_count * 5);
    for (size_t i = 0; i < joint_count; ++i)
    {
        size_t base = i * 5;
        msg.data[base + 0] = command_cache_[i][0];
        msg.data[base + 1] = command_cache_[i][1];
        msg.data[base + 2] = command_cache_[i][2];
        msg.data[base + 3] = command_cache_[i][3];
        msg.data[base + 4] = command_cache_[i][4];
    }
    command_pub_.publish(msg);
}

void RL_Real::SetCommand(const RobotCommand<double> *command)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        size_t hardware_index = static_cast<size_t>(this->params.joint_mapping[i]);
        if (hardware_index >= command_cache_.size())
        {
            continue;
        }
        double direction = joint_directions_[hardware_index];
        command_cache_[hardware_index][0] = command->motor_command.q[i] * direction;
        command_cache_[hardware_index][1] = command->motor_command.dq[i] * direction;
        command_cache_[hardware_index][2] = command->motor_command.kp[i];
        command_cache_[hardware_index][3] = command->motor_command.kd[i];
        command_cache_[hardware_index][4] = command->motor_command.tau[i] * direction;
    }
    PublishCommand();
}

void RL_Real::RobotControl()
{
    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0;
        this->control.y = 0;
        this->control.yaw = 0;
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::N)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl
                  << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);
    this->SetCommand(&this->robot_command);
}

torch::Tensor RL_Real::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (!this->params.observations_history.empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->cmd_vel_.linear.x, this->cmd_vel_.linear.y, this->cmd_vel_.angular.z}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (this->output_dof_pos.defined() && this->output_dof_pos.numel() > 0)
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (this->output_dof_vel.defined() && this->output_dof_vel.numel() > 0)
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (this->output_dof_tau.defined() && this->output_dof_tau.numel() > 0)
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

#if defined(CSV_LOGGER)
        torch::Tensor tau_est = torch::tensor(this->robot_state.motor_state.tau_est).unsqueeze(0);
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

#if defined(PLOT)
void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mapped_joint_positions_[i]);
        this->plot_target_joint_pos[i].push_back(this->output_dof_pos[0][i].item<double>());
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}
#endif

void RL_Real::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel_ = *msg;
}

#if !defined(USE_CMAKE) && defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1) && defined(USE_ROS)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar_gr1t1");
    {
        RL_Real rl_sar;
        ros::waitForShutdown();
    }
#endif
    return 0;
}
