/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_GR1T1_ROS_HPP
#define RL_REAL_GR1T1_ROS_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#if !defined(USE_ROS1) || !defined(USE_ROS)
#error "rl_real_gr1t1_ros is only supported in ROS1 environments"
#endif

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#if defined(USE_ROS1) && defined(USE_ROS)
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/async_spinner.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include <csignal>
#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class RL_Real : public RL
#if defined(USE_ROS2) && defined(USE_ROS)
    , public rclcpp::Node
#endif
{
public:
    RL_Real();
    ~RL_Real();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // ros callbacks
    void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

#if defined(USE_ROS1) && defined(USE_ROS)
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif

    void LoadHardwareConfig();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // hardware state cache
    std::vector<std::string> hardware_joint_names_;
    std::vector<double> joint_directions_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<std::array<double, 5>> command_cache_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;
    std::vector<std::string> joint_state_topics_;
    std::string command_topic_;
    std::string imu_topic_;
    geometry_msgs::Twist cmd_vel_{};
    std::array<double, 4> imu_quaternion_{{1.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> imu_gyro_{{0.0, 0.0, 0.0}};

    std::mutex data_mutex_;

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> joint_state_subs_;
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher command_pub_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
#elif defined(USE_ROS2) && defined(USE_ROS)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
#endif

    // others
    int motiontime = 0;
    std::vector<double> mapped_joint_positions_;
    std::vector<double> mapped_joint_velocities_;

    void PublishCommand();
};

#endif // RL_REAL_GR1T1_ROS_HPP
