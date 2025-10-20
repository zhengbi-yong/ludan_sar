#include "legged_dm_hw/DmHW.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <dmbot_serial/robot_connect.h>

#include <fstream>
#include <iostream>

namespace legged
{

static void writedata2file(float pos,float vel,float tau,const std::string& path)
{
  std::ofstream f(path, std::ios::app);
  if (!f.is_open()) return;
  f << pos << " " << vel << " " << tau << "\n";
}

bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  // 为了避免外部控制器切换误操作，置一个无效值
  root_nh.setParam("gsmp_controller_switch", "null");

  // 订阅外部IMU与手动覆盖话题
  odom_sub_       = root_nh.subscribe("/imu/data",   1, &DmHW::OdomCallBack, this);
  hybrid_cmd_sub_ = root_nh.subscribe("/hybrid_cmd", 1, &DmHW::hybridCmdCb,  this);
  emg_sub_        = root_nh.subscribe("/emergency_stop", 1, &DmHW::emgCb,    this);

  // 允许在 launch 里配置是否启用手动覆盖和超时
  root_nh.param("manual_topic_override", manual_override_, false);
  root_nh.param("manual_cmd_timeout",    manual_cmd_timeout_, 0.2);

  // 先跑基类（载入URDF等，注册接口，构造 motorsInterface）
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);

  // 注册关节、IMU句柄
  setupJoints();
  setupImu();

  // 可选的状态与命令回显（不强制）
  cmd_pos_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_pos", 10);
  cmd_vel_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_vel", 10);
  cmd_ff_pub_   = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_ff", 10);

  read_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_pos", 10);
  read_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_vel", 10);
  read_ff_pub_  = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_ff", 10);

  last_cmd_stamp_ = ros::Time(0);
  emergency_stop_ = false;

  return true;
}

void DmHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
  // 从下位机读 14 路电机
  double pos, vel, tau;
  for (int i=0; i< NUM_JOINTS; ++i)
  {
    motorsInterface->get_motor_data(pos, vel, tau, i);
    jointData_[i].pos_ = pos * directionMotor_[i];
    jointData_[i].vel_ = vel * directionMotor_[i];
    jointData_[i].tau_ = tau * directionMotor_[i];
    ROS_INFO_STREAM_THROTTLE(0.5, "pos[" << i << "] = " << jointData_[i].pos_);
  }

  // IMU 直接用外部 /imu/data
  imuData_.ori[0] = yesenceIMU_.orientation.x;
  imuData_.ori[1] = yesenceIMU_.orientation.y;
  imuData_.ori[2] = yesenceIMU_.orientation.z;
  imuData_.ori[3] = yesenceIMU_.orientation.w;

  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;

  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;
}

void DmHW::write(const ros::Time& time, const ros::Duration& /*period*/)
{
  bool use_manual = false;

  if (manual_override_)
  {
    const bool alive = (time - last_cmd_stamp_).toSec() < manual_cmd_timeout_;
    use_manual = alive && !emergency_stop_;
  }

  // 生成要下发的10路命令
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    double pos_des, vel_des, kp, kd, ff;

    if (use_manual)
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      pos_des = cmd_pos_[i];
      vel_des = cmd_vel_[i];
      kp      = cmd_kp_[i];
      kd      = cmd_kd_[i];
      ff      = cmd_tau_[i];
      ROS_INFO_STREAM("manual");
    }
    else
    {
      pos_des = jointData_[i].pos_des_;
      vel_des = jointData_[i].vel_des_;
      kp      = jointData_[i].kp_;
      kd      = jointData_[i].kd_;
      ff      = jointData_[i].ff_;
      // ROS_INFO_STREAM("not manual!!!!");
    }

    // 关节→电机 方向映射
    dmSendcmd_[i].pos_des_ = pos_des * directionMotor_[i];
    dmSendcmd_[i].vel_des_ = vel_des * directionMotor_[i];
    dmSendcmd_[i].kp_      = kp;
    dmSendcmd_[i].kd_      = kd;
    dmSendcmd_[i].ff_      = ff * directionMotor_[i];
      // std::lock_guard<std::mutex> lk(cmd_mtx_);
      // pos_des = cmd_pos_[i];
      // vel_des = cmd_vel_[i];
      // kp      = cmd_kp_[i];
      // kd      = cmd_kd_[i];
      // ff      = cmd_tau_[i];
  }

  // 写入下位机
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    motorsInterface->fresh_cmd_motor_data(dmSendcmd_[i].pos_des_,
                                          dmSendcmd_[i].vel_des_,
                                          dmSendcmd_[i].ff_,
                                          dmSendcmd_[i].kp_,
                                          dmSendcmd_[i].kd_, i);
  }
  motorsInterface->send_motor_data();

  // 简单回显（可选）
  std_msgs::Float64MultiArray a;
  a.data.resize(NUM_JOINTS);
  for (int i=0;i<NUM_JOINTS;++i) a.data[i] = dmSendcmd_[i].pos_des_;
  cmd_pos_pub_.publish(a);
  // std_msgs::Float64MultiArray msg;
  // msg.data = cmd_pos_;
  // cmd_pos_pub_.publish(msg);
  // ROS_INFO_STREAM("cmd_pos = " << a.data[0] << ", " << a.data[1] << ", ...");
}

bool DmHW::setupJoints()
{
  const int JOINTS_PER_LEG = 7;
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index=-1, joint_index=-1;
    if (joint.first.find("leg_l") != std::string::npos)      leg_index = 0;
    else if (joint.first.find("leg_r") != std::string::npos) leg_index = 1;
    else continue;

    if      (joint.first.find("1_joint") != std::string::npos) joint_index = 0;
    else if (joint.first.find("2_joint") != std::string::npos) joint_index = 1;
    else if (joint.first.find("3_joint") != std::string::npos) joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos) joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos) joint_index = 4;
    else if (joint.first.find("6_joint") != std::string::npos) joint_index = 5; // 新增
    else if (joint.first.find("7_joint") != std::string::npos) joint_index = 6; // 新增
    else continue;

    int index = leg_index * JOINTS_PER_LEG + joint_index;

    hardware_interface::JointStateHandle state_handle(
      joint.first, &jointData_[index].pos_, &jointData_[index].vel_, &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);

    hybridJointInterface_.registerHandle(HybridJointHandle(
      state_handle,
      &jointData_[index].pos_des_, &jointData_[index].vel_des_,
      &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool DmHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link",
      imuData_.ori, imuData_.ori_cov,
      imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc,  imuData_.linear_acc_cov));

  imuData_.ori_cov[0] = 0.0012; imuData_.ori_cov[4] = 0.0012; imuData_.ori_cov[8] = 0.0012;
  imuData_.angular_vel_cov[0] = 0.0004; imuData_.angular_vel_cov[4] = 0.0004; imuData_.angular_vel_cov[8] = 0.0004;
  return true;
}

void DmHW::hybridCmdCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != 5 * NUM_JOINTS) {
    ROS_WARN_THROTTLE(1.0, "[DmHW] /hybrid_cmd 长度应为%zu(%dpos+%dvel+%dkp+%dkd+%dtau)，当前=%zu",
                    5ul*NUM_JOINTS, NUM_JOINTS, NUM_JOINTS, NUM_JOINTS, NUM_JOINTS, NUM_JOINTS,
                    msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  const auto& d = msg->data;
  for (int i=0;i<NUM_JOINTS;++i) cmd_pos_[i] = d[0*NUM_JOINTS + i];
  for (int i=0;i<NUM_JOINTS;++i) cmd_vel_[i] = d[1*NUM_JOINTS + i];
  for (int i=0;i<NUM_JOINTS;++i) cmd_kp_[i]  = d[2*NUM_JOINTS + i];
  for (int i=0;i<NUM_JOINTS;++i) cmd_kd_[i]  = d[3*NUM_JOINTS + i];
  for (int i=0;i<NUM_JOINTS;++i) cmd_tau_[i] = d[4*NUM_JOINTS + i];

  last_cmd_stamp_ = ros::Time::now();
}

void DmHW::emgCb(const std_msgs::Float32::ConstPtr& msg)
{
  emergency_stop_ = (msg->data > 0.5);
  if (emergency_stop_) ROS_ERROR("[DmHW] 接收到急停！");
}

} // namespace legged
