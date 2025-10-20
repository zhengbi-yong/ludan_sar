#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_interface/controller.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace simple_hjc {

class AllJointsHybridController
  : public controller_interface::Controller<legged::HybridJointInterface> {
public:
  bool init(legged::HybridJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  std::vector<legged::HybridJointHandle> handles_;
  std::vector<std::string> joint_names_;
  size_t N_{0};  // 关节数 = joints.size()

  // 命令缓存：N x 5 (pos, vel, kp, kd, ff)
  std::vector<std::array<double,5>> cmd_;

  // 默认参数
  double default_kp_{5.0}, default_kd_{1.0}, default_vel_{0.0}, default_ff_{0.0};

  // 订阅者
  ros::Subscriber sub_same_, sub_pos_all_, sub_matrix_,sub_one_, sub_move_j_;

  void sameCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void posAllCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void matrixCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void oneCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void moveJCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
};

} // namespace simple_hjc
