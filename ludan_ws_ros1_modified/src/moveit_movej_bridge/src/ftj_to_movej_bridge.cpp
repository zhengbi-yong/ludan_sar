#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>

#include <xmlrpcpp/XmlRpcValue.h>
#include <boost/bind.hpp>

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cmath>

class FtjToMoveJBridge
{
public:
  using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;
  FtjToMoveJBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  static int  extractIndex(const std::string& name);      // 提取 r1/l1 中的数字
  static bool hwNameLess(const std::string& a, const std::string& b); // 用于排序

  ros::NodeHandle nh_, pnh_;
  Server as_;
  ros::Publisher pub_movej_;
  std::vector<std::string> joint_order_;               // 硬件顺序
  std::map<std::string, std::string> remap_;           // MoveIt → 硬件
  int stream_rate_hz_{200};
};

int FtjToMoveJBridge::extractIndex(const std::string& name)
{
  // 从 "..._r7_joint" 或 "..._l3_joint" 提取末尾数字；找不到则返回大数
  int n = -1;
  for (int i = static_cast<int>(name.size()) - 1; i >= 0; --i) {
    if (std::isdigit(name[i])) {
      int j = i;
      while (j >= 0 && std::isdigit(name[j])) --j;
      try {
        n = std::stoi(name.substr(j + 1, i - j));
      } catch (...) {
        n = 999;
      }
      break;
    }
  }
  return (n >= 0) ? n : 999;
}
bool FtjToMoveJBridge::hwNameLess(const std::string& a, const std::string& b)
{
  // 先按 l/r 字母，再按数字排序，保证 l1<l2<...<l7 且 r1<r2<...<r7
  char ca='z', cb='z';
  for (char c : a) { if (c=='l' || c=='r') { ca=c; break; } }
  for (char c : b) { if (c=='l' || c=='r') { cb=c; break; } }
  if (ca != cb) return ca < cb;
  int ia = extractIndex(a), ib = extractIndex(b);
  if (ia != ib) return ia < ib;
  return a < b;
}

FtjToMoveJBridge::FtjToMoveJBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh),
  as_(nh_, "follow_joint_trajectory",
      boost::bind(&FtjToMoveJBridge::executeCB, this, _1), false)
{
  pnh_.param("stream_rate", stream_rate_hz_, 200);

  // 读取映射参数（支持直接扁平 key:value，也支持一层嵌套）
  XmlRpc::XmlRpcValue remap_yaml;
  if (pnh_.getParam("joint_remap", remap_yaml) &&
      remap_yaml.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    // 可能是扁平： joint_remap: { A: x, B: y }
    // 也可能是嵌套： joint_remap: { right_arm: {A:x...}, left_arm:{...} }
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = remap_yaml.begin();
         it != remap_yaml.end(); ++it)
    {
      const std::string key = static_cast<std::string>(it->first);
      const XmlRpc::XmlRpcValue& val = it->second;

      if (val.getType() == XmlRpc::XmlRpcValue::TypeString) {
        remap_[key] = static_cast<std::string>(val);
      } else if (val.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        remap_[key] = std::to_string(static_cast<int>(val));
      } else if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        remap_[key] = std::to_string(static_cast<double>(val));
      } else if (val.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        // 展开一层嵌套
        for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator jt = val.begin();
             jt != val.end(); ++jt)
        {
          const std::string k2 = static_cast<std::string>(jt->first);
          const XmlRpc::XmlRpcValue& v2 = jt->second;
          if (v2.getType() == XmlRpc::XmlRpcValue::TypeString) {
            remap_[k2] = static_cast<std::string>(v2);
          } else if (v2.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            remap_[k2] = std::to_string(static_cast<int>(v2));
          } else if (v2.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            remap_[k2] = std::to_string(static_cast<double>(v2));
          }
        }
      }
    }
  } else {
    ROS_WARN("~joint_remap param missing or not a map.");
  }

  // 由 remap_ 的 “值”(硬件关节名) 构建稳定的硬件顺序
  {
    std::vector<std::string> uniq;
    uniq.reserve(remap_.size());
    for (const auto& kv : remap_) uniq.push_back(kv.second);
    std::sort(uniq.begin(), uniq.end(), hwNameLess);
    uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());
    joint_order_ = uniq;
  }

  // 打印加载结果
  ROS_INFO_STREAM("Loaded joint_remap (" << remap_.size() << "):");
  for (const auto& kv : remap_) {
    ROS_INFO_STREAM("  " << kv.first << "  ->  " << kv.second);
  }
  ROS_INFO_STREAM("Hardware joint order (" << joint_order_.size() << "):");
  for (size_t i=0;i<joint_order_.size();++i) {
    ROS_INFO_STREAM("  [" << i << "] " << joint_order_[i]);
  }

  pub_movej_ = nh_.advertise<std_msgs::Float64MultiArray>("command_moveJ", 1);
  as_.start();

  ROS_INFO_STREAM("FTJ→moveJ bridge ready. pub: "
                  << pub_movej_.getTopic()
                  << " @ " << stream_rate_hz_ << " Hz");
}

void FtjToMoveJBridge::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;
  control_msgs::FollowJointTrajectoryFeedback fb;

  const auto& traj = goal->trajectory;
  if (traj.joint_names.empty() || traj.points.empty()) {
    ROS_ERROR("Empty joint_names or points in trajectory goal.");
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result);
    return;
  }

  // MoveIt → 硬件索引映射
  std::map<std::string, int> name2idx;
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    std::string name = traj.joint_names[i];
    if (remap_.count(name)) name = remap_[name];
    name2idx[name] = static_cast<int>(i);
  }

  // 组装按硬件顺序的轨迹
  const size_t P = traj.points.size();
  std::vector<double> t_list(P, 0.0);
  std::vector<std::vector<double>> q_list(P, std::vector<double>(joint_order_.size(), 0.0));

  for (size_t p = 0; p < P; ++p) {
    t_list[p] = traj.points[p].time_from_start.toSec();
    if (traj.points[p].positions.size() < traj.joint_names.size()) {
      ROS_ERROR("Trajectory point %zu missing positions", p);
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setAborted(result);
      return;
    }
    for (size_t k = 0; k < joint_order_.size(); ++k) {
      const auto& jn = joint_order_[k];
      if (!name2idx.count(jn)) continue;              // 未出现在本次指令里就保持 0
      q_list[p][k] = traj.points[p].positions[name2idx[jn]];
    }
  }

  ros::Rate rate(stream_rate_hz_);
  const ros::Time t0 = ros::Time::now();
  std_msgs::Float64MultiArray out;
  out.data.resize(joint_order_.size(), 0.0);
  fb.joint_names = joint_order_;

  while (ros::ok()) {
    if (as_.isPreemptRequested()) {
      ROS_WARN("Goal preempted.");
      as_.setPreempted();
      return;
    }

    const double t = (ros::Time::now() - t0).toSec();
    if (t >= t_list.back()) {
      for (size_t k = 0; k < joint_order_.size(); ++k) out.data[k] = q_list.back()[k];
      pub_movej_.publish(out);
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      as_.setSucceeded(result);
      return;
    }

    size_t j = 1;
    while (j < P && t_list[j] < t) ++j;
    if (j >= P) j = P - 1;

    const double t0p = t_list[j-1], t1p = t_list[j];
    const double s = std::min(1.0, std::max(0.0, (t - t0p) / std::max(1e-6, (t1p - t0p))));
    for (size_t k = 0; k < joint_order_.size(); ++k)
      out.data[k] = q_list[j-1][k] + s * (q_list[j][k] - q_list[j-1][k]);

    pub_movej_.publish(out);
    fb.desired.positions = out.data;
    fb.actual.positions  = out.data;                  // 如需真实反馈，可订阅 joint_states
    fb.error.positions.assign(joint_order_.size(), 0.0);
    as_.publishFeedback(fb);
    rate.sleep();
  }

  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  as_.setAborted(result, "Node shutdown");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ftj_to_moveJ_bridge");
  ros::NodeHandle nh, pnh("~");
  FtjToMoveJBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
