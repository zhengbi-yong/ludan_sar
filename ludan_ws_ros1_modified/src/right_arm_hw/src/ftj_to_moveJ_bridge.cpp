#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/bind.hpp>  // for boost::bind, _1

#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <cmath>

static const std::vector<std::string> kRightJointsDefault = {
  "leg_r1_joint","leg_r2_joint","leg_r3_joint",
  "leg_r4_joint","leg_r5_joint","leg_r6_joint","leg_r7_joint"
};

class FtjToMoveJBridge
{
public:
  using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;

  // ★ 注意：类内声明时不要带 “FtjToMoveJBridge::”
  FtjToMoveJBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  static std::string vecToStr(const std::vector<std::string>& v)
  {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) { s += v[i]; if (i + 1 < v.size()) s += ", "; }
    s += "]";
    return s;
  }

  ros::NodeHandle nh_, pnh_;
  Server as_;
  ros::Publisher pub_movej_;
  std::vector<std::string> right_joints_;
  int stream_rate_hz_{200};
};

// ★ 类外实现：这里才使用 “FtjToMoveJBridge::”
FtjToMoveJBridge::FtjToMoveJBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh),
  // 兼容没有 registerExecuteCallback 的系统：构造时直接传 execute 回调
  as_(nh_, "follow_joint_trajectory",
      boost::bind(&FtjToMoveJBridge::executeCB, this, _1), false)
{
  pnh_.param("stream_rate", stream_rate_hz_, 200);
  if (!pnh_.getParam("joint_names", right_joints_)) {
    right_joints_ = kRightJointsDefault;
  }
  pub_movej_ = nh_.advertise<std_msgs::Float64MultiArray>("command_moveJ", 1);

  as_.start();
  ROS_INFO_STREAM("FTJ→moveJ C++ bridge ready. Action: "
                  << nh_.getNamespace() << "/follow_joint_trajectory, "
                  << "topic out: " << pub_movej_.getTopic()
                  << ", stream_rate=" << stream_rate_hz_ << " Hz");
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

  // 关节名映射
  std::map<std::string, int> name2idx;
  for (size_t i = 0; i < traj.joint_names.size(); ++i) name2idx[traj.joint_names[i]] = static_cast<int>(i);

  std::vector<int> order; order.reserve(right_joints_.size());
  try {
    for (const auto& jn : right_joints_) order.push_back(name2idx.at(jn));
  } catch (const std::out_of_range&) {
    ROS_ERROR_STREAM("Goal joints mismatch. Need (in order): " << vecToStr(right_joints_)
                     << " , but got: " << vecToStr(traj.joint_names));
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    as_.setAborted(result);
    return;
  }

  const size_t P = traj.points.size();
  std::vector<double> t_list(P, 0.0);
  std::vector<std::vector<double>> q_list(P, std::vector<double>(right_joints_.size(), 0.0));

  for (size_t p = 0; p < P; ++p) {
    t_list[p] = traj.points[p].time_from_start.toSec();
    if (t_list[p] < 0.0) {
      ROS_ERROR("Negative time_from_start at point %zu", p);
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setAborted(result);
      return;
    }
    if (traj.points[p].positions.size() < traj.joint_names.size()) {
      ROS_ERROR("Trajectory point %zu missing positions.", p);
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setAborted(result);
      return;
    }
    for (size_t k = 0; k < right_joints_.size(); ++k) {
      q_list[p][k] = traj.points[p].positions[order[k]];
    }
  }

  if (t_list.back() <= 0.0) {
    ROS_ERROR("Final time_from_start must be > 0.");
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    as_.setAborted(result);
    return;
  }

  ros::Rate r(stream_rate_hz_);
  const ros::Time t0 = ros::Time::now();

  std_msgs::Float64MultiArray out;
  out.data.resize(right_joints_.size(), 0.0);
  fb.joint_names = right_joints_;

  while (ros::ok()) {
    if (as_.isPreemptRequested()) {
      ROS_WARN("Goal preempted.");
      as_.setPreempted();
      return;
    }

    double t = (ros::Time::now() - t0).toSec();
    if (t >= t_list.back()) {
      for (size_t k = 0; k < right_joints_.size(); ++k) out.data[k] = q_list.back()[k];
      pub_movej_.publish(out);

      fb.desired.positions = out.data;
      fb.actual.positions  = out.data;    // 如需真实反馈可订阅 joint_states
      fb.error.positions.assign(right_joints_.size(), 0.0);
      as_.publishFeedback(fb);

      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      as_.setSucceeded(result);
      return;
    }

    // 找区间并线性插值
    size_t j = 1;
    while (j < P && t_list[j] < t) ++j;
    if (j >= P) j = P - 1;

    const double t0p = t_list[j-1];
    const double t1p = t_list[j];
    const double denom = std::max(1e-9, (t1p - t0p));
    const double s = std::min(1.0, std::max(0.0, (t - t0p) / denom));

    for (size_t k = 0; k < right_joints_.size(); ++k) {
      const double q0 = q_list[j-1][k];
      const double q1 = q_list[j][k];
      out.data[k] = q0 + s * (q1 - q0);
    }
    pub_movej_.publish(out);

    fb.desired.positions = out.data;
    fb.actual.positions  = out.data;
    fb.error.positions.assign(right_joints_.size(), 0.0);
    as_.publishFeedback(fb);

    r.sleep();
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
