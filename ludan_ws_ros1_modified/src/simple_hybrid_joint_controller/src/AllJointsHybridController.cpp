#include "simple_hybrid_joint_controller/AllJointsHybridController.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace simple_hjc {

// 如果未在参数里给 joints，就按这个默认（可改成 14 路）
static const std::vector<std::string> kDefaultJointNames = {
  "leg_l1_joint","leg_l2_joint","leg_l3_joint","leg_l4_joint","leg_l5_joint","leg_l6_joint","leg_l7_joint",
  "leg_r1_joint","leg_r2_joint","leg_r3_joint","leg_r4_joint","leg_r5_joint","leg_r6_joint","leg_r7_joint"
};

bool AllJointsHybridController::init(legged::HybridJointInterface* hw, ros::NodeHandle& nh) {
  // 关节列表：从参数读取；没有就用默认（建议在 launch 里明确传入 14 个）
  if (!nh.getParam("joints", joint_names_)) {
    joint_names_ = kDefaultJointNames;
    ROS_WARN("param '~joints' not set, using default (%zu)", joint_names_.size());
  }
  N_ = joint_names_.size();
  if (N_ == 0) {
    ROS_ERROR("No joints configured.");
    return false;
  }

  nh.param("default_kp", default_kp_, 20.0);
  nh.param("default_kd", default_kd_, 1.0);
  nh.param("default_vel", default_vel_, 0.0);
  nh.param("default_ff",  default_ff_,  0.0);

  handles_.reserve(N_);
  try {
    for (const auto& name : joint_names_) {
      handles_.push_back(hw->getHandle(name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("Get handle failed: " << e.what());
    return false;
  }

  cmd_.assign(N_, {0,0,0,0,0});

  // 订阅三种命令
  sub_same_   = nh.subscribe("command_same",   1, &AllJointsHybridController::sameCb,   this);
  sub_pos_all_= nh.subscribe("command_pos_all",1, &AllJointsHybridController::posAllCb, this);
  sub_matrix_ = nh.subscribe("command_matrix", 1, &AllJointsHybridController::matrixCb, this);
  sub_one_    = nh.subscribe("command_one",    5, &AllJointsHybridController::oneCb,    this);
  sub_move_j_ = nh.subscribe("command_moveJ",  1, &AllJointsHybridController::moveJCb,  this);

  ROS_INFO("AllJointsHybridController ready with %zu joints.", N_);
  return true;
}

void AllJointsHybridController::starting(const ros::Time&) {
  // 将起始目标设为当前位置，避免突跳
  for (size_t i = 0; i < N_; ++i) {
    cmd_[i][0] = handles_[i].getPosition(); // pos
    cmd_[i][1] = default_vel_;              // vel
    cmd_[i][2] = default_kp_;               // kp
    cmd_[i][3] = default_kd_;               // kd
    cmd_[i][4] = default_ff_;               // ff
  }
}

void AllJointsHybridController::update(const ros::Time&, const ros::Duration&) {
  // 把缓存的命令写到 HybridJointHandle
  for (size_t i = 0; i < N_; ++i) {
    handles_[i].setCommand(cmd_[i][0], cmd_[i][1], cmd_[i][2], cmd_[i][3], cmd_[i][4]);
  }
}

void AllJointsHybridController::sameCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // 期望 [pos, vel, kp, kd, ff]，支持只给前几项
  double pos = cmd_[0][0], vel = default_vel_, kp = default_kp_, kd = default_kd_, ff = default_ff_;
  if (msg->data.size() >= 1) pos = msg->data[0];
  if (msg->data.size() >= 2) vel = msg->data[1];
  if (msg->data.size() >= 3) kp  = msg->data[2];
  if (msg->data.size() >= 4) kd  = msg->data[3];
  if (msg->data.size() >= 5) ff  = msg->data[4];
  for (size_t i = 0; i < N_; ++i) {
    cmd_[i][0]=pos; cmd_[i][1]=vel; cmd_[i][2]=kp; cmd_[i][3]=kd; cmd_[i][4]=ff;
  }
}

void AllJointsHybridController::posAllCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < N_) {
    ROS_WARN("command_pos_all expects %zu positions, got %zu", N_, msg->data.size());
    return;
  }
  for (size_t i = 0; i < N_; ++i) {
    cmd_[i][0] = msg->data[i];
    cmd_[i][1] = default_vel_;
    cmd_[i][2] = default_kp_;
    cmd_[i][3] = default_kd_;
    cmd_[i][4] = default_ff_;
  }
}

void AllJointsHybridController::matrixCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // 行优先：第 i 行是第 i 个关节，共 N 行，每行 5 个
  if (msg->data.size() < 5*N_) {
    ROS_WARN("command_matrix expects %zu values (%zux5), got %zu", 5*N_, N_, msg->data.size());
    return;
  }
  for (size_t i = 0; i < N_; ++i) {
    size_t base = i*5;
    cmd_[i][0]=msg->data[base+0];  // pos
    cmd_[i][1]=msg->data[base+1];  // vel
    cmd_[i][2]=msg->data[base+2];  // kp
    cmd_[i][3]=msg->data[base+3];  // kd
    cmd_[i][4]=msg->data[base+4];  // ff
  }
}
// data[0] = 关节索引（int，0..N-1）
// data[1..5] = [pos, vel, kp, kd, ff]，可缺省；若提供且是有限数则更新，否则保持原值
void AllJointsHybridController::oneCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) {
    ROS_WARN("command_one expects at least index and optionally 5 values");
    return;
  }
  int idx = static_cast<int>(std::lrint(msg->data[0]));
  if (idx < 0 || idx >= static_cast<int>(N_)) {
    ROS_WARN("command_one: bad joint index %d (valid 0..%zu)", idx, N_-1);
    return;
  }


  // 安全读取助手（缺省/越界 -> NaN）
  auto getOrNaN = [&](size_t k)->double{
    return (k < msg->data.size()) ? msg->data[k] : std::numeric_limits<double>::quiet_NaN();
  };
  double pos = getOrNaN(1);
  double vel = getOrNaN(2);
  double kp  = getOrNaN(3);
  double kd  = getOrNaN(4);
  double ff  = getOrNaN(5);

  // 仅对“给了且是有限数”的字段做更新
  if (std::isfinite(pos)) cmd_[idx][0] = pos;
  if (std::isfinite(vel)) cmd_[idx][1] = vel;
  if (std::isfinite(kp )) cmd_[idx][2] = kp;
  if (std::isfinite(kd )) cmd_[idx][3] = kd;
  if (std::isfinite(ff )) cmd_[idx][4] = ff;
}


void AllJointsHybridController::moveJCb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 7) {  // 确保至少有 7 个位置数据
    ROS_WARN("MOVE J expects 7 positions, got %zu", msg->data.size());
    return;
  }

  // 目标关节位置为从位置 7 到 13（即 idx 7 到 idx 13），假设电机编号从 0 开始
  for (size_t i = 7; i < 14; ++i) {
    cmd_[i][0] = msg->data[i - 7];  // 将输入的位置传递给对应的电机
    cmd_[i][1] = 0.0;  // 速度设为 0
    cmd_[i][2] = 5.0; // Kp 设置为 20
    cmd_[i][3] = 1.0;  // Kd 设置为 1
    cmd_[i][4] = 0.0;  // 力矩为 0
  }
}

} // namespace simple_hjc

PLUGINLIB_EXPORT_CLASS(simple_hjc::AllJointsHybridController, controller_interface::ControllerBase)
