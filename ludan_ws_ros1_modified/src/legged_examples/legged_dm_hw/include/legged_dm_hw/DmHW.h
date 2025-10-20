#pragma once

#include <legged_hw/LeggedHW.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>

#include <mutex>
#include <array>
#include <vector>

#include <memory>
static constexpr int NUM_JOINTS = 14;
namespace legged
{

struct DmMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
};

struct DmImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class DmHW : public LeggedHW
{
public:
  DmHW() = default;
  ~DmHW() override = default;

  // 覆盖基类接口
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  // IMU回调（使用外部 /imu/data）
  void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom)
  {
    yesenceIMU_ = *odom;
  }

private:
  // 内部初始化
  bool setupJoints();
  bool setupImu();

  // 话题覆盖回调
  void hybridCmdCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void emgCb(const std_msgs::Float32::ConstPtr& msg);

  // 发布/订阅器
  ros::Publisher cmd_pos_pub_, cmd_vel_pub_, cmd_ff_pub_, read_pos_pub_, read_vel_pub_, read_ff_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber hybrid_cmd_sub_, emg_sub_;

  // 设备与配置
  DmMotorData jointData_[NUM_JOINTS]{};
  DmImuData imuData_{};
  int powerLimit_{0};
  int contactThreshold_{0};
  bool estimateContact_[4]{false, false, false, false};

  // 下发缓冲（电机方向映射）
  DmMotorData dmSendcmd_[NUM_JOINTS];
  const std::vector<int> directionMotor_{ 
    1, 1, -1, 1, 1,   -1, 1,    // 左腿 0..6
    -1, -1, -1, 1, 1, -1, 1     // 右腿 7..13 例子，按需改
  };

  // 外部IMU缓存
  sensor_msgs::Imu yesenceIMU_;

  // 手动覆盖相关
  std::mutex cmd_mtx_;
  std::array<double,NUM_JOINTS> cmd_pos_{};
  std::array<double,NUM_JOINTS> cmd_vel_{};
  std::array<double,NUM_JOINTS> cmd_kp_{};
  std::array<double,NUM_JOINTS> cmd_kd_{};
  std::array<double,NUM_JOINTS> cmd_tau_{};
  ros::Time last_cmd_stamp_;
  bool emergency_stop_{false};
  bool manual_override_{false};
  double manual_cmd_timeout_{0.2};  // 秒
};

} // namespace legged
