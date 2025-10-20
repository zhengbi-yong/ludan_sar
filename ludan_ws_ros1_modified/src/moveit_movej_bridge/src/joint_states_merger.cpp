#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <XmlRpcValue.h>
#include <map>
#include <string>
#include <vector>
#include <algorithm>

class JointStatesMerger
{
public:
  JointStatesMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void jointCB(const sensor_msgs::JointState::ConstPtr& msg, const std::string& source);

  void loadRemapYAML(const std::string& ns, ros::NodeHandle& nh);
  std::string mapName(const std::string& hw_name);

  ros::Publisher pub_;
  sensor_msgs::JointState merged_;

  std::vector<ros::Subscriber> subs_;
  std::map<std::string, std::string> hw2moveit_; // 硬件名 → MoveIt名
};

JointStatesMerger::JointStatesMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  // 输出合并后的 joint_states
  pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  // 默认订阅的子系统（根据你的系统调整）
  std::vector<std::string> sources = {
    "/mcu_leftarm/joint_states",
    "/mcu_rightarm/joint_states",
    "/mcu_leftleg/joint_states",
    "/mcu_rightleg/joint_states",
    "/mcu_neck/joint_states"
  };

  for (const auto& topic : sources) {
  subs_.push_back(
    nh.subscribe<sensor_msgs::JointState>(
      topic, 10,
      boost::bind(&JointStatesMerger::jointCB, this, _1, topic)
    )
  );
}


  // 加载映射配置文件
  std::vector<std::string> maps = {
    "left_arm_joint_remap",
    "right_arm_joint_remap",
    "left_leg_joint_remap",
    "right_leg_joint_remap",
    "neck_joint_remap"
  };

  for (const auto& key : maps) {
    ros::NodeHandle nh_map("/joint_states_merger/" + key);
    loadRemapYAML(key, nh_map);
  }

  ROS_INFO("joint_states_merger running... loaded %lu mappings.", hw2moveit_.size());
}

void JointStatesMerger::loadRemapYAML(const std::string& ns, ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue remap;
  if (!nh.getParam("", remap)) {  // 当前 namespace 下直接取
    ROS_WARN("No remap found under namespace '%s'", ns.c_str());
    return;
  }

  if (remap.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_WARN("Invalid remap format under '%s'", ns.c_str());
    return;
  }

  for (auto it = remap.begin(); it != remap.end(); ++it) {
    std::string moveit_name = it->first;
    std::string hw_name = static_cast<std::string>(it->second);
    hw2moveit_[hw_name] = moveit_name;
    ROS_INFO_STREAM("Mapping: " << hw_name << " -> " << moveit_name);
  }
}

std::string JointStatesMerger::mapName(const std::string& hw_name)
{
  auto it = hw2moveit_.find(hw_name);
  if (it != hw2moveit_.end()) return it->second;
  return hw_name; // 若未匹配则保持原名
}
void JointStatesMerger::jointCB(const sensor_msgs::JointState::ConstPtr& msg,
                                const std::string& source)
{
  // 提取模块名，例如 "/mcu_rightarm/joint_states" -> "rightarm"
  std::string prefix = source;
  size_t pos = prefix.find("mcu_");
  if (pos != std::string::npos)
    prefix = prefix.substr(pos + 4);  // 去掉 "mcu_"
  pos = prefix.find("/joint_states");
  if (pos != std::string::npos)
    prefix = prefix.substr(0, pos);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& hw_name = msg->name[i];

    // 跳过左腿数据
    if (hw_name.find("leg_l") != std::string::npos)
      continue;

    // 组合唯一键名，防止覆盖，例如 "rightarm_leg_r1_joint"
    std::string scoped_hw = prefix + "_" + hw_name;
    
    if (hw2moveit_.find(scoped_hw) == hw2moveit_.end())
        continue;
    // 映射成 MoveIt 名称
    std::string name = mapName(scoped_hw);

    auto it = std::find(merged_.name.begin(), merged_.name.end(), name);
    if (it == merged_.name.end()) {
      merged_.name.push_back(name);
      merged_.position.push_back(msg->position.size() > i ? msg->position[i] : 0.0);
      merged_.velocity.push_back(msg->velocity.size() > i ? msg->velocity[i] : 0.0);
      merged_.effort.push_back(msg->effort.size() > i ? msg->effort[i] : 0.0);
    } else {
      size_t idx = std::distance(merged_.name.begin(), it);
      merged_.position[idx] = msg->position.size() > i ? msg->position[i] : 0.0;
      merged_.velocity[idx] = msg->velocity.size() > i ? msg->velocity[i] : 0.0;
      merged_.effort[idx] = msg->effort.size() > i ? msg->effort[i] : 0.0;
    }
  }

  merged_.header.stamp = ros::Time::now();
  pub_.publish(merged_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_merger");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JointStatesMerger merger(nh, pnh);
  ros::spin();
  return 0;
}
