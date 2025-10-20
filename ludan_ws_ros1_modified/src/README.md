这个版本可以实现一套ros1 noetic控制多个mcu 目前是两个
启动的时候启动multi_bringup.launch就可以了
# 启动
当前版本启动多个mcu
roslaunch simple_hybrid_joint_controller multi_bringup.launch
# rostopic 测试
### 左右臂控制
#### id mcu_rightarm 7 8 9 10 11 12 13 
rostopic pub mcu_rightarm/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [13, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_leftarm 7 8 9 10 11 12 13
rostopic pub mcu_leftarm/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_neck 7 10 9
rostopic pub mcu_neck/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_rightleg 7 8 9 10 11 12
rostopic pub mcu_rightleg/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 1.0, 0.0, 5.0, 1.0, 0.0]"
#### id mcu_leftleg 7 8 9 10 11 12
rostopic pub mcu_leftleg/all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [10, 1.0, 0.0, 5.0, 1.0, 0.0]"
运动控制：
## 所有电机一起
rostopic pub -r 20 mcu_leftarm/all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [1.0, 0.0, 5.0, 1.0, 0.0]"
## 急停
rostopic pub -r 20 mcu_rightleg/all_joints_hjc/command_same std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
## 单个电机运动

### 速度控制 speed control
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 0, 0.2, 0, 1, 1]"
### 位置控制 pos control
rostopic pub /all_joints_hjc/command_one std_msgs/Float64MultiArray "data: [12, 1.0, 0.0, 1.0, 1.0, 0.0]"
### 高级控制MoveJ 
rostopic pub mcu_leftarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rostopic pub mcu_rightarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

rostopic pub mcu_rightarm/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
rostopic pub mcu_leftleg/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rostopic pub mcu_rightleg/all_joints_hjc/command_moveJ std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
### MoveIt
#### start MoveJ
roslaunch simple_hybrid_joint_controller bringup_real.launch 
#### start bridge
roslaunch right_arm_hw bringup.launch 
#### start moveit
roslaunch right_arm_moveit_config move_group.launch
#### start rviz
rosrun rviz rviz
#### set
     add Robotstate
     add MotionPlanning
     Joints set angle
     plan
     execute



# 其他注意事项
## 1. 打开端口的方法
rosparam set /port /dev/mcu_rightarm
rosparam set /baud 921600
rosparam get /port
rosparam get /baud
sudo chmod a+rw /dev/mcu_rightarm
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
## 2. 绑定串口为指定名字
输入
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial"
预期结果
        ludan@ludan:/etc/udev/rules.d$ udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial"
            ATTRS{serial}=="375539423233"
            ATTRS{idProduct}=="5740"
            ATTRS{idVendor}=="0483"
            ATTRS{idVendor}=="0bda"
            ATTRS{idProduct}=="5420"
            ATTRS{idVendor}=="1d6b"
            ATTRS{serial}=="3610000.xhci"
            ATTRS{idProduct}=="0002"
        ludan@ludan:/etc/udev/rules.d$ 
输入
ll /etc/udev/rules.d/99-stm32.rules
添加：
ATTRS{serial}=="375539423233"     # [LOG] 你的设备序列号：STM32芯片唯一标识
ATTRS{idProduct}=="5740"          # [LOG] 产品ID：STM32虚拟串口产品代码
ATTRS{idVendor}=="0483"           # [LOG] 厂商ID：STMicroelectronics (STM32制造商)
376A396C3233


3755394E3233
重新加载：
sudo udevadm control --reload-rules
sudo udevadm trigger

检查是否创建成功：
ll /dev/mcu_rightarm

rosrun joint_state_publisher_gui joint_state_publisher_gui
rosrun robot_state_publisher robot_state_publisher
rosrun rviz rviz -d $(rospack find ludan_moveit_config)/launch/moveit.rviz -f Body_Base
roslaunch ludan_moveit_config move_group.launch 
roslaunch moveit_movej_bridge bridge.launch 
roslaunch simple_hybrid_joint_controller multi_bringup.launch


rosrun moveit_commander moveit_commander_cmdline.py 
use right_arm
current
x = [12.665999984741211 0.11178779602050781 0.08525943756103516 0.07038211822509766 0.08907413482666016 -12.5]
plan x
execute