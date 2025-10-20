#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include <std_msgs/UInt8MultiArray.h>
#include "test_led/test_led.h"

void ledCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    if(msg->data.size() < 6) { // 必须至少包含 mode+G+R+B+brightness+speed
        ROS_WARN("LED command frame too short");
        return;
    }

    uint8_t mode = msg->data[0];

    // 颜色 GRB
    uint32_t color = (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];

    uint8_t brightness = msg->data[4];
    uint8_t speed = msg->data[5];

    // 发送 LED 命令到 STM32
    writeLedcmd(mode, color, brightness, speed);

    ROS_INFO("LED cmd: mode=%d, color=0x%06X, brightness=%d, speed=%d", 
             mode, color, brightness, speed);
    
}   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "public_node");
    ros::NodeHandle nh("~");

    // 串口初始化
    serialInit();

    // 订阅 /led_mode 话题
    ros::Subscriber sub = nh.subscribe("/led_mode", 10, ledCallback);

    ros::spin();
    return 0;
}
