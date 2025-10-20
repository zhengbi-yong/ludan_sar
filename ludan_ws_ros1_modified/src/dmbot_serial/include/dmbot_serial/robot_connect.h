#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>

#include "ros/ros.h"
#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial/serial.h> 

#include <sensor_msgs/JointState.h>

//================ 电机数量统一开关 ================
static constexpr int NUM_MOTORS = 14;     // <- 这里改电机总数（现为 14）
//==============================================

//4310
#define P_MIN1 -12.5f
#define P_MAX1 12.5f
#define V_MIN1 -30.0f
#define V_MAX1 30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1 10.0f

//4340
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -10.0f
#define V_MAX2 10.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -28.0f
#define T_MAX2 28.0f

//6006
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -45.0f
#define V_MAX3 45.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -12.0f
#define T_MAX3 12.0f

//8006
#define P_MIN4 -12.5f
#define P_MAX4 12.5f
#define V_MIN4 -45.0f
#define V_MAX4 45.0f
#define KP_MIN4 0.0f
#define KP_MAX4 500.0f
#define KD_MIN4 0.0f
#define KD_MAX4 5.0f
#define T_MIN4 -20.0f
#define T_MAX4 20.0f 

//6248p
#define P_MIN5 -12.566f
#define P_MAX5 12.566f
#define V_MIN5 -20.0f
#define V_MAX5 20.0f
#define KP_MIN5 0.0f
#define KP_MAX5 500.0f
#define KD_MIN5 0.0f
#define KD_MAX5 5.0f
#define T_MIN5 -120.0f
#define T_MAX5 120.0f 

//10010l
#define P_MIN6 -12.5f
#define P_MAX6 12.5f
#define V_MIN6 -25.0f
#define V_MAX6 25.0f
#define KP_MIN6 0.0f
#define KP_MAX6 500.0f
#define KD_MIN6 0.0f
#define KD_MAX6 5.0f
#define T_MIN6 -200.0f
#define T_MAX6 200.0f 




#define SEND_DATA_CHECK   1          // 发送数据校验标志位
#define READ_DATA_CHECK   0          // 接收数据校验标志位
#define FRAME_HEADER      0X7B       // 帧头
#define FRAME_TAIL        0X7D       // 帧尾

//================= 协议长度（关键改动） =================
// 下位机→上位机一帧长度：1(帧头) + NUM_MOTORS*5(每电机5字节) + 1(校验)
#define RECEIVE_DATA_SIZE (1 + NUM_MOTORS*5 + 1)   // 14 电机时 = 72
// 上位机→下位机单电机命令长度：固定 11
#define SEND_DATA_SIZE    11
//=======================================================

// 上位机→下位机 数据结构
typedef struct
{
    uint8_t tx[SEND_DATA_SIZE];        
    unsigned char Frame_Tail; 
}send_data_t;

// 下位机→上位机 数据结构
typedef struct      
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    unsigned char Frame_Tail;
}rev_data_t;

typedef struct      
{
    std::string name;
    std::string type;// 电机型号
    int index;

    float pos;  
    float vel;  
    float tor; 
    int p_int;
    int v_int;
    int t_int;

    float pos_set; 
    float vel_set; 
    float tor_set;
    float kp;
    float kd;
}motor_data_t;

namespace dmbot_serial
{
  class robot
  {
    private:
        std::string robot_name, Serial_Type;

        int motor_seial_baud;
        std::string motor_serial_port;
        serial::Serial serial_motor; // 电机串口
        std::thread rec_thread;
        rev_data_t Receive_Data;

        ros::NodeHandle n;        
        ros::Publisher joint_state_pub;
        std::thread pub_thread;
        send_data_t Send_Data;

        //============== 这里从 10 改为 NUM_MOTORS ==============
        std::array<motor_data_t, NUM_MOTORS> motors;
        //=======================================================

    public:
        robot();
        ~robot();
        void init_motor_serial(); 
        void get_motor_data_thread();  // 串口接收线程

        void fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx);
        void send_motor_data();

        void get_motor_data(double &pos,double &vel,double &torque, int motor_idx);
        void publishJointStates(); 
        
        void dm4310_fbdata(motor_data_t& moto,uint8_t *data);
        void dm4340_fbdata(motor_data_t& moto,uint8_t *data);
        void dm6006_fbdata(motor_data_t& moto,uint8_t *data);
        void dm8006_fbdata(motor_data_t& moto,uint8_t *data);
        void dm6248p_fbdata(motor_data_t& moto,uint8_t *data);
        void dm10010l_fbdata(motor_data_t& moto,uint8_t *data);


        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);      
        int16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);

        std::atomic<bool> stop_thread_ ;
  };
}
#endif
