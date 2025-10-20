#include <test_led/test_led.h>

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyACM5");
boost::system::error_code err;

/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[1] = {0xEF};
const unsigned char header[1] = {0xEE};


uint8_t modeSend;
uint8_t modeRecv;

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：灯光控制的数据帧
入口参数：“mode+颜色+亮度+速度”
出口参数：
数据帧格式：
┌────────┬────────┬──────────┬───────────────┬────────┬────────┐
│ Header │ Length │ Mode     │ Data          │ CRC8   │  Tail  │
├────────┼────────┼──────────┼───────────────┼────────┼────────┤
│ 1字节   │ 1字节  │ 1字节     │ N字节         │ 1字节   │ 1字节  │
│ 0xEE   │ N+2    │ 功能码    │ 颜色+亮度+速度 │ 校验值  │ 0x0D   │
└────────┴────────┴──────────┴───────────────┴────────┴────────┘

********************************************************/

void writeLedcmd(uint8_t mode, uint32_t color, uint8_t brightness, uint8_t speed)    
{
    unsigned char buf[12];
    int index = 0;
    // 帧头
    buf[index++] = header[0];
    // 帧长度
    buf[index++] = 6;
    // 数据帧
    buf[index++] = mode; // 功能码
    buf[index++] = color >> 16; // G颜色
    buf[index++] = color >> 8; // B颜色
    buf[index++] = color; // R颜色
    buf[index++] = brightness; // 亮度
    buf[index++] = speed; // 速度
    // 计算校验值
    unsigned char crc = getCrc8(&buf[1], 7); 
    buf[index++] = crc; // 校验值
    // 帧尾
    buf[index++] = ender[0];

    std::stringstream ss;
for (int i = 0; i < index; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
}
ROS_INFO_STREAM("Send data: " << ss.str());

    // 串口发送一个字节
    boost::asio::write(sp, boost::asio::buffer(buf, index));
}


/**********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
**********************************************************/
bool readSpeed(uint8_t &mode)
{
    try
    {
        boost::asio::streambuf response;
        // 读取一行，以 \r\n 结尾
        boost::asio::read_until(sp, response, "\r\n");

        std::istream is(&response);
        std::string line;
        std::getline(is, line);

        // 打印调试
        ROS_INFO("STM32 says: %s", line.c_str());

        // 解析 "receive: %d"
        int data;
        if (sscanf(line.c_str(), "receive: %d", &data) == 1)
        {
            // 这里根据实际需要赋值
            mode = data;
            return true;
        }
        else
        {
            ROS_ERROR("Parse error: %s", line.c_str());
            return false;
        }
    }
    catch(boost::system::system_error &err)
    {
        ROS_ERROR("read_until error: %s", err.what());
        return false;
    }
}

/**********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
**********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

