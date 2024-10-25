
#ifndef CAN_GENERAL_FUNCTION
#define CAN_GENERAL_FUNCTION

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string.h>
#include <unistd.h>
#include <array>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>
 
#include "stdint.h"
#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <json/json.h> 

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <boost/asio.hpp>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>

#include <cstring>
#include <termios.h>
#include <serial/serial.h>  
#include <linux/can.h>
#include <linux/can/raw.h>  
#include <cstdlib>  // 包含 system 函数 
#include <sys/types.h>
#include "sensor_msgs/JointState.h" 

const float Pi       = 3.141596f;
const float DegToRad = Pi / 180.0;
const float RadToDeg = 180.0 / Pi;
 
using namespace sensor_msgs;
using namespace std; 
 
struct HumanHandParam  
{ 
    float Joint1;  // 拇指1 弯曲 0-255
    float Joint2;  // 拇指2 左右横摆
    float Joint3;  // 食指 弯曲 0-255
    float Joint4;  // 中指 弯曲 0-255
    float Joint5;  // 无名指 弯曲 0-255
    float Joint6;  // 小拇指 弯曲 0-255
    float Joint7;  // 食指 左右横摆  
    float Joint8;  // 无名指 左右横摆
    float Joint9;  // 小拇指 左右横摆

    float Joint1_rotation; // 拇指 旋转 —— 10自由度多余部分
    
    float Joint3_raw; // 中指 左右运动 —— 16自由度多余部分
    float Joint1_tip; // 拇指 末端弯曲度
    float Joint2_tip; // 食指 末端弯曲度
    float Joint3_tip; // 中指 末端弯曲度
    float Joint4_tip; // 无名指 末端弯曲度
    float Joint5_tip; // 小拇指 末端弯曲度
};
struct HandForceParam  
{
    float Joint1;   
    float Joint2;   
    float Joint3;   
    float Joint4;   
    float Joint5;    
}; 

struct ForceSensorData
{
    int SensorData1;
    int SensorData2;
    int SensorData3;
    int SensorData4;
    int SensorData5;
    int SensorData6;
};

namespace general
{
    class GeneralFunction
    {
    public:
        GeneralFunction(){};

        bool TCP_Connect_error_deal(const int connect_sockfd, const int connect_status)
        {
            if(connect_status < 0)
            {
                if(errno == EINPROGRESS || errno == EALREADY)
                {
                    fd_set write_fds;  // 等待套接字变为可写
                    FD_ZERO(&write_fds);
                    FD_SET(connect_sockfd, &write_fds);

                    timeval timeout;
                    timeout.tv_sec  = 5;  // 设置超时时间为5秒
                    timeout.tv_usec = 0;

                    int select_status = select(connect_sockfd + 1, NULL, &write_fds, NULL, &timeout);
                    if(select_status < 0)
                    {
                        std::cerr << "Error in select" << std::endl;
                        return false;
                    }
                    else if(select_status == 0)
                    {
                        std::cerr << "Connection timed out" << std::endl;
                        return false;
                    }
                    else
                    {
                        int       error_code;  // 检查套接字上的错误状态
                        socklen_t error_code_size = sizeof(error_code);
                        getsockopt(connect_sockfd, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size);
                        if(error_code == 0)
                        {
                            std::cout << "Connection successful" << std::endl;  // 连接成功，可以进行数据发送和接收
                        }
                        else
                        {
                            std::cerr << "Connection failed: " << strerror(error_code) << std::endl;
                            return false;
                        }
                    }
                }
                else
                {
                    if((errno != EINPROGRESS) && (errno != EALREADY))
                        std::cerr << "EINPROGRESS EALREADY Failed to connect" << std::endl;
                    false;
                }
            }
            else
            {
                std::cout << "Connection successful" << std::endl;
            }
            return true;
        };
        int TCP_Blocking_deal(const int connect_sockfd, unsigned char* send_buf, const uint16_t send_len, unsigned char* recv_buf, const uint16_t recv_len)
        {
            if((send(connect_sockfd, send_buf, send_len, 0)) < 0)
            {
                ROS_WARN("TCP send TCP_Blocking_deal error!");
                return -1;
            }
            if((recv(connect_sockfd, recv_buf, recv_len, 0)) < 0)
            {
                ROS_WARN("TCP recv TCP_Blocking_deal error!");
                return -1;
            }
            return 0;
        };
        int TCP_nonBlocking_deal(const int connect_sockfd, unsigned char* send_buf, const uint16_t send_len, unsigned char* recv_buf, const uint16_t recv_len)
        {
            unsigned char recv_buf_temp[recv_len];
            if((send(connect_sockfd, send_buf, send_len, 0)) < 0)
            {
                // ROS_WARN("TCP send TCP_nonBlocking_deal error!");
                return -1;
            }
            int byte_recv = recv(connect_sockfd, recv_buf_temp, recv_len, 0);
            if((byte_recv) < 0)
            {
                if(errno == EWOULDBLOCK)  // 没有可用数据
                {
                    // ROS_ERROR("receive data is EWOULDBLOCK");
                    return -1;
                }
                else if(errno == EAGAIN)  // 没有可用数据
                {
                    // ROS_ERROR("receive data is EAGAIN");
                    return -1;
                }
                else
                {
                    ROS_ERROR("Failed to receive data");
                    return -1;
                }
            }
            else if(byte_recv == 0)  // 连接关闭
            {
                ROS_ERROR("connect is close");
                return -1;
            }
            for(int i = 0; i < recv_len; i++)
            {
                recv_buf[i] = recv_buf_temp[i];
            }
            return 0;
        };
    
    };
}  // namespace general

#endif
