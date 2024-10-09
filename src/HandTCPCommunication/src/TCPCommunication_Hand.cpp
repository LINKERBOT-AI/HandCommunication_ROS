
/*
  通讯处理类：Json处理数据 机械手
*/
#include "TCPCommunication_Hand.h"

namespace AIMOcommunicate
{
    TCPCommunication_Hand::TCPCommunication_Hand()
    {      
        Hand_TCPCommunication_Json_Launch_Deal();
        if(handEnable == true)
        {
            Hand_TCPCommunication_Init();  
            Hand_InitDataSend();         
        }
    }
    TCPCommunication_Hand::~TCPCommunication_Hand()
    {
        Hand_TCPCommunication_stop();
    }
    
    void TCPCommunication_Hand::Hand_TCPCommunication_stop(void)
    {
        if(m_hand_isConnected == true)
        {
            close(m_server_hand_sockfd);
            m_hand_isConnected = false;
        }
    }
    void TCPCommunication_Hand::Hand_TCPCommunication_Json_Launch_Deal(void)
    {
        m_server_hand_ip = "192.168.1.40";
        m_hand_tcp_nh.getParam("arm/hand_ip", m_server_hand_ip);
        m_server_hand_port = 20008;

        m_hand_tcp_nh.param<bool>("arm/handEnable", handEnable, false);
        m_hand_tcp_nh.param<int>("arm/Hand_Freedom", Hand_Freedom, 9);
        m_hand_tcp_nh.param<int>("arm/JointVelocity", JointVelocity, 10);
        m_hand_tcp_nh.param<int>("arm/JointEffort", JointEffort, 100);
    }
    
    void TCPCommunication_Hand::Hand_TCPCommunication_Init(void)
    {
        m_server_hand_sockfd = socket(AF_INET, SOCK_STREAM, 0);
        ROS_WARN(" --- TCPCommunication_Hand Init! ---");
        if(m_server_hand_sockfd < 0)
        {
            ROS_ERROR("Failed to create socket");
        }
        m_server_hand_addr.sin_family      = AF_INET;
        m_server_hand_addr.sin_addr.s_addr = inet_addr(m_server_hand_ip.c_str());
        m_server_hand_addr.sin_port        = htons(m_server_hand_port);
        // fcntl(m_server_hand_sockfd, F_SETFL, O_NONBLOCK);  // 非阻塞通讯
    }
    void TCPCommunication_Hand::Hand_InitDataSend(void)
    {
        HandClient_Send.Set_Force_RightHand.Joint1 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint2 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint3 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint4 = JointEffort;
        HandClient_Send.Set_Force_RightHand.Joint5 = JointEffort;

        HandClient_Send.Set_Force_LeftHand.Joint1 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint2 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint3 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint4 = JointEffort;
        HandClient_Send.Set_Force_LeftHand.Joint5 = JointEffort;

        HandClient_Send.Set_Speed_RightHand.Joint1          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint2          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint4          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint5          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint6          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint7          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint8          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint9          = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint1_rotation = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3_raw      = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint1_tip      = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint2_tip      = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint3_tip      = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint4_tip      = JointVelocity;
        HandClient_Send.Set_Speed_RightHand.Joint5_tip      = JointVelocity;

        HandClient_Send.Set_Speed_LeftHand.Joint1          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint2          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint4          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint5          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint6          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint7          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint8          = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint9          = JointVelocity; 
        HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3_raw      = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint1_tip      = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint2_tip      = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint3_tip      = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint4_tip      = JointVelocity;
        HandClient_Send.Set_Speed_LeftHand.Joint5_tip      = JointVelocity;   

        HandClient_Send.Set_Point_RightHand.Joint1          = 80;
        HandClient_Send.Set_Point_RightHand.Joint2          = 128;
        HandClient_Send.Set_Point_RightHand.Joint3          = 200;
        HandClient_Send.Set_Point_RightHand.Joint4          = 200;
        HandClient_Send.Set_Point_RightHand.Joint5          = 200;
        HandClient_Send.Set_Point_RightHand.Joint6          = 200;
        HandClient_Send.Set_Point_RightHand.Joint7          = 128;
        HandClient_Send.Set_Point_RightHand.Joint8          = 128;
        HandClient_Send.Set_Point_RightHand.Joint9          = 128;
        HandClient_Send.Set_Point_RightHand.Joint1_rotation = 0;
        HandClient_Send.Set_Point_RightHand.Joint3_raw      = 128;
        HandClient_Send.Set_Point_RightHand.Joint1_tip      = 0;
        HandClient_Send.Set_Point_RightHand.Joint2_tip      = 0;
        HandClient_Send.Set_Point_RightHand.Joint3_tip      = 0;
        HandClient_Send.Set_Point_RightHand.Joint4_tip      = 0;
        HandClient_Send.Set_Point_RightHand.Joint5_tip      = 0;

        HandClient_Send.Set_Point_LeftHand.Joint1          = 80;
        HandClient_Send.Set_Point_LeftHand.Joint2          = 128;
        HandClient_Send.Set_Point_LeftHand.Joint3          = 200;
        HandClient_Send.Set_Point_LeftHand.Joint4          = 200;
        HandClient_Send.Set_Point_LeftHand.Joint5          = 200;
        HandClient_Send.Set_Point_LeftHand.Joint6          = 200;
        HandClient_Send.Set_Point_LeftHand.Joint7          = 128;
        HandClient_Send.Set_Point_LeftHand.Joint8          = 128;
        HandClient_Send.Set_Point_LeftHand.Joint9          = 128; 
        HandClient_Send.Set_Point_LeftHand.Joint1_rotation = 0;
        HandClient_Send.Set_Point_LeftHand.Joint3_raw      = 128;
        HandClient_Send.Set_Point_LeftHand.Joint1_tip      = 0;
        HandClient_Send.Set_Point_LeftHand.Joint2_tip      = 0;
        HandClient_Send.Set_Point_LeftHand.Joint3_tip      = 0;
        HandClient_Send.Set_Point_LeftHand.Joint4_tip      = 0;
        HandClient_Send.Set_Point_LeftHand.Joint5_tip      = 0;
    }

    bool TCPCommunication_Hand::Hand_TCPCommunication_connect(const ros::NodeHandle& ros_TCP_nh)
    {
        int connect_status;
        m_hand_tcp_nh = ros_TCP_nh;
        connect_status = connect(m_server_hand_sockfd, reinterpret_cast<struct sockaddr*>(&m_server_hand_addr), sizeof(m_server_hand_addr));
        hand_TCPcommunicate_generalfun.TCP_Connect_error_deal(m_server_hand_sockfd, connect_status);

        m_hand_isConnected = true;
        m_hand_timer = ros_TCP_nh.createTimer(ros::Duration(0.02), &TCPCommunication_Hand::Hand_TCPSendRecvMessage, this);
        std::cout << " m_hand_timer is start " << std::endl;

        sub_right_hand  = m_hand_tcp_nh.subscribe("/right_hand_control", 10, &TCPCommunication_Hand::RightHandCtrl_subCallback, this);
        sub_left_hand   = m_hand_tcp_nh.subscribe("/left_hand_control", 10, &TCPCommunication_Hand::LeftHandCtrl_subCallback, this);
        pub_right_hand  = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/right_hand_states", 10);
        pub_left_hand   = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/left_hand_states", 10);
        pub_right_glove = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/right_glove_states", 10);
        pub_left_glove  = m_hand_tcp_nh.advertise<sensor_msgs::JointState>("/left_glove_states", 10);
    
        return true; 
    }
    
    ///////////////////////////////////////////////////////////////////////////
    void TCPCommunication_Hand::RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
        if(msg->position.size() == 16)
        {
            std::cout << "RightHandCtrl_subCallback " << std::endl;
            // 关节角度
            HandClient_Send.Set_Point_RightHand.Joint1 = msg->position[0];
            HandClient_Send.Set_Point_RightHand.Joint2 = msg->position[1];
            HandClient_Send.Set_Point_RightHand.Joint3 = msg->position[2];
            HandClient_Send.Set_Point_RightHand.Joint4 = msg->position[3];
            HandClient_Send.Set_Point_RightHand.Joint5 = msg->position[4];
            HandClient_Send.Set_Point_RightHand.Joint6 = msg->position[5];
            HandClient_Send.Set_Point_RightHand.Joint7 = msg->position[6];
            HandClient_Send.Set_Point_RightHand.Joint8 = msg->position[7];
            HandClient_Send.Set_Point_RightHand.Joint9 = msg->position[8];
            
            HandClient_Send.Set_Point_RightHand.Joint1_rotation = msg->position[9];
            
            HandClient_Send.Set_Point_RightHand.Joint3_raw = msg->position[10];
            HandClient_Send.Set_Point_RightHand.Joint1_tip = msg->position[11];
            HandClient_Send.Set_Point_RightHand.Joint2_tip = msg->position[12];
            HandClient_Send.Set_Point_RightHand.Joint3_tip = msg->position[13];
            HandClient_Send.Set_Point_RightHand.Joint4_tip = msg->position[14];
            HandClient_Send.Set_Point_RightHand.Joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: joint state size is not 16!");
        }

        // 速度值
        if(msg->velocity.size() < 16 && msg->velocity.size() > 0)
        {
            HandClient_Send.Set_Speed_RightHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint3 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint4 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint5 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint6 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint7 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint8 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint9 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            
            HandClient_Send.Set_Speed_RightHand.Joint1_rotation = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            
            HandClient_Send.Set_Speed_RightHand.Joint3_raw = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint1_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint3_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint4_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint5_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
        }
        else if(msg->velocity.size() >= 16)
        {
            HandClient_Send.Set_Speed_RightHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_RightHand.Joint2 = (msg->velocity[1] == 0) ? JointVelocity : msg->velocity[1];
            HandClient_Send.Set_Speed_RightHand.Joint3 = (msg->velocity[2] == 0) ? JointVelocity : msg->velocity[2];
            HandClient_Send.Set_Speed_RightHand.Joint4 = (msg->velocity[3] == 0) ? JointVelocity : msg->velocity[3];
            HandClient_Send.Set_Speed_RightHand.Joint5 = (msg->velocity[4] == 0) ? JointVelocity : msg->velocity[4];
            HandClient_Send.Set_Speed_RightHand.Joint6 = (msg->velocity[5] == 0) ? JointVelocity : msg->velocity[5];
            HandClient_Send.Set_Speed_RightHand.Joint7 = (msg->velocity[6] == 0) ? JointVelocity : msg->velocity[6];
            HandClient_Send.Set_Speed_RightHand.Joint8 = (msg->velocity[7] == 0) ? JointVelocity : msg->velocity[7];
            HandClient_Send.Set_Speed_RightHand.Joint9 = (msg->velocity[8] == 0) ? JointVelocity : msg->velocity[8];
            
            HandClient_Send.Set_Speed_RightHand.Joint1_rotation = (msg->velocity[9] == 0) ? JointVelocity : msg->velocity[9];
            
            HandClient_Send.Set_Speed_RightHand.Joint3_raw = (msg->velocity[10] == 0) ? JointVelocity : msg->velocity[10];
            HandClient_Send.Set_Speed_RightHand.Joint1_tip = (msg->velocity[11] == 0) ? JointVelocity : msg->velocity[11];
            HandClient_Send.Set_Speed_RightHand.Joint2_tip = (msg->velocity[12] == 0) ? JointVelocity : msg->velocity[12];
            HandClient_Send.Set_Speed_RightHand.Joint3_tip = (msg->velocity[13] == 0) ? JointVelocity : msg->velocity[13];
            HandClient_Send.Set_Speed_RightHand.Joint4_tip = (msg->velocity[14] == 0) ? JointVelocity : msg->velocity[14];
            HandClient_Send.Set_Speed_RightHand.Joint5_tip = (msg->velocity[15] == 0) ? JointVelocity : msg->velocity[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: velocity state size is warn!");
        }

        // 力度
        if(msg->effort.size() < 5 && msg->effort.size() > 0)
        {
            HandClient_Send.Set_Force_RightHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint2 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint3 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint4 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint5 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
        }
        else if(msg->effort.size() >= 5)
        {
            HandClient_Send.Set_Force_RightHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_RightHand.Joint2 = (msg->effort[1] == 0) ? JointEffort : msg->effort[1];
            HandClient_Send.Set_Force_RightHand.Joint3 = (msg->effort[2] == 0) ? JointEffort : msg->effort[2];
            HandClient_Send.Set_Force_RightHand.Joint4 = (msg->effort[3] == 0) ? JointEffort : msg->effort[3];
            HandClient_Send.Set_Force_RightHand.Joint5 = (msg->effort[4] == 0) ? JointEffort : msg->effort[4];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: effort state size is warn!");
        }
    }
    void TCPCommunication_Hand::LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
        if(msg->position.size() == 16)
        {
            std::cout << "LeftHandCtrl_subCallback " << std::endl;
            HandClient_Send.Set_Point_LeftHand.Joint1 = msg->position[0];
            HandClient_Send.Set_Point_LeftHand.Joint2 = msg->position[1];
            HandClient_Send.Set_Point_LeftHand.Joint3 = msg->position[2];
            HandClient_Send.Set_Point_LeftHand.Joint4 = msg->position[3];
            HandClient_Send.Set_Point_LeftHand.Joint5 = msg->position[4];
            HandClient_Send.Set_Point_LeftHand.Joint6 = msg->position[5];
            HandClient_Send.Set_Point_LeftHand.Joint7 = msg->position[6];
            HandClient_Send.Set_Point_LeftHand.Joint8 = msg->position[7];
            HandClient_Send.Set_Point_LeftHand.Joint9 = msg->position[8];
            
            HandClient_Send.Set_Point_LeftHand.Joint1_rotation = msg->position[9];
            
            HandClient_Send.Set_Point_LeftHand.Joint3_raw = msg->position[10];
            HandClient_Send.Set_Point_LeftHand.Joint1_tip = msg->position[11];
            HandClient_Send.Set_Point_LeftHand.Joint2_tip = msg->position[12];
            HandClient_Send.Set_Point_LeftHand.Joint3_tip = msg->position[13];
            HandClient_Send.Set_Point_LeftHand.Joint4_tip = msg->position[14];
            HandClient_Send.Set_Point_LeftHand.Joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("LeftHandCtrl_subCallback: joint state size is not 16!");
        } 

        // 速度值
        if(msg->velocity.size() < 16 && msg->velocity.size() > 0)
        {
            HandClient_Send.Set_Speed_LeftHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint3 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint4 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint5 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint6 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint7 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint8 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint9 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            
            HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            
            HandClient_Send.Set_Speed_LeftHand.Joint3_raw = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint1_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint3_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint4_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint5_tip = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
        }
        else if(msg->velocity.size() >= 16)
        {
            HandClient_Send.Set_Speed_LeftHand.Joint1 = (msg->velocity[0] == 0) ? JointVelocity : msg->velocity[0];
            HandClient_Send.Set_Speed_LeftHand.Joint2 = (msg->velocity[1] == 0) ? JointVelocity : msg->velocity[1];
            HandClient_Send.Set_Speed_LeftHand.Joint3 = (msg->velocity[2] == 0) ? JointVelocity : msg->velocity[2];
            HandClient_Send.Set_Speed_LeftHand.Joint4 = (msg->velocity[3] == 0) ? JointVelocity : msg->velocity[3];
            HandClient_Send.Set_Speed_LeftHand.Joint5 = (msg->velocity[4] == 0) ? JointVelocity : msg->velocity[4];
            HandClient_Send.Set_Speed_LeftHand.Joint6 = (msg->velocity[5] == 0) ? JointVelocity : msg->velocity[5];
            HandClient_Send.Set_Speed_LeftHand.Joint7 = (msg->velocity[6] == 0) ? JointVelocity : msg->velocity[6];
            HandClient_Send.Set_Speed_LeftHand.Joint8 = (msg->velocity[7] == 0) ? JointVelocity : msg->velocity[7];
            HandClient_Send.Set_Speed_LeftHand.Joint9 = (msg->velocity[8] == 0) ? JointVelocity : msg->velocity[8];
            
            HandClient_Send.Set_Speed_LeftHand.Joint1_rotation = (msg->velocity[9] == 0) ? JointVelocity : msg->velocity[9];
            
            HandClient_Send.Set_Speed_LeftHand.Joint3_raw = (msg->velocity[10] == 0) ? JointVelocity : msg->velocity[10];
            HandClient_Send.Set_Speed_LeftHand.Joint1_tip = (msg->velocity[11] == 0) ? JointVelocity : msg->velocity[11];
            HandClient_Send.Set_Speed_LeftHand.Joint2_tip = (msg->velocity[12] == 0) ? JointVelocity : msg->velocity[12];
            HandClient_Send.Set_Speed_LeftHand.Joint3_tip = (msg->velocity[13] == 0) ? JointVelocity : msg->velocity[13];
            HandClient_Send.Set_Speed_LeftHand.Joint4_tip = (msg->velocity[14] == 0) ? JointVelocity : msg->velocity[14];
            HandClient_Send.Set_Speed_LeftHand.Joint5_tip = (msg->velocity[15] == 0) ? JointVelocity : msg->velocity[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: velocity state size is warn!");
        }

        // 力度
        if(msg->effort.size() < 5 && msg->effort.size() > 0)
        {
            HandClient_Send.Set_Force_LeftHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint2 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint3 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint4 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint5 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
        }
        else if(msg->effort.size() >= 5)
        {
            HandClient_Send.Set_Force_LeftHand.Joint1 = (msg->effort[0] == 0) ? JointEffort : msg->effort[0];
            HandClient_Send.Set_Force_LeftHand.Joint2 = (msg->effort[1] == 0) ? JointEffort : msg->effort[1];
            HandClient_Send.Set_Force_LeftHand.Joint3 = (msg->effort[2] == 0) ? JointEffort : msg->effort[2];
            HandClient_Send.Set_Force_LeftHand.Joint4 = (msg->effort[3] == 0) ? JointEffort : msg->effort[3];
            HandClient_Send.Set_Force_LeftHand.Joint5 = (msg->effort[4] == 0) ? JointEffort : msg->effort[4];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: effort state size is warn!");
        }
    }
    
    void TCPCommunication_Hand::RightHand_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "right_hand";
        joint_state.name.resize(16);
        joint_state.position.resize(16); 
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1; 
        joint_state.position[1]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint2;
        joint_state.position[2]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3;
        joint_state.position[3]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint4;
        joint_state.position[4]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint5;
        joint_state.position[5]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint6;
        joint_state.position[6]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint7;
        joint_state.position[7]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint8;
        joint_state.position[8]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint9;
        joint_state.position[9]  = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_RightHand.Joint5_tip;

        joint_state.effort[0] = HandClient_Recv.Get_CurrentForce_RightHand.Joint1;
        joint_state.effort[1] = HandClient_Recv.Get_CurrentForce_RightHand.Joint2;
        joint_state.effort[2] = HandClient_Recv.Get_CurrentForce_RightHand.Joint3;
        joint_state.effort[3] = HandClient_Recv.Get_CurrentForce_RightHand.Joint4;
        joint_state.effort[4] = HandClient_Recv.Get_CurrentForce_RightHand.Joint5;       
                    
        pub_right_hand.publish(joint_state);
    }
    void TCPCommunication_Hand::LeftHand_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "left_hand"; 
        joint_state.name.resize(16);
        joint_state.position.resize(16); 
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1; 
        joint_state.position[1]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint2;
        joint_state.position[2]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3;
        joint_state.position[3]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint4;
        joint_state.position[4]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint5;
        joint_state.position[5]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint6;
        joint_state.position[6]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint7;
        joint_state.position[7]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint8;
        joint_state.position[8]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint9;
        joint_state.position[9]  = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_LeftHand.Joint5_tip; 

        joint_state.effort[0] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint1;
        joint_state.effort[1] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint2;
        joint_state.effort[2] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint3;
        joint_state.effort[3] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint4;
        joint_state.effort[4] = HandClient_Recv.Get_CurrentForce_LeftHand.Joint5;        
                    
        pub_left_hand.publish(joint_state);
    }
    
    void TCPCommunication_Hand::RightGlove_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "right_glove";
        joint_state.name.resize(16);
        joint_state.position.resize(16);  

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1; 
        joint_state.position[1]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint2;
        joint_state.position[2]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3;
        joint_state.position[3]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint4;
        joint_state.position[4]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint5;
        joint_state.position[5]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint6;
        joint_state.position[6]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint7;
        joint_state.position[7]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint8;
        joint_state.position[8]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint9;
        joint_state.position[9]  = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_RightGlove.Joint5_tip;     
                    
        pub_right_glove.publish(joint_state);
    }
    void TCPCommunication_Hand::LeftGlove_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "left_glove";
        joint_state.name.resize(16);
        joint_state.position.resize(16);  

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1; 
        joint_state.position[1]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint2;
        joint_state.position[2]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3;
        joint_state.position[3]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint4;
        joint_state.position[4]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint5;
        joint_state.position[5]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint6;
        joint_state.position[6]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint7;
        joint_state.position[7]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint8;
        joint_state.position[8]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint9;
        joint_state.position[9]  = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1_rotation;
        joint_state.position[10] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3_raw;
        joint_state.position[11] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint1_tip;
        joint_state.position[12] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint2_tip;
        joint_state.position[13] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint3_tip;
        joint_state.position[14] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint4_tip;
        joint_state.position[15] = HandClient_Recv.Get_CurrentPoint_LeftGlove.Joint5_tip;     
                    
        pub_left_glove.publish(joint_state);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    void TCPCommunication_Hand::Hand_TCPSendRecvMessage(const ros::TimerEvent& event)
    {
        Hand_DataDeal_Struct(m_server_hand_sockfd); 
    }

    int TCPCommunication_Hand::Hand_DataDeal_Struct(const int connect_sockfd)
    {
        Hand_SendStructDataDeal(HandClient_Send, TCP_SendStruct_Data);
        if((send(connect_sockfd, &TCP_SendStruct_Data, sizeof(TCP_SendStruct_Data), 0)) < 0)
        {
            ROS_WARN("hand TCP send Hand_SendDataStr error!");
            return -1;
        }  

        if (recv(connect_sockfd, &TCP_RecvStruct_Data, sizeof(TCP_RecvStruct_Data), 0) < 0) 
        {
            ROS_WARN("hand TCP recv error!");
            return -1;
        }

        // std::cout << static_cast<int>(sizeof(TCP_RecvStruct_Data)) << std::endl;
        // std::cout << "TCP_RecvStruct_Data: " << TCP_RecvStruct_Data.Get_CurrentPoint_RightHand.Joint1 
        // << ", " << TCP_RecvStruct_Data.Get_CurrentPoint_RightHand.Joint2 << ", " << TCP_RecvStruct_Data.Get_CurrentPoint_RightHand.Joint3 
        // << ", " << TCP_RecvStruct_Data.Get_CurrentPoint_RightHand.Joint4 << ", " << TCP_RecvStruct_Data.Get_CurrentPoint_RightHand.Joint5
        // << std::endl;
        Hand_RecvStructDataDeal(TCP_RecvStruct_Data, HandClient_Recv);
        return 0;
    }
    void TCPCommunication_Hand::Hand_SendStructDataDeal(TCP_Hand_SendData handData, Hand_SendStruct& sendData)
    {
        sendData.PowerOn            = handData.PowerOn;
        sendData.Work_Mode          = handData.Work_Mode;
        sendData.Control_Mode       = handData.Control_Mode;
        sendData.Move_Control       = handData.Move_Control;  

        sendData.Set_Point_RightHand = handData.Set_Point_RightHand;
        sendData.Set_Speed_RightHand = handData.Set_Speed_RightHand;  

        sendData.Set_Point_LeftHand  = handData.Set_Point_LeftHand;
        sendData.Set_Speed_LeftHand  = handData.Set_Speed_LeftHand; 

        sendData.Set_Force_RightHand = handData.Set_Force_RightHand;  
        sendData.Set_Force_LeftHand  = handData.Set_Force_LeftHand; 
        
        sendData.Hand_Freedom        = Hand_Freedom;     
    }
    void TCPCommunication_Hand::Hand_RecvStructDataDeal(Hand_RecvStruct recvData, TCP_Hand_RecvData& handData)
    {   
        handData.PowerOn_State = recvData.PowerOn_State; 
        switch(recvData.Work_State)
        {
            case 1:  handData.Work_State = "Cart";  break;
            case 2:  handData.Work_State = "Point"; break;
            default: break;
        }
        switch(recvData.Control_State)
        {
            case 1:   handData.Control_State = "Homing";              break;
            case 2:   handData.Control_State = "QuickMove";           break;
            case 3:   handData.Control_State = "Prospect";            break;
            case 4:   handData.Control_State = "LinearInterpolation"; break;
            case 5:   handData.Control_State = "ClearError";          break;
            default: break;
        }
        
        switch(recvData.RightHand_MoveState)
        {
            case 1:  handData.RightHand_MoveState = "Static";  break;
            case 2:  handData.RightHand_MoveState = "Move";    break;
            case 3:  handData.RightHand_MoveState = "MoveEnd"; break;  
            default: break;
        }
        switch(recvData.LeftHand_MoveState)
        {
            case 1:  handData.LeftHand_MoveState = "Static";  break;
            case 2:  handData.LeftHand_MoveState = "Move";    break;
            case 3:  handData.LeftHand_MoveState = "MoveEnd"; break;
            default: break;
        }       
        handData.Error_State        = recvData.Error_State;
        
        handData.Get_CurrentPoint_RightHand = recvData.Get_CurrentPoint_RightHand;
        handData.Get_CurrentPoint_LeftHand  = recvData.Get_CurrentPoint_LeftHand;
        
        handData.Get_CurrentForce_RightHand = recvData.Get_CurrentForce_RightHand;
        handData.Get_CurrentForce_LeftHand  = recvData.Get_CurrentForce_LeftHand;

        handData.Get_CurrentPoint_RightGlove = recvData.Get_CurrentPoint_RightGlove;
        handData.Get_CurrentPoint_LeftGlove  = recvData.Get_CurrentPoint_LeftGlove;
        

        // std::cout << "recvData.Get_CurrentPoint_RightHand: " << recvData.Get_CurrentPoint_RightHand.Joint1 
        // << ", " << recvData.Get_CurrentPoint_RightHand.Joint2 << ", " << recvData.Get_CurrentPoint_RightHand.Joint3 
        // << ", " << recvData.Get_CurrentPoint_RightHand.Joint4 << ", " << recvData.Get_CurrentPoint_RightHand.Joint5
        // << std::endl;
    }   
   
}  // namespace AIMOcommunicate
