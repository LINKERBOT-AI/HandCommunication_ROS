
/*
  通讯处理类：can 总线通信 控制机械手
*/
#include "Can_Communication.h"

namespace AIMOcommunicate
{
    Can_Communication::Can_Communication()
    {      
        Can_Communication_LaunchRead(); 
        if(HandEnable == true)
        {
            Can_Communication_Init();
            Hand_InitDataRecv();
            Hand_InitDataSend();
        } 
    }
    Can_Communication::~Can_Communication()
    {
        Can_Communication_stop();
    }
    
    void Can_Communication::Can_Communication_stop(void)
    {
        if(m_hand_isConnected == true)
        { 
            close(m_socket_can);
            m_hand_isConnected = false;
        }
    }
    void Can_Communication::Can_Communication_LaunchRead(void)
    {
        m_hand_can_nh.param<bool>("arm/HandEnable", HandEnable, false);
        m_hand_can_nh.param<int>("arm/RightHandID", RightHandID, 39);
        m_hand_can_nh.param<int>("arm/LeftHandID", LeftHandID, 40);
        m_hand_can_nh.param<int>("arm/Hand_Freedom", Hand_Freedom, 9);
        m_hand_can_nh.param<int>("arm/JointVelocity", JointVelocity, 10);
        m_hand_can_nh.param<int>("arm/JointEffort", JointEffort, 100);  

        std::cout << "Can_Communication_LaunchRead : Hand_Freedom = " << static_cast<int>(Hand_Freedom) << std::endl;
        std::cout << "Can_Communication_LaunchRead : RightHandID = " << static_cast<int>(RightHandID) << std::endl;
        std::cout << "Can_Communication_LaunchRead : LeftHandID = " << static_cast<int>(LeftHandID) << std::endl;
        std::cout << "Can_Communication_LaunchRead : JointVelocity = " << static_cast<int>(JointVelocity) << std::endl;
        std::cout << "Can_Communication_LaunchRead : JointEffort = " << static_cast<int>(JointEffort) << std::endl;
    } 
    void Can_Communication::Can_Communication_Init(void)
    {
        m_socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 打开套接字
        if (m_socket_can < 0) 
        {
            ROS_ERROR("Can_Communication_Init Failed to create socket"); 
        } 
        std::cout << "Can_Communication_Init : Socket opened!" << std::endl;
    }
    void Can_Communication::Hand_InitDataSend(void)
    {
        RightHand_Send.pressure_1 = static_cast<uint8_t>(JointEffort);
        RightHand_Send.pressure_2 = static_cast<uint8_t>(JointEffort);
        RightHand_Send.pressure_3 = static_cast<uint8_t>(JointEffort);
        RightHand_Send.pressure_4 = static_cast<uint8_t>(JointEffort);
        RightHand_Send.pressure_5 = static_cast<uint8_t>(JointEffort);

        LeftHand_Send.pressure_1 = static_cast<uint8_t>(JointEffort);
        LeftHand_Send.pressure_2 = static_cast<uint8_t>(JointEffort);
        LeftHand_Send.pressure_3 = static_cast<uint8_t>(JointEffort);
        LeftHand_Send.pressure_4 = static_cast<uint8_t>(JointEffort);
        LeftHand_Send.pressure_5 = static_cast<uint8_t>(JointEffort);

        RightHand_Send.speed_1 = static_cast<uint8_t>(JointVelocity); 
        RightHand_Send.speed_2 = static_cast<uint8_t>(JointVelocity);
        RightHand_Send.speed_3 = static_cast<uint8_t>(JointVelocity);
        RightHand_Send.speed_4 = static_cast<uint8_t>(JointVelocity);
        RightHand_Send.speed_5 = static_cast<uint8_t>(JointVelocity);

        LeftHand_Send.speed_1 = static_cast<uint8_t>(JointVelocity); 
        LeftHand_Send.speed_2 = static_cast<uint8_t>(JointVelocity);
        LeftHand_Send.speed_3 = static_cast<uint8_t>(JointVelocity);
        LeftHand_Send.speed_4 = static_cast<uint8_t>(JointVelocity);
        LeftHand_Send.speed_5 = static_cast<uint8_t>(JointVelocity); 

        RightHand_Send.joint_angle_1 = 200;
        RightHand_Send.joint_angle_2 = 128;
        RightHand_Send.joint_angle_3 = 200;
        RightHand_Send.joint_angle_4 = 200;
        RightHand_Send.joint_angle_5 = 200;
        RightHand_Send.joint_angle_6 = 200;
        RightHand_Send.joint_angle_7 = 128;
        RightHand_Send.joint_angle_8 = 128;
        RightHand_Send.joint_angle_9 = 128;
        RightHand_Send.joint1_rotation = 200;
        RightHand_Send.joint3_raw = 128;
        RightHand_Send.joint1_tip = 200;
        RightHand_Send.joint2_tip = 200;
        RightHand_Send.joint3_tip = 200;
        RightHand_Send.joint4_tip = 200;
        RightHand_Send.joint5_tip = 200;

        LeftHand_Send.joint_angle_1 = 200;
        LeftHand_Send.joint_angle_2 = 128;
        LeftHand_Send.joint_angle_3 = 200;
        LeftHand_Send.joint_angle_4 = 200;
        LeftHand_Send.joint_angle_5 = 200;
        LeftHand_Send.joint_angle_6 = 200;
        LeftHand_Send.joint_angle_7 = 128;
        LeftHand_Send.joint_angle_8 = 128;
        LeftHand_Send.joint_angle_9 = 128;
        LeftHand_Send.joint1_rotation = 200;
        LeftHand_Send.joint3_raw = 128;
        LeftHand_Send.joint1_tip = 200;
        LeftHand_Send.joint2_tip = 200;
        LeftHand_Send.joint3_tip = 200;
        LeftHand_Send.joint4_tip = 200;
        LeftHand_Send.joint5_tip = 200;
    }
    void Can_Communication::Hand_InitDataRecv(void)
    {
        RightHand_Send.pressure_1 = 0;
        RightHand_Send.pressure_2 = 0;
        RightHand_Send.pressure_3 = 0;
        RightHand_Send.pressure_4 = 0;
        RightHand_Send.pressure_5 = 0;
        
        LeftHand_Send.pressure_1 = 0;
        LeftHand_Send.pressure_2 = 0;
        LeftHand_Send.pressure_3 = 0;
        LeftHand_Send.pressure_4 = 0;
        LeftHand_Send.pressure_5 = 0; 

        RightHand_Send.joint_angle_1 = 0;
        RightHand_Send.joint_angle_2 = 0;
        RightHand_Send.joint_angle_3 = 0;
        RightHand_Send.joint_angle_4 = 0;
        RightHand_Send.joint_angle_5 = 0;
        RightHand_Send.joint_angle_6 = 0;
        RightHand_Send.joint_angle_7 = 0;
        RightHand_Send.joint_angle_8 = 0;
        RightHand_Send.joint_angle_9 = 0;
        RightHand_Send.joint1_rotation = 0;
        RightHand_Send.joint3_raw = 0;
        RightHand_Send.joint1_tip = 0;
        RightHand_Send.joint2_tip = 0;
        RightHand_Send.joint3_tip = 0;
        RightHand_Send.joint4_tip = 0;
        RightHand_Send.joint5_tip = 0;

        LeftHand_Send.joint_angle_1 = 0;
        LeftHand_Send.joint_angle_2 = 0;
        LeftHand_Send.joint_angle_3 = 0;
        LeftHand_Send.joint_angle_4 = 0;
        LeftHand_Send.joint_angle_5 = 0;
        LeftHand_Send.joint_angle_6 = 0;
        LeftHand_Send.joint_angle_7 = 0;
        LeftHand_Send.joint_angle_8 = 0;
        LeftHand_Send.joint_angle_9 = 0;
        LeftHand_Send.joint1_rotation = 0;
        LeftHand_Send.joint3_raw = 0;
        LeftHand_Send.joint1_tip = 0;
        LeftHand_Send.joint2_tip = 0;
        LeftHand_Send.joint3_tip = 0;
        LeftHand_Send.joint4_tip = 0;
        LeftHand_Send.joint5_tip = 0;  
    }

    bool Can_Communication::Can_Communication_connect(const ros::NodeHandle& ros_nh)
    { 
        struct ifreq ifr = {0};
        strcpy(ifr.ifr_name, "can0");            // 绑定 socket 到 can0 接口
        ioctl(m_socket_can, SIOCGIFINDEX, &ifr); // 获取接口索引       
        
        // 设置 CAN 接口的波特率, 替代命令行命令 sudo ip link set can0 up type can bitrate 1000000
        // struct can_bittiming can_bt;             
        // can_bt.bitrate = 1000000;
        // struct ifreq ifr_bt = {0};
        // strcpy(ifr_bt.ifr_name, "can0");
        // ifr_bt.ifr_ifru.ifru_data = static_cast<void*>(&can_bt);
        // if (ioctl(m_socket_can, SIOCSCANBITTIMING, &ifr_bt) < 0) // 使接口"up"
        // {
        //     std::cerr << "Error setting CAN bitrate: " << strerror(errno) << std::endl;
        //     close(m_socket_can);
        //     return false;
        // }
        std::string command = "sudo ip link set can0 up type can bitrate 1000000"; // 构造命令字符串 
        int result = system(command.c_str()); // 使用 system 函数执行命令
        if (result == 0)  std::cout << "CAN interface configured successfully." << std::endl; 
        else              std::cerr << "Failed to configure CAN interface." << std::endl; 

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(m_socket_can, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
        {
            std::cerr << "Error in socket bind." << std::endl;
            close(m_socket_can);
            return false;
        }
        m_hand_isConnected = true;
        m_handSend_timer   = m_hand_can_nh.createTimer(ros::Duration(0.02), &Can_Communication::Hand_CanSendData_TimerEvent, this);  
        m_handRecv_timer   = m_hand_can_nh.createTimer(ros::Duration(0.02), &Can_Communication::Hand_CanRecvData_TimerEvent, this); 

        m_hand_can_nh   = ros_nh;     
        sub_right_hand  = m_hand_can_nh.subscribe("/right_hand_control", 10, &Can_Communication::RightHandCtrl_subCallback, this);
        sub_left_hand   = m_hand_can_nh.subscribe("/left_hand_control", 10, &Can_Communication::LeftHandCtrl_subCallback, this);
        pub_right_hand  = m_hand_can_nh.advertise<sensor_msgs::JointState>("/right_hand_states", 10);
        pub_left_hand   = m_hand_can_nh.advertise<sensor_msgs::JointState>("/left_hand_states", 10);
        return true; 
    }

    ///////////////////////////////////////////////////////////////////////////
    void Can_Communication::RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
        if(msg->position.size() == 16)
        {
            std::cout << "RightHandCtrl_subCallback " << std::endl;
            RightHand_Send.joint_angle_1 = msg->position[0];
            RightHand_Send.joint_angle_2 = msg->position[1];
            RightHand_Send.joint_angle_3 = msg->position[2];
            RightHand_Send.joint_angle_4 = msg->position[3];
            RightHand_Send.joint_angle_5 = msg->position[4];
            RightHand_Send.joint_angle_6 = msg->position[5];
            RightHand_Send.joint_angle_7 = msg->position[6];
            RightHand_Send.joint_angle_8 = msg->position[7];
            RightHand_Send.joint_angle_9 = msg->position[8];

            RightHand_Send.joint1_rotation = msg->position[9];
            RightHand_Send.joint3_raw = msg->position[10];
            RightHand_Send.joint1_tip = msg->position[11];
            RightHand_Send.joint2_tip = msg->position[12];
            RightHand_Send.joint3_tip = msg->position[13];
            RightHand_Send.joint4_tip = msg->position[14];
            RightHand_Send.joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("RightHandCtrl_subCallback: joint state size is not 16!");
        }
    }
    void Can_Communication::LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    { 
        if(msg->position.size() == 16)
        {
            std::cout << "LeftHandCtrl_subCallback " << std::endl;
            LeftHand_Send.joint_angle_1 = msg->position[0];
            LeftHand_Send.joint_angle_2 = msg->position[1];
            LeftHand_Send.joint_angle_3 = msg->position[2];
            LeftHand_Send.joint_angle_4 = msg->position[3];
            LeftHand_Send.joint_angle_5 = msg->position[4];
            LeftHand_Send.joint_angle_6 = msg->position[5];
            LeftHand_Send.joint_angle_7 = msg->position[6];
            LeftHand_Send.joint_angle_8 = msg->position[7];
            LeftHand_Send.joint_angle_9 = msg->position[8];

            LeftHand_Send.joint1_rotation = msg->position[9];
            LeftHand_Send.joint3_raw = msg->position[10];
            LeftHand_Send.joint1_tip = msg->position[11];
            LeftHand_Send.joint2_tip = msg->position[12];
            LeftHand_Send.joint3_tip = msg->position[13];
            LeftHand_Send.joint4_tip = msg->position[14];
            LeftHand_Send.joint5_tip = msg->position[15];
        }
        else
        {
            ROS_WARN("LeftHandCtrl_subCallback: joint state size is not 16!");
        } 
    }
    
    void Can_Communication::RightHandCtrl_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "right_hand";
        joint_state.name.resize(16);
        joint_state.position.resize(16); 
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = RightHand_Recv.joint_angle_1;  
        joint_state.position[1]  = RightHand_Recv.joint_angle_2;
        joint_state.position[2]  = RightHand_Recv.joint_angle_3;
        joint_state.position[3]  = RightHand_Recv.joint_angle_4;
        joint_state.position[4]  = RightHand_Recv.joint_angle_5;
        joint_state.position[5]  = RightHand_Recv.joint_angle_6;
        joint_state.position[6]  = RightHand_Recv.joint_angle_7;
        joint_state.position[7]  = RightHand_Recv.joint_angle_8;
        joint_state.position[8]  = RightHand_Recv.joint_angle_9;
        joint_state.position[9]  = RightHand_Recv.joint1_rotation;
        joint_state.position[10] = RightHand_Recv.joint3_raw;
        joint_state.position[11] = RightHand_Recv.joint1_tip;
        joint_state.position[12] = RightHand_Recv.joint2_tip;
        joint_state.position[13] = RightHand_Recv.joint3_tip;
        joint_state.position[14] = RightHand_Recv.joint4_tip;
        joint_state.position[15] = RightHand_Recv.joint5_tip; 

        joint_state.effort[0] = RightHand_Recv.pressure_1;
        joint_state.effort[1] = RightHand_Recv.pressure_2;
        joint_state.effort[2] = RightHand_Recv.pressure_3;
        joint_state.effort[3] = RightHand_Recv.pressure_4;
        joint_state.effort[4] = RightHand_Recv.pressure_5;      
                    
        pub_right_hand.publish(joint_state);
    }
    void Can_Communication::LeftHandCtrl_pubFunction(void)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "left_hand"; 
        joint_state.name.resize(16);
        joint_state.position.resize(16); 
        joint_state.effort.resize(5);

        joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8", "joint9",
                            "joint10", "joint11", "joint12", "joint13", "joint14", "joint15", "joint16"};
        
        joint_state.position[0]  = LeftHand_Recv.joint_angle_1;  
        joint_state.position[1]  = LeftHand_Recv.joint_angle_2;
        joint_state.position[2]  = LeftHand_Recv.joint_angle_3;
        joint_state.position[3]  = LeftHand_Recv.joint_angle_4;
        joint_state.position[4]  = LeftHand_Recv.joint_angle_5;
        joint_state.position[5]  = LeftHand_Recv.joint_angle_6;
        joint_state.position[6]  = LeftHand_Recv.joint_angle_7;
        joint_state.position[7]  = LeftHand_Recv.joint_angle_8;
        joint_state.position[8]  = LeftHand_Recv.joint_angle_9;
        joint_state.position[9]  = LeftHand_Recv.joint1_rotation;
        joint_state.position[10] = LeftHand_Recv.joint3_raw;
        joint_state.position[11] = LeftHand_Recv.joint1_tip;
        joint_state.position[12] = LeftHand_Recv.joint2_tip;
        joint_state.position[13] = LeftHand_Recv.joint3_tip;
        joint_state.position[14] = LeftHand_Recv.joint4_tip;
        joint_state.position[15] = LeftHand_Recv.joint5_tip; 

        joint_state.effort[0] = LeftHand_Recv.pressure_1;
        joint_state.effort[1] = LeftHand_Recv.pressure_2;
        joint_state.effort[2] = LeftHand_Recv.pressure_3;
        joint_state.effort[3] = LeftHand_Recv.pressure_4;
        joint_state.effort[4] = LeftHand_Recv.pressure_5;        
                    
        pub_left_hand.publish(joint_state);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    void Can_Communication::Hand_CanSendData_TimerEvent(const ros::TimerEvent& event)
    {
        switch (Hand_Freedom)
        {
        case 6:
        case 9:
            CanSend_PressureData(RightHandID, RightHand_Send);  
            CanSend_PressureData(LeftHandID, LeftHand_Send);  
            CanSend_6_9DofHand_DataDeal(RightHandID, RightHand_Send);
            CanSend_6_9DofHand_DataDeal(LeftHandID, LeftHand_Send);
            break;
        case 10:
            CanSend_PressureData(RightHandID, RightHand_Send);
            CanSend_PressureData(LeftHandID, LeftHand_Send); 
            CanSend_10DofHand_DataDeal(RightHandID, RightHand_Send);
            CanSend_10DofHand_DataDeal(LeftHandID, LeftHand_Send);
            break;
        case 16: 
            AllSendDataDeal_16DofHand(); 
            break;
        
        default: break;
        }
    } 
    void Can_Communication::Hand_CanRecvData_TimerEvent(const ros::TimerEvent& event)
    {
        Hand_CanRecvData();
    }
    void Can_Communication::Hand_CanRecvData(void)
    {
        int HandID = CanRecvDataDeal();
        switch (Hand_Freedom)
        {
            case 6:
            case 9:  
                CanRecv_6_9DofHand(HandID, can_recv_frame.data); 
                break;
            case 10:  
                CanRecv_10DofHand(HandID, can_recv_frame.data); 
                break;
            case 16:  
                CanRecv_16DofHand(HandID, can_recv_frame.data); 
                break; 
            default: break;  
        }
    }
    
    void Can_Communication::CanSendDataDeal(const uint8_t ID, const uint8_t data[8], const uint8_t len)
    {
        can_send_frame.can_id = ID;
        can_send_frame.can_dlc = len;
        for (int i = 0; i < len; i++)
        {
            can_send_frame.data[i] = data[i];
        }
        // 发送 CAN 帧
        int write_bytes = write(m_socket_can, &can_send_frame, sizeof(struct can_frame));
        // std::cout << "Write " << write_bytes << " bytes to CAN bus." << std::endl;
        if (write(m_socket_can, &can_send_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) 
        {
            std::cerr << "Error while sending CAN frame." << std::endl;
            return;
        } 
        usleep(1*1000); // 1ms  
    }
    int Can_Communication::CanRecvDataDeal(void)
    { 
        int read_bytes = read(m_socket_can, &can_recv_frame, sizeof(struct can_frame));
        if (read_bytes < 0) 
        {
            std::cerr << "Error in reading CAN frame." << std::endl;
            return 0;    
        }
        // std::cout << "Received CAN frame with ID: 0x" << std::hex << can_recv_frame.can_id << std::endl;
        // std::cout << "Data: ";
        // for (int i = 0; i < can_recv_frame.can_dlc; i++)
        // {
        //     std::cout << std::hex << (int)can_recv_frame.data[i] << " "; 
        // }
        // std::cout << std::dec << std::endl; 

        return can_recv_frame.can_id;
    }
     
    void Can_Communication::CanSend_PressureData(const uint32_t ID, CanHand_SendData sendData)
    {
        uint8_t send_data[8];
        send_data[0] = 0x02;
        send_data[1] = sendData.pressure_1;
        send_data[2] = sendData.pressure_2;
        send_data[3] = sendData.pressure_3;
        send_data[4] = sendData.pressure_4;
        send_data[5] = sendData.pressure_5;
        CanSendDataDeal(ID, send_data, 6); 
    } 
    void Can_Communication::CanSend_10DofHand_DataDeal(const uint32_t ID, CanHand_SendData sendData)
    {
        uint8_t send_data[8];
        send_data[0] = 0x01;
        send_data[1] = sendData.joint_angle_1; 
        send_data[2] = sendData.joint_angle_2;
        send_data[3] = sendData.joint_angle_3;
        send_data[4] = sendData.joint_angle_4;
        send_data[5] = sendData.joint_angle_5;
        send_data[6] = sendData.joint_angle_6;
        CanSendDataDeal(ID, send_data, 7);  	

        send_data[0] = 0x04;
        send_data[1] = sendData.joint_angle_7;
        send_data[2] = sendData.joint_angle_8;
        send_data[3] = sendData.joint_angle_9;
        send_data[4] = sendData.joint1_rotation;
        CanSendDataDeal(ID, send_data, 5);   

        send_data[0] = 0x05;
        send_data[1] = sendData.speed_1;
        send_data[2] = sendData.speed_2;
        send_data[3] = sendData.speed_3;
        send_data[4] = sendData.speed_4;
        send_data[5] = sendData.speed_5;
        CanSendDataDeal(ID, send_data, 6); 
    }  
    void Can_Communication::CanSend_6_9DofHand_DataDeal(const uint32_t ID, CanHand_SendData sendData)
    {
        uint8_t send_data[8];
        send_data[0] = 0x01;
        send_data[1] = sendData.joint_angle_1;
        send_data[2] = sendData.joint_angle_2;
        send_data[3] = sendData.joint_angle_3;
        send_data[4] = sendData.joint_angle_4;
        send_data[5] = sendData.joint_angle_5;
        send_data[6] = sendData.joint_angle_6; 
        CanSendDataDeal(ID, send_data, 7);  

        send_data[0] = 0x04;
        send_data[1] = sendData.joint_angle_7;
        send_data[2] = sendData.joint_angle_8;
        send_data[3] = sendData.joint_angle_9;
        CanSendDataDeal(ID, send_data, 4); 
    }

    void Can_Communication::CanRecv_6_9DofHand(const uint32_t ID, const uint8_t data[8])
    {
        switch(data[0])
        {
            case JOINT_POSIION_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.joint_angle_1 = data[1];
                    RightHand_Recv.joint_angle_2 = data[2];
                    RightHand_Recv.joint_angle_3 = data[3];
                    RightHand_Recv.joint_angle_4 = data[4];
                    RightHand_Recv.joint_angle_5 = data[5];
                    RightHand_Recv.joint_angle_6 = data[6]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.joint_angle_1 = data[1];
                    LeftHand_Recv.joint_angle_2 = data[2];
                    LeftHand_Recv.joint_angle_3 = data[3];
                    LeftHand_Recv.joint_angle_4 = data[4];
                    LeftHand_Recv.joint_angle_5 = data[5];
                    LeftHand_Recv.joint_angle_6 = data[6]; 
                } 
                // std::cout << "JOINT_POSIION_RCO ID = " << static_cast<int>(ID) 
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.joint_angle_1)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_2)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_3)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_4)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_5)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_6) << std::endl; 
                break;
            }
            case MAX_PRESS_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.pressure_1 = data[1];
                    RightHand_Recv.pressure_2 = data[2];
                    RightHand_Recv.pressure_3 = data[3];
                    RightHand_Recv.pressure_4 = data[4];
                    RightHand_Recv.pressure_5 = data[5];
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.pressure_1 = data[1];
                    LeftHand_Recv.pressure_2 = data[2];
                    LeftHand_Recv.pressure_3 = data[3];
                    LeftHand_Recv.pressure_4 = data[4];
                    LeftHand_Recv.pressure_5 = data[5];
                } 
                // std::cout << "MAX_PRESS_RCO ID = " << static_cast<int>(ID) 
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.pressure_1)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_2)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_3)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_4)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_5) << std::endl;
                break;
            } 
            case JOINT_POSITION2_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.joint_angle_7   = data[1];
                    RightHand_Recv.joint_angle_8   = data[2];
                    RightHand_Recv.joint_angle_9   = data[3]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.joint_angle_7   = data[1];
                    LeftHand_Recv.joint_angle_8   = data[2];
                    LeftHand_Recv.joint_angle_9   = data[3]; 
                } 
                // std::cout << "JOINT_POSITION2_RCO ID = " << static_cast<int>(ID)
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.joint_angle_7)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_8)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_9) << std::endl;
                break;
            }  
            default: break; 
        }
    }
    void Can_Communication::CanRecv_10DofHand(const uint32_t ID, const uint8_t data[8])
    {
        switch(data[0])
        {
            case JOINT_POSIION_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.joint_angle_1 = data[1];
                    RightHand_Recv.joint_angle_2 = data[2];
                    RightHand_Recv.joint_angle_3 = data[3];
                    RightHand_Recv.joint_angle_4 = data[4];
                    RightHand_Recv.joint_angle_5 = data[5];
                    RightHand_Recv.joint_angle_6 = data[6]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.joint_angle_1 = data[1];
                    LeftHand_Recv.joint_angle_2 = data[2];
                    LeftHand_Recv.joint_angle_3 = data[3];
                    LeftHand_Recv.joint_angle_4 = data[4];
                    LeftHand_Recv.joint_angle_5 = data[5];
                    LeftHand_Recv.joint_angle_6 = data[6]; 
                } 
                // std::cout << "JOINT_POSIION_RCO ID = " << static_cast<int>(ID) 
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.joint_angle_1)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_2)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_3)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_4)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_5)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_6) << std::endl; 
                break;
            }
            case MAX_PRESS_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.pressure_1 = data[1];
                    RightHand_Recv.pressure_2 = data[2];
                    RightHand_Recv.pressure_3 = data[3];
                    RightHand_Recv.pressure_4 = data[4];
                    RightHand_Recv.pressure_5 = data[5];
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.pressure_1 = data[1];
                    LeftHand_Recv.pressure_2 = data[2];
                    LeftHand_Recv.pressure_3 = data[3];
                    LeftHand_Recv.pressure_4 = data[4];
                    LeftHand_Recv.pressure_5 = data[5];
                } 
                // std::cout << "MAX_PRESS_RCO ID = " << static_cast<int>(ID) 
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.pressure_1)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_2)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_3)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_4)
                //           << " " << static_cast<int>(RightHand_Recv.pressure_5) << std::endl;
                break;
            } 
            case JOINT_POSITION2_RCO:
            {
                if (ID == RightHandID)
                {
                    RightHand_Recv.joint_angle_7   = data[1];
                    RightHand_Recv.joint_angle_8   = data[2];
                    RightHand_Recv.joint_angle_9   = data[3]; 
                    RightHand_Recv.joint1_rotation = data[4];
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_Recv.joint_angle_7   = data[1];
                    LeftHand_Recv.joint_angle_8   = data[2];
                    LeftHand_Recv.joint_angle_9   = data[3]; 
                    LeftHand_Recv.joint1_rotation = data[4];
                } 
                // std::cout << "JOINT_POSITION2_RCO ID = " << static_cast<int>(ID)
                //           << " RightHand_Recv = " << static_cast<int>(RightHand_Recv.joint_angle_7)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_8)
                //           << " " << static_cast<int>(RightHand_Recv.joint_angle_9) 
                //           << " " << static_cast<int>(RightHand_Recv.joint1_rotation) << std::endl;
                break;
            }  
            default: break; 
        } 
    }
    
    void Can_Communication::CanSend_16DofHand(const uint32_t ID)
    {
        uint8_t send_data[8]; 

        send_data[0] = JOINT_PITCH_R;
        send_data[1] = Hand_16Dof_Send.thumb.pitch_angle;
        send_data[2] = Hand_16Dof_Send.index.pitch_angle;
        send_data[3] = Hand_16Dof_Send.middle.pitch_angle;
        send_data[4] = Hand_16Dof_Send.ring.pitch_angle;
        send_data[5] = Hand_16Dof_Send.little.pitch_angle;
        CanSendDataDeal(ID, send_data, 6);   

        send_data[0] = JOINT_YAW_R;
        send_data[1] = Hand_16Dof_Send.thumb.yaw_angle;
        send_data[2] = Hand_16Dof_Send.index.yaw_angle;
        send_data[3] = Hand_16Dof_Send.middle.yaw_angle;
        send_data[4] = Hand_16Dof_Send.ring.yaw_angle;
        send_data[5] = Hand_16Dof_Send.little.yaw_angle;
        CanSendDataDeal(ID, send_data, 6);   

        send_data[0] = JOINT_ROLL_R;
        send_data[1] = Hand_16Dof_Send.thumb.roll_angle;
        send_data[2] = Hand_16Dof_Send.index.roll_angle;
        send_data[3] = Hand_16Dof_Send.middle.roll_angle;
        send_data[4] = Hand_16Dof_Send.ring.roll_angle;
        send_data[5] = Hand_16Dof_Send.little.roll_angle;
        CanSendDataDeal(ID, send_data, 6);   

        send_data[0] = JOINT_TIP_R;
        send_data[1] = Hand_16Dof_Send.thumb.tip_angle;
        send_data[2] = Hand_16Dof_Send.index.tip_angle;
        send_data[3] = Hand_16Dof_Send.middle.tip_angle;
        send_data[4] = Hand_16Dof_Send.ring.tip_angle;
        send_data[5] = Hand_16Dof_Send.little.tip_angle;
        CanSendDataDeal(ID, send_data, 6);   

        send_data[0] = JOINT_SPEED_R;
        send_data[1] = Hand_16Dof_Send.thumb.speed;
        send_data[2] = Hand_16Dof_Send.index.speed;
        send_data[3] = Hand_16Dof_Send.middle.speed;
        send_data[4] = Hand_16Dof_Send.ring.speed;
        send_data[5] = Hand_16Dof_Send.little.speed;
        CanSendDataDeal(ID, send_data, 6);  

        send_data[0] = JOINT_CURRENT_R;
        send_data[1] = Hand_16Dof_Send.thumb.over_current;
        send_data[2] = Hand_16Dof_Send.index.over_current;
        send_data[3] = Hand_16Dof_Send.middle.over_current;
        send_data[4] = Hand_16Dof_Send.ring.over_current;
        send_data[5] = Hand_16Dof_Send.little.over_current;
        CanSendDataDeal(ID, send_data, 6);  
    }
    void Can_Communication::CanRecv_16DofHand(const uint32_t ID, const uint8_t data[8])
    {   
        std::cout << "CanRecv_16DofHand" << std::endl;
        switch(data[0])
        {
            case JOINT_PITCH_R: // 上位侧写入一次pitch，返回一次pitch 
            {
                if(ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.pitch_angle  = data[1];
                    RightHand_16Dof_Recv.index.pitch_angle  = data[2];
                    RightHand_16Dof_Recv.middle.pitch_angle = data[3];
                    RightHand_16Dof_Recv.ring.pitch_angle   = data[4];
                    RightHand_16Dof_Recv.little.pitch_angle = data[5];                     
                }
                else if(ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.pitch_angle  = data[1];
                    LeftHand_16Dof_Recv.index.pitch_angle  = data[2];
                    LeftHand_16Dof_Recv.middle.pitch_angle = data[3];
                    LeftHand_16Dof_Recv.ring.pitch_angle   = data[4];
                    LeftHand_16Dof_Recv.little.pitch_angle = data[5];                     
                }
                break;
            } 
            case JOINT_YAW_R: // 上位侧写入一次yaw，返回一次yaw 
            {
                if (ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.yaw_angle  = data[1];
                    RightHand_16Dof_Recv.index.yaw_angle  = data[2];
                    RightHand_16Dof_Recv.middle.yaw_angle = data[3];
                    RightHand_16Dof_Recv.ring.yaw_angle   = data[4];
                    RightHand_16Dof_Recv.little.yaw_angle = data[5]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.yaw_angle  = data[1];
                    LeftHand_16Dof_Recv.index.yaw_angle  = data[2];
                    LeftHand_16Dof_Recv.middle.yaw_angle = data[3];
                    LeftHand_16Dof_Recv.ring.yaw_angle   = data[4];
                    LeftHand_16Dof_Recv.little.yaw_angle = data[5]; 
                } 
                break;
            } 
            case JOINT_ROLL_R: // 上位侧写入一次roll，返回一次roll 
            {
                if (ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.roll_angle  = data[1];
                    RightHand_16Dof_Recv.index.roll_angle  = data[2];
                    RightHand_16Dof_Recv.middle.roll_angle = data[3];
                    RightHand_16Dof_Recv.ring.roll_angle   = data[4];
                    RightHand_16Dof_Recv.little.roll_angle = data[5]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.roll_angle  = data[1];
                    LeftHand_16Dof_Recv.index.roll_angle  = data[2];
                    LeftHand_16Dof_Recv.middle.roll_angle = data[3];
                    LeftHand_16Dof_Recv.ring.roll_angle   = data[4];
                    LeftHand_16Dof_Recv.little.roll_angle = data[5]; 
                }  
                break;
            }  
            case JOINT_TIP_R:  // 上位侧写入一次tip，返回一次tip 
            { 
                if (ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.tip_angle  = data[1];
                    RightHand_16Dof_Recv.index.tip_angle  = data[2];
                    RightHand_16Dof_Recv.middle.tip_angle = data[3];
                    RightHand_16Dof_Recv.ring.tip_angle   = data[4];
                    RightHand_16Dof_Recv.little.tip_angle = data[5]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.tip_angle  = data[1];
                    LeftHand_16Dof_Recv.index.tip_angle  = data[2];
                    LeftHand_16Dof_Recv.middle.tip_angle = data[3];
                    LeftHand_16Dof_Recv.ring.tip_angle   = data[4];
                    LeftHand_16Dof_Recv.little.tip_angle = data[5];     
                }
                
                break;
            }  
            case JOINT_SPEED_R:  // 上位侧写入一次speed，返回一次speed 
            {
                if (ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.speed  = data[1];
                    RightHand_16Dof_Recv.index.speed  = data[2];
                    RightHand_16Dof_Recv.middle.speed = data[3];
                    RightHand_16Dof_Recv.ring.speed   = data[4];
                    RightHand_16Dof_Recv.little.speed = data[5]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.speed  = data[1];
                    LeftHand_16Dof_Recv.index.speed  = data[2];
                    LeftHand_16Dof_Recv.middle.speed = data[3];
                    LeftHand_16Dof_Recv.ring.speed   = data[4];
                    LeftHand_16Dof_Recv.little.speed = data[5]; 
                }
                break;
            }    
            case JOINT_CURRENT_R:  // 上位侧写入一次speed，返回一次speed 
            {
                if (ID == RightHandID)
                {
                    RightHand_16Dof_Recv.thumb.over_current  = data[1];
                    RightHand_16Dof_Recv.index.over_current  = data[2];
                    RightHand_16Dof_Recv.middle.over_current = data[3];
                    RightHand_16Dof_Recv.ring.over_current   = data[4];
                    RightHand_16Dof_Recv.little.over_current = data[5]; 
                }
                else if (ID == LeftHandID)
                {
                    LeftHand_16Dof_Recv.thumb.over_current  = data[1];
                    LeftHand_16Dof_Recv.index.over_current  = data[2];
                    LeftHand_16Dof_Recv.middle.over_current = data[3];
                    LeftHand_16Dof_Recv.ring.over_current   = data[4];
                    LeftHand_16Dof_Recv.little.over_current = data[5]; 
                } 
                break;
            }  
            default:
                break;
        }
        AllRecvDataDeal_16DofHand();
    }
    void Can_Communication::AllSendDataDeal_16DofHand(void)
    {
        Hand_16Dof_Send.thumb.pitch_angle   = RightHand_Send.joint_angle_1;
        Hand_16Dof_Send.index.pitch_angle   = RightHand_Send.joint_angle_3;
        Hand_16Dof_Send.middle.pitch_angle  = RightHand_Send.joint_angle_4;
        Hand_16Dof_Send.ring.pitch_angle 	= RightHand_Send.joint_angle_5;
        Hand_16Dof_Send.little.pitch_angle  = RightHand_Send.joint_angle_6;

        Hand_16Dof_Send.thumb.yaw_angle     = RightHand_Send.joint_angle_2;
        Hand_16Dof_Send.index.yaw_angle     = RightHand_Send.joint_angle_7;
        Hand_16Dof_Send.middle.yaw_angle 	= RightHand_Send.joint3_raw;
        Hand_16Dof_Send.ring.yaw_angle 	    = RightHand_Send.joint_angle_8;
        Hand_16Dof_Send.little.yaw_angle 	= RightHand_Send.joint_angle_9;

        Hand_16Dof_Send.thumb.roll_angle 	= RightHand_Send.joint1_rotation;
        Hand_16Dof_Send.index.roll_angle 	= RightHand_Send.joint1_rotation;
        Hand_16Dof_Send.middle.roll_angle   = RightHand_Send.joint1_rotation;
        Hand_16Dof_Send.ring.roll_angle 	= RightHand_Send.joint1_rotation;
        Hand_16Dof_Send.little.roll_angle   = RightHand_Send.joint1_rotation;
 
        Hand_16Dof_Send.thumb.tip_angle 	= RightHand_Send.joint1_tip;
        Hand_16Dof_Send.index.tip_angle 	= RightHand_Send.joint2_tip;
        Hand_16Dof_Send.middle.tip_angle    = RightHand_Send.joint3_tip;
        Hand_16Dof_Send.ring.tip_angle 	    = RightHand_Send.joint4_tip;
        Hand_16Dof_Send.little.tip_angle    = RightHand_Send.joint5_tip;

        Hand_16Dof_Send.thumb.speed         = RightHand_Send.speed_1;
        Hand_16Dof_Send.index.speed         = RightHand_Send.speed_2;
        Hand_16Dof_Send.middle.speed        = RightHand_Send.speed_3;
        Hand_16Dof_Send.ring.speed 	        = RightHand_Send.speed_4;
        Hand_16Dof_Send.little.speed        = RightHand_Send.speed_5;

        Hand_16Dof_Send.thumb.over_current  = RightHand_Send.pressure_1;
        Hand_16Dof_Send.index.over_current  = RightHand_Send.pressure_2;
        Hand_16Dof_Send.middle.over_current = RightHand_Send.pressure_3;
        Hand_16Dof_Send.ring.over_current   = RightHand_Send.pressure_4;
        Hand_16Dof_Send.little.over_current = RightHand_Send.pressure_5;

        CanSend_16DofHand(RightHandID);

        Hand_16Dof_Send.thumb.pitch_angle 	= LeftHand_Send.joint_angle_1;
        Hand_16Dof_Send.index.pitch_angle 	= LeftHand_Send.joint_angle_3;
        Hand_16Dof_Send.middle.pitch_angle 	= LeftHand_Send.joint_angle_4;
        Hand_16Dof_Send.ring.pitch_angle 	= LeftHand_Send.joint_angle_5;
        Hand_16Dof_Send.little.pitch_angle 	= LeftHand_Send.joint_angle_6;

        Hand_16Dof_Send.thumb.yaw_angle 	= LeftHand_Send.joint_angle_2;
        Hand_16Dof_Send.index.yaw_angle 	= LeftHand_Send.joint_angle_7;
        Hand_16Dof_Send.middle.yaw_angle 	= LeftHand_Send.joint3_raw;
        Hand_16Dof_Send.ring.yaw_angle 		= LeftHand_Send.joint_angle_8;
        Hand_16Dof_Send.little.yaw_angle 	= LeftHand_Send.joint_angle_9;

        Hand_16Dof_Send.thumb.roll_angle 	= LeftHand_Send.joint1_rotation;
        Hand_16Dof_Send.index.roll_angle 	= LeftHand_Send.joint1_rotation;
        Hand_16Dof_Send.middle.roll_angle 	= LeftHand_Send.joint1_rotation;
        Hand_16Dof_Send.ring.roll_angle 	= LeftHand_Send.joint1_rotation;
        Hand_16Dof_Send.little.roll_angle 	= LeftHand_Send.joint1_rotation;

        Hand_16Dof_Send.thumb.tip_angle 	= LeftHand_Send.joint1_tip;
        Hand_16Dof_Send.index.tip_angle 	= LeftHand_Send.joint2_tip;
        Hand_16Dof_Send.middle.tip_angle 	= LeftHand_Send.joint3_tip;
        Hand_16Dof_Send.ring.tip_angle 		= LeftHand_Send.joint4_tip;
        Hand_16Dof_Send.little.tip_angle 	= LeftHand_Send.joint5_tip;

        Hand_16Dof_Send.thumb.speed      	= LeftHand_Send.speed_1;
        Hand_16Dof_Send.index.speed 	    = LeftHand_Send.speed_2;
        Hand_16Dof_Send.middle.speed 	    = LeftHand_Send.speed_3;
        Hand_16Dof_Send.ring.speed       	= LeftHand_Send.speed_4;
        Hand_16Dof_Send.little.speed     	= LeftHand_Send.speed_5;

        Hand_16Dof_Send.thumb.over_current 	= LeftHand_Send.pressure_1;
        Hand_16Dof_Send.index.over_current 	= LeftHand_Send.pressure_2;
        Hand_16Dof_Send.middle.over_current = LeftHand_Send.pressure_3;
        Hand_16Dof_Send.ring.over_current 	= LeftHand_Send.pressure_4;
        Hand_16Dof_Send.little.over_current = LeftHand_Send.pressure_5;

        CanSend_16DofHand(LeftHandID);
    }
    void Can_Communication::AllRecvDataDeal_16DofHand(void)
    {
        // 右手的处理
        RightHand_Recv.joint_angle_1 = RightHand_16Dof_Recv.thumb.pitch_angle;
        RightHand_Recv.joint_angle_3 = RightHand_16Dof_Recv.index.pitch_angle;
        RightHand_Recv.joint_angle_4 = RightHand_16Dof_Recv.middle.pitch_angle;
        RightHand_Recv.joint_angle_5 = RightHand_16Dof_Recv.ring.pitch_angle;
        RightHand_Recv.joint_angle_6 = RightHand_16Dof_Recv.little.pitch_angle;
        
        RightHand_Recv.joint_angle_2 = RightHand_16Dof_Recv.thumb.yaw_angle;
        RightHand_Recv.joint_angle_7 = RightHand_16Dof_Recv.index.yaw_angle;
        RightHand_Recv.joint3_raw    = RightHand_16Dof_Recv.middle.yaw_angle;
        RightHand_Recv.joint_angle_8 = RightHand_16Dof_Recv.ring.yaw_angle;
        RightHand_Recv.joint_angle_9 = RightHand_16Dof_Recv.little.yaw_angle;
        
        RightHand_Recv.joint1_rotation = RightHand_16Dof_Recv.thumb.roll_angle; 
        
        RightHand_Recv.joint1_tip = RightHand_16Dof_Recv.thumb.tip_angle;
        RightHand_Recv.joint2_tip = RightHand_16Dof_Recv.index.tip_angle;
        RightHand_Recv.joint3_tip = RightHand_16Dof_Recv.middle.tip_angle;
        RightHand_Recv.joint4_tip = RightHand_16Dof_Recv.ring.tip_angle;
        RightHand_Recv.joint5_tip = RightHand_16Dof_Recv.little.tip_angle; 

        RightHand_Recv.pressure_1 = RightHand_16Dof_Recv.thumb.over_current;
        RightHand_Recv.pressure_2 = RightHand_16Dof_Recv.index.over_current;
        RightHand_Recv.pressure_3 = RightHand_16Dof_Recv.middle.over_current;
        RightHand_Recv.pressure_4 = RightHand_16Dof_Recv.ring.over_current;
        RightHand_Recv.pressure_5 = RightHand_16Dof_Recv.little.over_current;
        
        // 左手的处理
        LeftHand_Recv.joint_angle_1 = LeftHand_16Dof_Recv.thumb.pitch_angle;
        LeftHand_Recv.joint_angle_3 = LeftHand_16Dof_Recv.index.pitch_angle;
        LeftHand_Recv.joint_angle_4 = LeftHand_16Dof_Recv.middle.pitch_angle;
        LeftHand_Recv.joint_angle_5 = LeftHand_16Dof_Recv.ring.pitch_angle;
        LeftHand_Recv.joint_angle_6 = LeftHand_16Dof_Recv.little.pitch_angle;
        
        LeftHand_Recv.joint_angle_2 = LeftHand_16Dof_Recv.thumb.yaw_angle;
        LeftHand_Recv.joint_angle_7 = LeftHand_16Dof_Recv.index.yaw_angle;
        LeftHand_Recv.joint3_raw    = LeftHand_16Dof_Recv.middle.yaw_angle;
        LeftHand_Recv.joint_angle_8 = LeftHand_16Dof_Recv.ring.yaw_angle;
        LeftHand_Recv.joint_angle_9 = LeftHand_16Dof_Recv.little.yaw_angle;
        
        LeftHand_Recv.joint1_rotation = LeftHand_16Dof_Recv.thumb.roll_angle; 
        
        LeftHand_Recv.joint1_tip = LeftHand_16Dof_Recv.thumb.tip_angle;
        LeftHand_Recv.joint2_tip = LeftHand_16Dof_Recv.index.tip_angle;
        LeftHand_Recv.joint3_tip = LeftHand_16Dof_Recv.middle.tip_angle;
        LeftHand_Recv.joint4_tip = LeftHand_16Dof_Recv.ring.tip_angle;
        LeftHand_Recv.joint5_tip = LeftHand_16Dof_Recv.little.tip_angle; 

        LeftHand_Recv.pressure_1 = LeftHand_16Dof_Recv.thumb.over_current;
        LeftHand_Recv.pressure_2 = LeftHand_16Dof_Recv.index.over_current;
        LeftHand_Recv.pressure_3 = LeftHand_16Dof_Recv.middle.over_current;
        LeftHand_Recv.pressure_4 = LeftHand_16Dof_Recv.ring.over_current;
        LeftHand_Recv.pressure_5 = LeftHand_16Dof_Recv.little.over_current;
    }
    
}  // namespace AIMOcommunicate
