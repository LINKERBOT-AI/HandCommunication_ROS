/*
  通讯处理类：Json处理数据 机械手
*/

#ifndef HAND_TCP_COMMUNICATION_H
#define HAND_TCP_COMMUNICATION_H

#include "GeneralFunction.h"

struct Hand_SendStruct
{
    uint8_t PowerOn;
    uint8_t Work_Mode;
    uint8_t Control_Mode;
    uint8_t Move_Control;

    uint8_t Hand_Freedom; // 机械手自由度
    uint8_t Retain1;
    uint8_t Retain2;
    uint8_t Retain3;

    HumanHandParam  Set_Point_RightHand;
    HumanHandParam  Set_Speed_RightHand; 
    HumanHandParam  Set_Point_LeftHand;
    HumanHandParam  Set_Speed_LeftHand;
    HandForceParam  Set_Force_RightHand;
    HandForceParam  Set_Force_LeftHand;  
};
struct Hand_RecvStruct
{
    uint8_t       PowerOn_State; 
    uint8_t       Work_State;
    uint8_t       Control_State;
    uint8_t       Retain;
    
    uint8_t       RightHand_MoveState;
    uint8_t       LeftHand_MoveState;
    uint16_t      Error_State;  

    HumanHandParam  Get_CurrentPoint_RightHand;
    HumanHandParam  Get_CurrentPoint_LeftHand;
    
    HandForceParam  Get_CurrentForce_RightHand;
    HandForceParam  Get_CurrentForce_LeftHand; 

    HumanHandParam  Get_CurrentPoint_RightGlove;
    HumanHandParam  Get_CurrentPoint_LeftGlove;
};

struct TCP_Hand_SendData
{
    uint8_t PowerOn;
    uint8_t Work_Mode;
    uint8_t Control_Mode;
    uint8_t Move_Control; 

    uint8_t Hand_Freedom; // 机械手自由度
    uint8_t Retain1;
    uint8_t Retain2;
    uint8_t Retain3; 

    HumanHandParam  Set_Point_RightHand;
    HumanHandParam  Set_Speed_RightHand; 
    HumanHandParam  Set_Point_LeftHand;
    HumanHandParam  Set_Speed_LeftHand; 
    HandForceParam  Set_Force_RightHand;
    HandForceParam  Set_Force_LeftHand; 
};
struct TCP_Hand_RecvData
{
    uint8_t       PowerOn_State;
    std::string   Work_State;
    std::string   Control_State;
 
    std::string   RightHand_MoveState;
    std::string   LeftHand_MoveState;
    uint16_t      Error_State;
 
    HumanHandParam  Get_CurrentPoint_RightHand;
    HumanHandParam  Get_CurrentPoint_LeftHand;
    
    HandForceParam  Get_CurrentForce_RightHand;
    HandForceParam  Get_CurrentForce_LeftHand; 

    HumanHandParam  Get_CurrentPoint_RightGlove;
    HumanHandParam  Get_CurrentPoint_LeftGlove;
}; 

namespace AIMOcommunicate
{
    class TCPCommunication_Hand
    {
    public:
        TCPCommunication_Hand();
        ~TCPCommunication_Hand();
        bool Hand_TCPCommunication_connect(const ros::NodeHandle& ros_TCP_nh);

        void RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg); 
        void LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg); 
        void RightHand_pubFunction(void); 
        void LeftHand_pubFunction(void);  
        void RightGlove_pubFunction(void); 
        void LeftGlove_pubFunction(void); 

    private:   
        void Hand_InitDataSend(void);
        void Hand_TCPCommunication_Init(void); 
        void Hand_TCPCommunication_stop(void);
        void Hand_TCPCommunication_Json_Launch_Deal();
        
    public:  
        TCP_Hand_RecvData HandClient_Recv;
        TCP_Hand_SendData HandClient_Send;

    private:  // 通讯连接变量
        struct sockaddr_in m_server_hand_addr;
        int                m_server_hand_port;
        int                m_server_hand_sockfd;

    private:
        general::GeneralFunction hand_TCPcommunicate_generalfun;

        ros::NodeHandle m_hand_tcp_nh;
        std::string     m_server_hand_ip;
        bool            handEnable;
        int             Hand_Freedom; 
        int             JointVelocity; 
        int             JointEffort; 

        ros::Subscriber sub_right_hand;
        ros::Subscriber sub_left_hand;
        ros::Publisher  pub_right_hand;
        ros::Publisher  pub_left_hand;
        ros::Publisher  pub_right_glove;
        ros::Publisher  pub_left_glove;

        ros::Timer      m_hand_timer;
        bool            m_hand_isConnected;

    private:
        void createTimer(int seconds)
        {
            signal(SIGALRM, [](int) {});
            alarm(seconds);
        }
        void Hand_TCPSendRecvMessage(const ros::TimerEvent& event);
        
        // 结构体通讯
        int  Hand_DataDeal_Struct(const int connect_sockfd);
        void Hand_SendStructDataDeal(TCP_Hand_SendData handData, Hand_SendStruct& sendData);
        void Hand_RecvStructDataDeal(Hand_RecvStruct recvData, TCP_Hand_RecvData& handData);
        Hand_SendStruct TCP_SendStruct_Data;
        Hand_RecvStruct TCP_RecvStruct_Data; 
        
        std::string Hand_RecvDataStr, Hand_SendDataStr;
    };
}  // namespace AIMOcommunicate
#endif