/*
  通讯处理类：can 总线通信 控制机械手
*/

#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "GeneralFunction.h"

typedef enum
{									 
	INVALID_FRAME_PROPERTY = 0x0,  //无效的can帧属性	 |无返回
	JOINT_POSIION_RCO      = 0x1,  //关节位置			|返回本类型数据
	MAX_PRESS_RCO          = 0x2,  //最大压力			|返回本类型数据
	OTHER_DATA_RCO         = 0x3,  //其它数据			|返回本类型数据
	JOINT_POSITION2_RCO    = 0X04, //关节位置2补充超过6轴的 

	REQUEST_DATA_RETURN    = 0x9,  //请求数据返回		|返回所有数据

	JOINT_POSIION_N        = 0x11, //关节位置			|不返回任何数据
	MAX_PRESS_N            = 0x12, //最大压力			|不返回任何数据
	OTHER_DATA_N           = 0x13, //其它数据			|不返回任何数据
}CAN_FRAME_PROPERTY;

typedef enum
{									  
	INVALID_FRAME_PROPERTY_ = 0x00,  //无效的can帧属性	   |无返回
	JOINT_PITCH_R   = 0x01,	//短帧俯仰角-手指根部弯曲		 |返回本类型数据
	JOINT_YAW_R     = 0x02,	//短帧航向角-手指横摆，控制间隙	  |返回本类型数据
	JOINT_ROLL_R    = 0x03,	//短帧横滚角-只有大拇指副用到了   |返回本类型数据
	JOINT_TIP_R	    = 0X04,	//短帧指尖角度控制				|返回本类型数据
	JOINT_SPEED_R   = 0X05,	//短帧速度 	电机运行速度控制	|返回本类型数据
	JOINT_CURRENT_R = 0X06,	//短帧电流 	电机运行电流反馈	|返回本类型数据
	JOINT_FAULT_R   = 0X07, //短帧故障 	电机运行故障反馈	|返回本类型数据
	
	REQUEST_DATA_RETURN_ = 0x9,	 //请求数据返回				    |返回所有数据
	JOINT_PITCH_NR	 = 0x11,	 //俯仰角-手指根部弯曲           |不返回本类型数据
	JOINT_YAW_NR	 = 0x12,     //航向角-手指横摆，控制间隙	  |不返回本类型数据
	JOINT_ROLL_NR	 = 0x13,	 //横滚角-只有大拇指副用到了	  |不返回本类型数据
	JOINT_TIP_NR	 = 0X14,	 //指尖角度控制			        |不返回本类型数据
	JOINT_SPEED_NR	 = 0X15,	 //速度 	电机运行速度控制	  |不返回本类型数据
	JOINT_CURRENT_NR = 0X16,     //电流 电机运行电流反馈	      |不返回本类型数据
	JOINT_FAULT_NR   = 0X17,	 //故障 	电机运行故障反馈	  |不返回本类型数据
}FRAME_PROPERTY;

struct CanHand_SendData 
{ 
	uint8_t joint_angle_1;	 //每个手指的关节位置设定
	uint8_t joint_angle_2;
	uint8_t joint_angle_3;
	uint8_t joint_angle_4;
	uint8_t joint_angle_5;
	uint8_t joint_angle_6;
	uint8_t joint_angle_7;
	uint8_t joint_angle_8;
	uint8_t joint_angle_9; 

	uint8_t joint1_rotation; // 拇指旋转 —— 10自由度多余部分

	uint8_t joint1_tip;  // 拇指 末端弯曲度 —— 16自由度多余部分
	uint8_t joint3_raw;  // 中指 旋转
	uint8_t joint2_tip;  // 食指 末端弯曲度
	uint8_t joint3_tip;  // 中指 末端弯曲度
	uint8_t joint4_tip;  // 无名 指末端弯曲度
	uint8_t joint5_tip;  // 小拇 指末端弯曲度
 
	uint8_t speed_1;	 //速度 设定
	uint8_t speed_2;
	uint8_t speed_3;
	uint8_t speed_4;
	uint8_t speed_5; 
 
	uint8_t pressure_1;	 //每个指尖的压力设定
	uint8_t pressure_2;
	uint8_t pressure_3;
	uint8_t pressure_4;
	uint8_t pressure_5; 
 
	bool press_calibration;  //压力传感器校准，暂时不启用
	uint8_t clear_fault;	 //对应位高电平有效，高电平清除故障,低六位对应六个电机的清除电流故障
}; 
struct CanHand_RecvData
{ 
	uint8_t joint_angle_1;	 //每个手指的关节位置设定
	uint8_t joint_angle_2;
	uint8_t joint_angle_3;
	uint8_t joint_angle_4;
	uint8_t joint_angle_5;
	uint8_t joint_angle_6;
	uint8_t joint_angle_7;
	uint8_t joint_angle_8;
	uint8_t joint_angle_9; 

	uint8_t joint1_rotation; // 拇指旋转 —— 10自由度多余部分

	uint8_t joint1_tip;  // 拇指 末端弯曲度 —— 16自由度多余部分
	uint8_t joint3_raw;  // 中指 旋转
	uint8_t joint2_tip;  // 食指 末端弯曲度
	uint8_t joint3_tip;  // 中指 末端弯曲度
	uint8_t joint4_tip;  // 无名 指末端弯曲度
	uint8_t joint5_tip;  // 小拇 指末端弯曲度
 
	uint8_t pressure_1;	 // 每个指尖的压力设定
	uint8_t pressure_2;
	uint8_t pressure_3;
	uint8_t pressure_4;
	uint8_t pressure_5; 

	uint8_t is_force_calibration; // 压力传感器是否校准标志位
    uint8_t fault_code;			  // 故障码低六位分别是六个电机的过扭矩标志
}; 

struct Finger_SendCmd
{
	uint8_t pitch_angle;		//取值范围0-255，手指根部弯曲
	uint8_t yaw_angle;			//取值范围0-255，手指间隙控制
	uint8_t roll_angle;			//取值范围0-255，手指横滚
	uint8_t tip_angle;			//取值范围0-255，指尖弯曲控制

	uint8_t speed;				//取值范围0-255，速度控制
	uint8_t over_current;		//取值范围0-255，过流设置
	uint8_t clear_fault;		//取值范围0-1，清除错误	
};
struct Finger_RecvState
{
	uint8_t pitch_angle;		//取值范围0-255，手指根部弯曲
	uint8_t yaw_angle;			//取值范围0-255，手指间隙控制
	uint8_t roll_angle;			//取值范围0-255，手指横滚
	uint8_t tip_angle;			//取值范围0-255，指尖弯曲控制

	uint8_t speed;				//取值范围0-255，速度控制
	uint8_t over_current;		//取值范围0-255，过流设置
	uint8_t fault;					
};
struct SendData_16DofHand
{
	Finger_SendCmd thumb;
	Finger_SendCmd index;
	Finger_SendCmd middle;
	Finger_SendCmd ring;
	Finger_SendCmd little;	 
};
struct RecvData_16DofHand
{
	Finger_RecvState thumb; 
	Finger_RecvState index;
	Finger_RecvState middle;
	Finger_RecvState ring;
	Finger_RecvState little;
};

namespace AIMOcommunicate
{
    class Can_Communication
    {
    public:
        Can_Communication();
        ~Can_Communication();

        bool Can_Communication_connect(const ros::NodeHandle& ros_nh);

        void RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg); 
        void LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr& msg); 
        void RightHandCtrl_pubFunction(void); 
        void LeftHandCtrl_pubFunction(void);  

    private:  
        void Can_Communication_LaunchRead();
        void Can_Communication_Init(void); 
        void Can_Communication_stop(void); 
		void Hand_InitDataSend(void);  
		void Hand_InitDataRecv(void); 

    private:   
        void CanSendDataDeal(const uint8_t ID, const uint8_t data[8], const uint8_t len);
        void CanSend_PressureData(const uint32_t ID, CanHand_SendData sendData); 
        void CanSend_6_9DofHand_DataDeal(const uint32_t ID, CanHand_SendData sendData);  
        void CanSend_10DofHand_DataDeal(const uint32_t ID, CanHand_SendData sendData); 

        int  CanRecvDataDeal(void);
        void CanRecv_6_9DofHand(const uint32_t ID, const uint8_t data[8]);   
        void CanRecv_10DofHand(const uint32_t ID, const uint8_t data[8]);   
        
        void AllSendDataDeal_16DofHand(void);
        void AllRecvDataDeal_16DofHand(void);
        void CanSend_16DofHand(const uint32_t ID);
        void CanRecv_16DofHand(const uint32_t ID, const uint8_t data[8]);

    public:  
        CanHand_RecvData RightHand_Recv, LeftHand_Recv;
        CanHand_SendData RightHand_Send, LeftHand_Send;
        SendData_16DofHand Hand_16Dof_Send;
        RecvData_16DofHand RightHand_16Dof_Recv, LeftHand_16Dof_Recv;

    private:
        general::GeneralFunction can_generalfun;

        ros::NodeHandle m_hand_can_nh;
        bool            HandEnable; 
        int             RightHandID;
        int             LeftHandID;
        int             Hand_Freedom; 
        int             JointVelocity; 
        int             JointEffort; 

        ros::Subscriber sub_right_hand;
        ros::Subscriber sub_left_hand;
        ros::Publisher  pub_right_hand;
        ros::Publisher  pub_left_hand;
	
	private:
        ros::Timer      m_handSend_timer; 
        ros::Timer      m_handRecv_timer; 
        bool            m_hand_isConnected;
        int             m_socket_can;   
    private:
        void createTimer(int seconds)
        {
            signal(SIGALRM, [](int) {});
            alarm(seconds);
        }
        void Hand_CanSendData_TimerEvent(const ros::TimerEvent& event); 
		void Hand_CanRecvData_TimerEvent(const ros::TimerEvent& event);    
		void Hand_CanRecvData(void); 
        
        struct can_frame can_send_frame;
        struct can_frame can_recv_frame;
    };
}  // namespace AIMOcommunicate
#endif
