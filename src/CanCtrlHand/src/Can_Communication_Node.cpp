
#include "Can_Communication.h"  

void exit_sig_handler(int signum)
{
    fprintf(stderr, "Ctrl-C caught, exit process...\n");
    ROS_WARN("Ctrl-C caught, exit process...\n");
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, exit_sig_handler);

    ros::init(argc, argv, "CanHandCtrl_node");
    ros::NodeHandle node;

    AIMOcommunicate::Can_Communication canHand;
    canHand.Can_Communication_connect(node);  
    ros::Rate loop_rate(10); // 设置发布频率（以每秒100次的速率发布数据） 

    while(ros::ok())
    {
        canHand.RightHandCtrl_pubFunction();
        canHand.LeftHandCtrl_pubFunction();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("CanHandCtrl_node over");
    return 0;
}