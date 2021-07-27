/************************************************
@jyf
功能包 ： string_ws
功能 ： AOA跟随TOF避障
进度 ： 实现一个安装了anchor的小车跟踪一个自由tag并使用TOF避障
*************************************************/

#include "nodefollow/myGoal.h"
#include "nodefollow/node_pid.h"

#define PI 3.141592



int main(int argc, char** argv)
{
    //休眠三秒为人操作作准备
    sleep(3);
    //初始化节点
    ros::init(argc,argv,"nodefollow");
    ros::NodeHandle n;

    //初始化状态量为0
    double distance = 0.0;
    double angle = 0.0;

    //创建publisher用于与底盘通讯
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>("/cmd_vel", 200);

    /*构建pid算法对象来完成：
    * 控制量的计算 ： calculatePID
    * 是否到达终点的判断 ： closeEnough
    * 控制量的发布 ： publishMessage
    * 数据的储存及小车状态的更新（回调函数） ： messageCallback
    */
    NodePID* nodePID = new NodePID(pubMessage, distance, angle);

    //创建subscriber用于订阅uwb消息并控制跟踪
    ros::Subscriber sub_uwb = n.subscribe("/nlink_linktrack_aoa_nodeframe0", 10 , &NodePID::messageCallback, nodePID);
    //创建subscriber用于订阅tof消息并控制避障
    ros::Subscriber sub_tof = n.subscribe("/nlink_tofsense_cascade", 10 , &NodePID::tofCallback, nodePID);

    // ros::Rate loop_rate(10);
    // while(ros::ok())
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }
    ros::spin();

    return 0;
}
