/************************************************
@jyf
功能包 ： string_ws3.0
功能 ： AOA跟随TOF避障
进度 ： 
1.实现一个安装了anchor的小车跟踪一个自由tag并使用TOF避障
2.需要整体优化的代码部分标记为"@调试"
*************************************************/

#include "nodefollow/myGoal.h"
#include "nodefollow/node_pid.h"

#define PI 3.141592

//@调试
#define TOLERANCE 0.8  // 由于uwb本身在安装距车头0.2m多的位置，为了和车头让出一个距离就之外另加了0.6m
#define TOLERANCEA 10.0 // Difference from targer angle, which will be tolerated
#define MAXSPEED 0.4
#define MAXASPEED 0.7



int main(int argc, char** argv)
{
    //初始化节点
    ros::init(argc,argv,"nodefollow");
    ros::NodeHandle n;

    //初始化状态量为0
    double distance = 0.0;
    double angle = 0.0;

    //创建publisher用于与底盘通讯
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    /*构建pid算法对象来完成：
    * 控制量的计算 ： calculatePID
    * 是否到达终点的判断 ： closeEnough
    * 控制量的发布 ： publishMessage
    * 数据的储存及小车状态的更新（回调函数） ： messageCallback
    */
    NodePID* nodePID = new NodePID(pubMessage, TOLERANCE, TOLERANCEA, distance, angle, MAXSPEED, MAXASPEED);


    //创建subscriber用于订阅tof消息
    ros::Subscriber sub_tof = n.subscribe("/nlink_tofsense_frame0", 100 , &NodePID::tofCallback, nodePID);
    //创建subscriber用于订阅uwb消息
    ros::Subscriber sub_uwb = n.subscribe("/nlink_linktrack_aoa_nodeframe0", 100 , &NodePID::messageCallback, nodePID);

    ros::spin();

    return 0;
}
