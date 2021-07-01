/************************************************
@jyf
功能包 ： string_ws2.1
功能 ： AOA跟随
进度 ： 
1.实现一个安装了anchor的小车跟踪一个自由tag
2.所有出现”msg->nodes[0]“的地方在以后实现多车编队时都要改动，需要改动的地方标记为"@string2.0"
3.需要整体优化的代码部分标记为"@调试"
4.目前无法处理tag在anchor后半平面的问题，即angle在（90°～180°）与（-90°～-180°）的区间，nooploop给出方法是可以通过信号强度来进行判断前后平面
*************************************************/

#include "nodefollow/myGoal.h"
#include "nodefollow/node_pid.h"

#define PI 3.141592

//@调试
#define TOLERANCE 0.1  // Distance from the target, at which the distance will be considered as chieved
#define TOLERANCEA 5.0 // Difference from targer angle, which will be tolerated
#define MAXSPEED 1.0
#define MAXASPEED 2.0



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

    //创建subscriber用于完成订阅uwb消息的收集
    ros::Subscriber sub = n.subscribe("/nlink_linktrack_aoa_nodeframe0", 100 , &NodePID::messageCallback, nodePID);
    ros::spin();

    return 0;
}