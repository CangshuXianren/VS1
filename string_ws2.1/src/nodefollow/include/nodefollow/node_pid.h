/************************************************
@jyf
函数名称 ： node_pid
函数功能 ： 
备注 ： 
*************************************************/

#ifndef NODEPID_H
#define NODEPID_H

#include "myGoal.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>
#include <nlink_parser/LinktrackAoaNodeframe0.h>

class NodePID
{
public:
    /*Constructor:
    pub     Publisher,发送指令给小车
    tol     Distance tolerance(m)
    tolA    Angle tolerance(度)
    dis     小车要跟随的距离
    ang     小车要跟随的方向角
    mSpeed  小车最大速度
    mASpeed 小车最大角速度
    */
    NodePID(ros::Publisher pub,double tol,double tolA,double dis,double ang,double mSpeed,double mASpeed);

    ~NodePID();

    /*本函数发布小车的控制量：
    angleCommand 角速度
    speedCommand 线速度
    */
    void publishMessage(double angleCommand,double speedCommand);

    /*本函数从uwb读取数据并将其处理为变量
    它保存当前的dis和ang，调用计算控制量及发布消息的函数
    msg Message,which contains odometry data
    */
    void messageCallback(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr& msg);

    /*本函数计算是否足够接近目标
    如果dis小于tol，则完成目标并且返回true
    actual 当前状态量
    */
    bool closeEnough(myGoal* actual);

    /*本函数通过PID控制器计算动作干预
    actualVal   当前输出量
    lastVal     上一步的输出量
    ref         参考量
    Kp、Ki、Kd
    sum         sum of errors
    */
    double calculatePID(myGoal* actual,double actualVal,double lastVal,double ref,double Kp,double Ki,double Kd,double* sum);

//变量
    myGoal* start;  //初始状态
    myGoal* last;   //上一时刻状态
    double tolerance;     //dis容忍度
    double toleranceA;    //ang容忍度
    double maxSpeed;  //最大线速度
    double maxASpeed; //最大角速度
    ros::Publisher pubMessage;  //发布消息

    //@string2.1
    const double dis_tar = 0.0; //跟随目标直至dis为0
    const double ang_tar = 0.0; //转动身体直至与目标相对角度为0
    bool flag; //判断当前是否为刚开始运行

    double dis_sum; //PID的sum of dis errors
    double ang_sum; //PID的sum of ang errors

    ros::Time time;
};

#endif