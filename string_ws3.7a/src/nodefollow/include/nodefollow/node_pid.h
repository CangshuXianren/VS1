/************************************************
@jyf
函数名称 ： node_pid
函数功能 ： pid跟随避障
备注 ： 
*************************************************/

#ifndef NODEPID_H
#define NODEPID_H

#include "myGoal.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <nlink_parser/TofsenseCascade.h>

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
    NodePID(ros::Publisher pub, double dis, double ang);

    ~NodePID();

    /*本函数发布小车的控制量：
    angleCommand 角速度
    speedCommand 线速度
    */
    void publishMessage(double angleCommand, double speedCommand);

    /*本函数通过激光测距值判断障碍物危险程度
    1——遇到障碍物，需要转向
    2——障碍物过近，需要倒车
    3——安全
    */
    void check_obs(double danger_dis_f, double danger_dis_fl, double danger_dis_fr);

    /*本函数接受tof距离并判断是否需要躲避
    通过修改全局变量if_obss来阻止继续执行跟踪而引发的抖动
    返回true代表仍有障碍物在前方，不能进行跟踪
    */
    void tofCallback(const nlink_parser::TofsenseCascade::ConstPtr &msg_tof);

    /*本函数从uwb读取数据并将其处理为变量
    它保存当前的dis和ang，调用计算控制量及发布消息的函数
    */
    void messageCallback(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr &msg);

    //因为使用的messageCallback_fortof里面有是否到达目标点的的判断，所以先不用这个
    // /*本函数计算是否足够接近目标
    // 如果dis小于tol，则完成目标并且返回true
    // actual 当前状态量
    // */
    // bool closeEnough(myGoal* actual);

    /*本函数通过PID控制器计算动作干预
    actualVal   当前输出量
    lastVal     上一步的输出量
    ref         参考量
    Kp、Ki、Kd
    sum         sum of errors
    */
    double calculatePID(myGoal *actual, double actualVal, double lastVal, double ref, double Kp, double Ki, double Kd, double *sum);

    //变量
    myGoal *start; //初始状态
    myGoal *last;  //上一时刻状态

    //@调试
    const double tolerance = 0.8;   //dis容忍度,由于uwb本身在安装距车头0.2m多的位置，为了和车头让出一个距离就之外另加了0.6m
    const double toleranceA = 10.0; //ang容忍度
    const double maxSpeed = 0.6;    //最大线速度,xtark最大线速度值为2
    const double maxASpeed = 0.9;   //最大角速度,xtark最大角速度值为1
    const double dis_tar = 0.0; //跟随目标直至dis为0
    const double ang_tar = 0.0; //转动身体直至与目标相对角度为0

    ros::Publisher pubMessage; //发布消息

    bool flag;                  //判断当前是否为刚开始运行

    double dis_sum; //PID的sum of dis errors
    double ang_sum; //PID的sum of ang errors

    int obs = 1;      // 障碍物信标
    //bool see = true;         // 避障转向信标,危险障碍物避障死循环嫌疑人
    bool if_forward = true; // 是否前进信标
    bool if_achieve = false; // 到达目的地信标

    ros::Time time;
};

#endif