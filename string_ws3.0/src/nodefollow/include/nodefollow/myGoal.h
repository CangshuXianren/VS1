/************************************************
@jyf
函数名称 ： myGoal
函数功能 ： 实现获取一个anchor相对于tag的距离和角度并构造出一个目标
备注 ： 
*************************************************/

#ifndef MYGOAL_H
#define MYGOAL_H

#include <math.h>
#include "ros/ros.h"
#include <nlink_parser/LinktrackAoaNodeframe0.h>

class myGoal
{
public:
    /*Constructor:
    dis、ang
    t   Time of measurement
    */
    myGoal(double dis,double ang,ros::Time t);

    ~myGoal();

//变量
    double dis;
    double ang;
    ros::Time time; //Time. when the state was measured
};

#endif