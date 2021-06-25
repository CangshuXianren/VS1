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