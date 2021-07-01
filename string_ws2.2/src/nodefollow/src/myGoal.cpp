#include"nodefollow/myGoal.h"

myGoal::myGoal(double dis,double ang, ros::Time t)
{
    this->dis = dis;
    this->ang = ang;
    this->time = t;
}

myGoal::~myGoal()
{

}