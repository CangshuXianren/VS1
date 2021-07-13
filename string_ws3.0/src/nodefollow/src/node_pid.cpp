#include "nodefollow/node_pid.h"
#include <math.h>
#define PI 3.141592
//@调试
#define D_KP 0.3  // dis控制器的P
#define D_KI 0.2  // dis控制器的I
#define D_KD 0.0  // dis控制器的D
#define A_KP 0.08  // ang控制器的P
#define A_KI 0.02  // ang控制器的I
#define A_KD 0.02  // ang控制器的D
//@调试
#define DANGER_DIS  0.3  // 避障距离阈值(m) 
#define BACK_DIS 0.1  // 后退避障距离阈值(m)
extern bool do_follow = false;  // 障碍物信标


NodePID::NodePID(ros::Publisher pub, double tol, double tolA, double dis, double ang, double mSpeed, double mASpeed)
{
    tolerance = tol;
    toleranceA = tolA;
    maxSpeed = mSpeed;
    maxASpeed = mASpeed;
    pubMessage = pub;
    flag = true;
    dis_sum = 0.0;
    ang_sum = 0.0;
    start = new myGoal(0.0, 0.0, ros::Time::now());
    last = new myGoal(0.0, 0.0, ros::Time::now());
}

NodePID::~NodePID()
{
}

double NodePID::calculatePID(myGoal *actual, double actualVal, double lastVal, double ref, double Kp, double Kd, double Ki, double *sum)
{
    double speed = 0;
    double error = actualVal - ref;
    double previousError = lastVal - ref;
    double dt = actual->time.toSec() - last->time.toSec();
    double derivative = (error - previousError) / dt;
    *sum = *sum + error * dt;
    speed = Kp * error + Kd * derivative + Ki * (*sum);
    return speed; 
}


bool NodePID::closeEnough(myGoal *actual)
{
    if (fabs(actual->dis - dis_tar) > tolerance) return false;
    if ((fabs(actual->ang - ang_tar) > toleranceA)) return false;
    return true;
}

//Publisher
void NodePID::publishMessage(double angleCommand, double speedCommand)
{
    //preparing message
    geometry_msgs::Twist msg;

    msg.linear.x = speedCommand;
    msg.angular.z = angleCommand;

    //publishing message
    pubMessage.publish(msg);
}

void NodePID::tofCallback(const nlink_parser::TofsenseFrame0::ConstPtr& msg_tof)
{
    double danger_dis = msg_tof->dis;

    geometry_msgs::Twist msg;

    if(danger_dis <= DANGER_DIS && danger_dis > BACK_DIS)
    {
        ROS_WARN("Front Danger! : %f ",danger_dis);
        
        publishMessage(10.0, 0.05);  // 给10的角速度其实就是给最大转角

        ROS_WARN("---Avoiding By Turn---\n");

        do_follow = true;
        return;
    }
    else if(danger_dis <= BACK_DIS)
    {
        ROS_WARN("Front TOO Danger!!! : %f ",danger_dis);
        
        publishMessage(-10.0, -0.05);

        ROS_WARN("---Avoiding By Back---\n");

        do_follow = true;
        return;
    }
    else
    {
        do_follow = false;
        return; 
    }   
}


void NodePID::messageCallback(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr &msg)
{
    if(do_follow) return;
    double angleCommand = 0;
    double speedCommand = 0;

    myGoal *actual = new myGoal(msg->nodes[0].dis, msg->nodes[0].angle, ros::Time::now());
    ROS_INFO("---STATE---\n role[1]: \n dis : %f , angle : %f ", actual->dis, actual->ang);

    if (closeEnough(actual))
    {
        ROS_WARN("Goal Achieved!");
        publishMessage(0.0, 0.0);
        return;
    }
    if (flag)
    {
        start->dis = actual->dis;
        start->ang = actual->ang;
        start->time = actual->time;
        last->dis = actual->dis;
        last->ang = actual->ang;
        last->time = actual->time;
    }
    flag = false;


    //普通控制
    speedCommand = calculatePID(actual, actual->dis, last->dis, dis_tar, D_KP, D_KD, D_KI, &dis_sum);
    angleCommand = calculatePID(actual, actual->ang, last->ang, ang_tar, A_KP, A_KD, A_KI, &ang_sum);


    //Saving to last
    last->dis = actual->dis;
    last->ang = actual->ang;
    last->time = actual->time;

    //Invoking method for publishing message
    double finalspeeda = (fabs(angleCommand) < this->maxASpeed ? angleCommand : copysign( this->maxASpeed , angleCommand));
    double finalspeed = (fabs(speedCommand) < this->maxSpeed ? speedCommand : copysign( this->maxSpeed , speedCommand));
    publishMessage(finalspeeda, finalspeed);
    printf("---CONTROL---\nrole[1]:\nspeedCommand : %f ,angleCommand : %f\n" ,finalspeed , finalspeeda);

    //防止core dump
    delete actual;
}


