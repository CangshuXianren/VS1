#include "nodefollow/node_pid.h"
#include <math.h>
#define PI 3.141592
//@调试
#define D_KP 300.0 //dis控制器的P
#define D_KD 0.0   //dis控制器的D
#define D_KI 200.0 //dis控制器的I
#define A_KP 300.0 //ang控制器的P
#define A_KI 0.0   //ang控制器的I
#define A_KD 200.0 //ang控制器的D

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
    return speed; //这里虽然返回的叫“speed”，但是实际上对于angle可复用，因为pid计算是一样的
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

//Subscriber
void NodePID::messageCallback(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr &msg)
{
    double angleCommand = 0;
    double speedCommand = 0;

    //@string2.0
    myGoal *actual = new myGoal(msg->nodes[0].dis, msg->nodes[0].angle, ros::Time::now());
    //show current state
    ROS_INFO("(STATE)\n role[1]: \n dis : %f , angle : %f ", actual->dis, actual->ang);

    if (closeEnough(actual))
    {
        ROS_INFO("Goal Achieved!");
        publishMessage(0.0, 0.0);
        exit(0);
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

    //Calculation of action intervention
    //@调试
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
    //show control
    ROS_INFO("(CONTROL)\nrole[1]:\nspeedCommand : %f ,angleCommand : %f" ,finalspeed , finalspeeda);

    //防止core dump
    delete actual;
}


