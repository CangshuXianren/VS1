#include"nodefollow/node_pid.h"
#include<math.h>
#define PI 3.141592
//@jyf这里需要调试
#define D_KP 1.0//dis控制器的P
#define D_KD 1.0//dis控制器的D
#define D_KI 1.0//dis控制器的I
#define A_KP 1.0//ang控制器的P
#define A_KI 1.0//ang控制器的I
#define A_KD 1.0//ang控制器的D

NodePID::NodePID(ros::Publisher pub,double tol,double tolA,double dis,double ang,double mSpeed,double mASpeed)
{
    tolerance = tol;
    toleranceA = tolA;
    maxSpeed = mSpeed;
    maxASpeed = mASpeed;
    pubMessage = pub;
    iterations = 0;
    dis_sum = 0;
    ang_sum = 0;
    start = new myGoal(0.0,0.0,ros::Time::now());
    last = new myGoal(0.0,0.0,ros::Time::now());
}

NodePID::~NodePID()
{

}

double NodePID::calculatePID(myGoal* actual,double actualVal,double lastVal,double ref,double Kp,double Kd,double Ki,double* sum)
{
    double speed = 0;
    double error = ref - actualVal;
    double previousError = ref - lastVal;
    double dt = actual->time.toSec() - last->time.toSec();
    double derivative = (error - previousError)/dt;
    *sum = *sum + error*dt;
    speed = Kp*error + Kd *derivative + Ki*(*sum);
    return speed;//这里虽然返回的叫“speed”，但是实际上对于angle可复用，因为pid计算是一样的
}

//Publisher
void NodePID::publishMessage(double angleCommand,double speedCommand)
{
    //preparing message
    geometry_msgs::Twist msg;

    msg.linear.x = speedCommand;
    msg.angular.z = angleCommand;

    //publishing message
    pubMessage.publish(msg);
}

//Subscriber
void NodePID::messageCallback(const nlink_parser::LinktrackAoaNode0::ConstPtr& msg)
{
    double angleCommand = 0;
    double speedCommand = 0;
    myGoal* actual = new myGoal(msg->dis, msg->angle, ros::Time::now());
    if(closeEnough(actual)==true)
    {
        ROS_INFO("Goal Achieved");
        publishMessage(0.0,0.0);
        exit(0);
    }
    if(iterations == 0)
    {
        start->dis = actual->dis;
        start->ang = actual->ang;
        start->time = actual->time;
        last->dis = actual->dis;
        last->ang = actual->ang;
        last->time = actual->time;
    }
    iterations++;

    //Calculation of action intervention@jyf
    //在计算速度和角度的command的时候用的都是增量，这里直接用状态量,因此对两个求commnand的method都进行了修改
    if(fabs(dis_tar) > tolerance)
    {
        speedCommand = calculatePID(actual,actual->dis,last->dis,dis_tar,D_KP,D_KD,D_KI,&dis_sum);
    }

    if(actual->ang - last->ang < -PI)//这两段应该是为了将角度偏差控制在±180度
    {
        actual->ang += 2*PI;
    }
    if(actual->ang - last->ang > PI)
    {
        actual->ang -= 2*PI;
    }

    if(fabs(ang_tar) > toleranceA)
    {
        angleCommand = calculatePID(actual,actual->ang,last->ang,ang_tar,A_KP,A_KD,A_KI,&ang_sum);
    }

    //Saving to last
    last->dis = actual->dis;
    last->ang = actual->ang;
    last->time = actual->time;

    //Invoking method for publishing message
    publishMessage(fmin(maxASpeed,angleCommand),fmin(maxSpeed,speedCommand));
}

bool NodePID::closeEnough(myGoal* actual)////类似于Calculation of action intervention进行了参数修改@jyf
{
    double distance;
    distance = actual->dis;
    if(fabs(distance - dis_tar) > tolerance) return false;
    if((fabs(ang_tar - actual->ang) > toleranceA) && 
        (fabs(ang_tar - actual->ang + 2*PI) > toleranceA) &&
        (fabs(ang_tar - actual->ang - 2*PI) > toleranceA))  return false;
    return true;
}

