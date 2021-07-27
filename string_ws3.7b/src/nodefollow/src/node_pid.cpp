#include "nodefollow/node_pid.h"
#include <math.h>
#define PI 3.141592
//@调试
#define D_KP 0.3  // dis控制器的P
#define D_KI 0.2  // dis控制器的I
#define D_KD 0.0  // dis控制器的D
#define A_KP 0.03  // ang控制器的P
#define A_KI 0.02  // ang控制器的I
#define A_KD 0.01  // ang控制器的D
//@调试(目前正前方和侧方危险参数相同)
#define DANGER_DIS_F  0.4  // 正前方避障距离阈值(m)，xtark的最小转向半径是0.3 
#define BACK_DIS_F 0.2  // 正前方后退避障距离阈值(m)



NodePID::NodePID(ros::Publisher pub, double dis, double ang)
{
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

//这是3.6版本之前的代码，当时是三个回调函数，现在直接将三个集成为了两个
// bool NodePID::closeEnough(myGoal *actual)
// {
//     if (fabs(actual->dis - dis_tar) > tolerance) return false;
//     if ((fabs(actual->ang - ang_tar) > toleranceA)) return false;
//     return true;
// }

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

int NodePID::check_obs(double danger_dis_f,double danger_dis_fl,double danger_dis_fr)
{
    if((danger_dis_f > DANGER_DIS_F || danger_dis_f < 1e-5) &&
        (danger_dis_fl > DANGER_DIS_F || danger_dis_fl < 1e-5) &&
        (danger_dis_fr > DANGER_DIS_F || danger_dis_fr < 1e-5)
    )
        {
            //obs = 0;
            return 0;
        }
    else if((danger_dis_f <= BACK_DIS_F && danger_dis_f >= 1e-5) ||
            (danger_dis_fl <= BACK_DIS_F && danger_dis_fl >= 1e-5) ||
            (danger_dis_fr <= BACK_DIS_F && danger_dis_fr >= 1e-5))
        {
            //obs = 2;
            return 2;
        }
    else 
    {
        //obs = 1;
        return 1;
    }
}

void NodePID::tofCallback(const nlink_parser::TofsenseCascade::ConstPtr& msg_tof)
{
    if(if_achieve) return;
    double danger_dis_f = msg_tof->nodes[0].dis;  // 正前方激光测距值
    double danger_dis_fl = msg_tof->nodes[1].dis;  // 左前方激光测距值
    double danger_dis_fr = msg_tof->nodes[2].dis;  // 右前方激光测距值

    int temp_obs = check_obs(danger_dis_f,danger_dis_fl,danger_dis_fr);
    if(obs == temp_obs) circle = true;
    else circle = false;
    obs = temp_obs;

    if(obs == 1)
    {
        ROS_WARN("\n---Front Danger!---\nFLeft : %f || Front : %f || FRight : %f ",danger_dis_fl, danger_dis_f, danger_dis_fr);
        
        // if(see) 
        // {
        //     if(!circle)
        //     {
        //         publishMessage(maxASpeed,0.0);
        //         sleep(1.7);
        //     }
        //     publishMessage(maxASpeed, 0.8*maxSpeed);
        //     printf("---Avoiding By TURN, Go to Left---\n");
        // }
        // else 
        // {
        //     if(!circle)
        //     {
        //         publishMessage(-maxASpeed,0.0);
        //         sleep(1.7);
        //     }
        //     publishMessage(-maxASpeed, 0.8*maxSpeed);
        //     printf("---Avoiding By TURN, Go to Right---\n");
        // }
        if(!circle)
        {
            publishMessage(maxASpeed,0.0);
            sleep(1.7);
        }
        publishMessage(maxASpeed, 0.8*maxSpeed);
        printf("---Avoiding By TURN, Go to Left---\n");
        if_forward = true;
        
        return;
    }
    else if(obs == 2)
    {
        ROS_WARN("\n---Front Too Danger!!!---\nFLeft : %f || Front : %f || FRight : %f ", danger_dis_fl, danger_dis_f, danger_dis_fr);
        
        if(if_forward)
        {
            publishMessage(0.0, 0.0);
            if_forward = false;
            obs = 3;
            printf("---Avoiding By BACK, Just Stop---\n");
        }
        else
        {
            // if(see) 
            // {
            //     if(!circle)
            //     {
            //         publishMessage(-maxASpeed,0.0);
            //         sleep(1.7);
            //     }
            //     printf("---Avoiding By BACK, Go to Left---\n");
            //     publishMessage(-maxASpeed, -0.2*maxSpeed);
            // }
            // else 
            // {
            //     if(!circle)
            //     {
            //         publishMessage(maxASpeed,0.0);
            //         sleep(1.7);
            //     }
            //     printf("---Avoiding By BACK, Go to Right---\n");
            //     publishMessage(maxASpeed, -0.2*maxSpeed);
            // }
            if(!circle)
            {
                publishMessage(-maxASpeed,0.0);
                sleep(1.7);
            }
            printf("---Avoiding By BACK, Go to Left---\n");
            publishMessage(-maxASpeed, -0.2*maxSpeed);
        }
        if_forward = false;

        return;
    }
    else
    {
        return; 
    }   
}


void NodePID::messageCallback(const nlink_parser::LinktrackAoaNodeframe0::ConstPtr &msg_aoa)
{
    //判断接受aoa是否正常(core dump核心原因)
    if(msg_aoa->nodes.size()==0) 
    {
        publishMessage(0.0,0.0);
        return;
    }

    //是否到达目标点
    if ((msg_aoa->nodes[0].dis - dis_tar) > tolerance) if_achieve = false;
    else if ((msg_aoa->nodes[0].angle - ang_tar) > toleranceA) if_achieve = false;
    else if_achieve = true;

    // //避障优先方向判断
    // double tarside = msg_aoa->nodes[0].angle;
    // if(tarside >= 0) see = true;
    // else see = false;

    if(obs && (!if_achieve)) return;
    if (if_achieve)
    {
        ROS_WARN("Goal Achieved!");
        publishMessage(0.0, 0.0);
        if_forward = false;
        return;
    }

    if(!if_forward) 
    {
        ROS_WARN("---CONTROL---\nJust stop\n");
        publishMessage(0.0, 0.0);
        if_forward = true;
        return;
    }


    double angleCommand = 0.0;
    double speedCommand = 0.0;

    myGoal *actual = new myGoal(msg_aoa->nodes[0].dis, msg_aoa->nodes[0].angle, ros::Time::now());
    ROS_INFO("\n---STATE---\n num.3: \n dis : %f , angle : %f ", actual->dis, actual->ang);

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
    printf("---CONTROL---\nnum.3:\nspeedCommand : %f ,angleCommand : %f\n" ,finalspeed , finalspeeda);
    publishMessage(finalspeeda, finalspeed);
    if_forward = true;

}


