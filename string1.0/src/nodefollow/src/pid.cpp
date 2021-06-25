#include "nodefollow/myGoal.h"
#include "nodefollow/node_pid.h"

//@jyf
#define SUBSCRIBER_BUFFER_SIZE 1    // Size of buffer for subscriber
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher
#define TOLERANCE 0.01  // Distance from the target, at which the distance will be considered as chieved
#define TOLERANCEA 0.02 // Difference from targer angle, which will be tolerated
#define MAXSPEED 0.5
#define MAXASPEED 2.0
#define PI 3.141592
//这一部分可能需要修改@jyf
#define PUBLISHER_TOPIC "/cmd_vel"
#define SUBSCRIBER_TOPIC "odom"

int main(int argc, char** argv)
{
    //Initialization of node
    ros::init(argc,argv,"pid");
    ros::NodeHandle n;

    if(argc < 3)
    {
        printf("You must provide two arguments\n Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
        exit(0);
    }
    double distance = 0.0;
    double angle = 0.0;

    // //@jyf这一段应该是检测参数传入是否正常的，但是含义还没弄清楚
    // if (strcmp(argv[link](#1),"-f") == 0)
    // {
    //     if (1 == sscanf(argv[link](#2), "%f", &distance))
    //         {
    //             ROS_INFO("Going forward %f meters.", distance);
    //         }
    //     else
    //         {
    //             ROS_INFO("Can not parse second argument.\nYou must provide numeric argument\n");
    //             ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
    //             exit(0);
    //         }
    // }
    // else if(strcmp(argv[link](#1),"-r") == 0)
    // {
    //     if (1 == sscanf(argv[link](#2), "%f", &angle))
    //     {
    //         //Normalizing angle to interval <-pi-xi;pi+xi>
    //         while (fabs(angle) > PI+0.01)
    //         {
    //             if (angle > 0)
    //             {
    //                 angle -= 2*PI;
    //             }
    //             else
    //             {
    //                 angle += 2*PI;
    //             }
    //         }
    //         ROS_INFO("Turning by %f radians.", angle);
    //     }
    //     else
    //     {
    //         ROS_INFO("Can not parse second argument.\nYou must provide numeric argument\n");
    //         ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians\n");
    //         exit(0);
    //     }
    // }
    // else
    // {
    //     ROS_INFO("Usage\n -f [link](#x) move forward x meters\n -r [link](#x) turn around by x radians");
    //     exit(0);
    // }

    //Creating publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

    //Creating object, which stores data from sensors and has methods for publishing and subscribing
    NodePID* nodePID = new NodePID(pubMessage, TOLERANCE, TOLERANCEA, distance, angle, MAXSPEED, MAXASPEED);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodePID::messageCallback, nodePID);

    return 0;
}