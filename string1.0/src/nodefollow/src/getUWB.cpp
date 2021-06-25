#include "ros/ros.h"
#include "nodefollow/node_pid.h"
#include <nlink_parser/LinktrackAoaNodeframe0.h>

void uwbtogoal(const nlink_parser::LinktrackAoaNode0::ConstPtr& msg)
{
    ROS_INFO("dis: %f , angle: %f ", msg->dis, msg->angle);
}

int main(int argc, char** argv)
{
    //Initialization of node
    ros::init(argc,argv,"getUWB");
    ros::NodeHandle n;

    //Creating subscriber for recieving uwb message
    ros::Subscriber sub = n.subscribe("/nlink_linktrack_aoa_nodeframe0", 1, uwbtogoal);

    ros::spin();

    return 0;
}