#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ethan_lslam");
    ros::NodeHandle nodeHandler;
    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);
    ros::spin();
    return 0;
}