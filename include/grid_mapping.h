//
// Created by ethan on 19-5-10.
//

#ifndef ETHAN_LSLAM_GRID_MAPPING_H
#define ETHAN_LSLAM_GRID_MAPPING_H
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
class myMapping
{
public :
    myMapping(){
    laserSub_=nodeHandler.subscribe("sick_scan",10,&myMapping::laserReceived,this);
    mapPub_=nodeHandler.advertise<nav_msgs::OccupancyGrid>("laserMap",1,true);
};
    void laserReceived(const sensor_msgs::LaserScan laserScan);
private:
    ros::NodeHandle nodeHandler;
    ros::Subscriber laserSub_;
    ros::Publisher  mapPub_;
};



#endif //ETHAN_LSLAM_GRID_MAPPING_H
