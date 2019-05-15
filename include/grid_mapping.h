//
// Created by ethan on 19-5-10.
//

#ifndef ETHAN_LSLAM_GRID_MAPPING_H
#define ETHAN_LSLAM_GRID_MAPPING_H
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include "calib_odom.h"
class myMapping
{
public :
    myMapping();
    void laserReceived(const sensor_msgs::LaserScan laserScan);
private:
    ros::NodeHandle nodeHandler;
    ros::Subscriber laserSub_;
    ros::Publisher  mapPub_;

    bool use_calib_odom_;//TODO:should be set by parameter
    std::shared_ptr<CalibOdom> calib_odomPtr;

};



#endif //ETHAN_LSLAM_GRID_MAPPING_H
