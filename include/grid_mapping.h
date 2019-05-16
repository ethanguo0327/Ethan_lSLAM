//
// Created by ethan on 19-5-10.
//

#ifndef ETHAN_LSLAM_GRID_MAPPING_H
#define ETHAN_LSLAM_GRID_MAPPING_H
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <eigen3/Eigen/Core>
#include "calib_odom.h"
class myMapping
{
public :
    myMapping();

    void laserReceived(const sensor_msgs::LaserScan::ConstPtr&scan);

private:
    ros::NodeHandle nodeHandler;
    std::string odom_frame_="odom";
    std::string base_frame_="base_link";
    tf::TransformListener tf_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    //odom corrector
    bool use_calib_odom_;//TODO:should be set by parameter
    std::shared_ptr<CalibOdom> calib_odomPtr= nullptr;
    Eigen::Matrix3d odomCorrector_;
};



#endif //ETHAN_LSLAM_GRID_MAPPING_H
