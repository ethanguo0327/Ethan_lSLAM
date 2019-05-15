//
// Created by ethan on 19-5-10.
//

#include "../include/grid_mapping.h"
#include "../include/calib_odom.h"

myMapping::myMapping(){
    laserSub_=nodeHandler.subscribe("sick_scan",10,&myMapping::laserReceived,this);
    mapPub_  =nodeHandler.advertise<nav_msgs::OccupancyGrid>("laserMap",1,true);
    use_calib_odom_=true;//TODO:set by hand temporarily

    if(use_calib_odom_)
        calib_odomPtr = std::make_shared<CalibOdom>();

}
void myMapping::laserReceived(const sensor_msgs::LaserScan laserScan) {
    std::cout<<"laser received "<<std::endl;

    if(use_calib_odom_)
        calib_odomPtr->countscan();
    if(calib_odomPtr->getscan()>60)
        std::cout<<calib_odomPtr->gettemp(55)<<std::endl;
}
