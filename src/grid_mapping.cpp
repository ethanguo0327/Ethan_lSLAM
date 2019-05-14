//
// Created by ethan on 19-5-10.
//

#include "../include/grid_mapping.h"
void myMapping::laserReceived(const sensor_msgs::LaserScan laserScan) {
    std::cout<<"laser received "<<std::endl;
    std::cout<<"laser sequence is: "<<laserScan.header.seq<<std::endl;
}
