//
// Created by ethan on 19-5-10.
//

#include <iostream>
#include "../include/calib_odom.h"
CalibOdom::CalibOdom() {
    std::cout<<"start Calib odom ..."<<std::endl;
}
void CalibOdom::countscan(){
    scanCnt_++;
    temp.push_back(scanCnt_);
}
int CalibOdom::gettemp(int i) {
    return temp[i];
}
int CalibOdom::getscan() {
    return scanCnt_;
}