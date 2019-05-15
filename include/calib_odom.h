//
// Created by ethan on 19-5-10.
//

#ifndef ETHAN_LSLAM_CALIB_ODOM_H
#define ETHAN_LSLAM_CALIB_ODOM_H

#include <zconf.h>
#include <stdlib.h>
#include <vector>
class CalibOdom
{
public:
    CalibOdom();
    void countscan();
    int gettemp(int i);
    int getscan();
private:
    std::vector<int > temp;
    static int scanCnt_;
};
int CalibOdom::scanCnt_=0;//类的静态成员变量需要在类外分配内存空间

#endif //ETHAN_LSLAM_CALIB_ODOM_H
