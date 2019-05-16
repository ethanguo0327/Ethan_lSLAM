//
// Created by ethan on 19-5-10.
//

#include "../include/grid_mapping.h"
#include "../include/calib_odom.h"

myMapping::myMapping(){
    //里程计和激光雷达数据的同步
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nodeHandler, "/sick_scan", 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 10);
    scan_filter_->registerCallback(boost::bind(&myMapping::laserReceived, this, _1));

    use_calib_odom_=true;//TODO:set by hand temporarily
    if(use_calib_odom_)
//        calib_odomPtr=new CalibOdom(50);
//        calib_odomPtr = std::make_shared<CalibOdom>(CalibOdom(50));//error
        odomCorrector_.setIdentity();
        calib_odomPtr = std::shared_ptr<CalibOdom>(new CalibOdom(50));
}


void myMapping::laserReceived(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::cout<<"laser received "<<std::endl;
    //calib odometry by scanmatch
    if(use_calib_odom_)
    {
        if(calib_odomPtr->now_len<calib_odomPtr->data_len)
            calib_odomPtr->prepareData(scan);
        else
        {
            odomCorrector_=calib_odomPtr->Solve();
            std::cout<<"odom correct matrix solved !"<<std::endl;
            std::cout<<odomCorrector_<<std::endl;
            use_calib_odom_=false;
        }
    }


}
