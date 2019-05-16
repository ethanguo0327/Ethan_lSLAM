//
// Created by ethan on 19-5-10.
//

#ifndef ETHAN_LSLAM_CALIB_ODOM_H
#define ETHAN_LSLAM_CALIB_ODOM_H
#include <stdlib.h>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>//inverse/houseQR...
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <csm/csm_all.h>//csm放在tf相关的后面的就没问题
class CalibOdom
{
public:
    explicit CalibOdom(int len);


    //进行pl-icp的相关函数.
    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,LDP& ldp);
        //求两帧之间的icp位姿匹配
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,Eigen::Vector3d tmprPose);
    //tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);
        //求解当前位姿　在　上一时刻　坐标系中的坐标
    Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose);
    //add data
    void prepareData(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg);
    bool Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan);
    void Set_data_len(int len);
    int data_len;
    int now_len;
    //solve the quation
    Eigen::Matrix3d Solve();
private:
    Eigen::MatrixXd  A;
    Eigen::VectorXd  b;

    std::string odom_frame_="odom";
    std::string base_frame_="base_link";
    tf::TransformListener tf_;
    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;
    //odom data
    Eigen::Vector3d now_pose,last_pose;
    std::vector<Eigen::Vector3d> odom_increments;          //用来储存两帧之间的里程计的增量
};


#endif //ETHAN_LSLAM_CALIB_ODOM_H
