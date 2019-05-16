//
// Created by ethan on 19-5-10.
//

#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include "../include/calib_odom.h"
#include "calib_odom.h"

CalibOdom::CalibOdom(int len) {
    std::cout<<"start Calib odom ..."<<std::endl;
    now_len=0;
    SetPIICPParams();
    Set_data_len(len);
}
void CalibOdom::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}
void CalibOdom::prepareData(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg)
{
    sensor_msgs::LaserScan scan;
    Eigen::Vector3d odom_pose;              //激光对应的里程计位姿
    Eigen::Vector3d d_point_odom;           //里程计计算的dpose
    Eigen::Vector3d d_point_scan;           //激光的scanmatch计算的dpose
    Eigen::MatrixXd transform_matrix(3,3);  //临时的变量

    double c,s;
    scan = *_laserScanMsg;

    //得到对应的里程计数据
    if(!getOdomPose(odom_pose, _laserScanMsg->header.stamp))
    return ;

    //前后两帧里程计的位姿差
    d_point_odom = cal_delta_distence(odom_pose);

    //如果运动的距离太短，则不进行处理．
    if(d_point_odom(0) < 0.05 &&
    d_point_odom(1) < 0.05 &&
    d_point_odom(2) < tfRadians(5.0))
    {
    return ;
    }

    //记录下里程计的增量数据
    odom_increments.push_back(d_point_odom);

    //把当前的激光数据转换为 pl-icp能识别的数据 & 进行矫正
    //d_point_scan就是用激光计算得到的两帧数据之间的旋转 & 平移
    LDP currentLDP;
    if(m_prevLDP != NULL)
    {
    LaserScanToLDP(&scan,currentLDP);
    d_point_scan = PIICPBetweenTwoFrames(currentLDP,d_point_odom);
    }
    else
    {
    LaserScanToLDP(&scan,m_prevLDP);
    }

    //构造超定方程组
    Add_Data(d_point_odom,d_point_scan);

    std::cout <<"scanCnt:"<<now_len<<std::endl;
}

bool CalibOdom::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //构建超定方程组
        A.block<1,3>(3*now_len  ,0)<<Odom[0],Odom[1],Odom[2];
        A.block<1,3>(3*now_len+1,3)<<Odom[0],Odom[1],Odom[2];
        A.block<1,3>(3*now_len+2,6)<<Odom[0],Odom[1],Odom[2];
        b.segment(3*now_len,3)<<scan[0],scan[1],scan[2];

        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}
//------------scan match----------------
//设置PI-ICP的参数
void CalibOdom::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 20;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}

//把激光雷达数据 转换为PI-ICP需要的数据
void CalibOdom::LaserScanToLDP(sensor_msgs::LaserScan *pScan,LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

//求两帧之间的icp位姿匹配
Eigen::Vector3d  CalibOdom::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                              Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);

//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;

//        std::cout <<"PI ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }

    //更新

    //ld_free(m_prevLDP);

    m_prevLDP = currentLDPScan;

    return rPose;
}
//------------odom pose-----------------
//得到时刻t时候 机器人在里程计坐标下的坐标
bool CalibOdom::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }


    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());



    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    //pub_msg(pose,path_odom,odom_path_pub_);
    return true;
}

//即求解当前位姿在上一时刻坐标系中的坐标
Eigen::Vector3d  CalibOdom::cal_delta_distence(Eigen::Vector3d odom_pose)
{

    Eigen::Vector3d d_pos;  //return value
    now_pose = odom_pose;

    //TODO:
    Eigen::Matrix3d To_new;
    Eigen::Matrix3d To_old;
    To_new<<cos(now_pose[2]),-sin(now_pose[2]),now_pose[0],sin(now_pose[2]),cos(now_pose[2]),now_pose[1],0,0,1;
    To_old<<cos(last_pose[2]),-sin(last_pose[2]),last_pose[0],sin(last_pose[2]),cos(last_pose[2]),last_pose[1],0,0,1;

    Eigen::Matrix3d Told_new;
    Told_new=To_old.inverse()*To_new;
    d_pos<<Told_new(0,2),Told_new(1,2),atan2(Told_new(1,0),Told_new(0,0));
    //end of TODO:
    last_pose = now_pose;
    return d_pos;
}

//------------solve---------------------
Eigen::Matrix3d CalibOdom::Solve(){

    Eigen::Matrix3d correct_matrix;

    //求解线性最小二乘
    Eigen::VectorXd correct_vector(9);

    Eigen::MatrixXd tmp_A(9, 9);
    Eigen::MatrixXd tmp_B(9, 9);
    tmp_A=A.transpose()*A;
    tmp_B=A.transpose()*b;

    correct_vector=tmp_A.householderQr().solve(tmp_B);
    correct_matrix<<correct_vector.segment(0,3),correct_vector.segment(3,3),correct_vector.segment(6,3);
    correct_matrix.transposeInPlace();
    return correct_matrix;
}