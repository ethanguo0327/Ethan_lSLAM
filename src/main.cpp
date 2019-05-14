#include <grid_mapping.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ethan_lslam");

    myMapping myMapping_;

    ros::spin();
    return 0;
}