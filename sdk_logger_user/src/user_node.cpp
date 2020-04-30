#include <ros/ros.h>
#include <sdk_logger/sdk_logger.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_node");
    ros::NodeHandle nh;

    sdk_logger::boop();

    ros::spin();
    return 0;
}
