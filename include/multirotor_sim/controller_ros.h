#include "controller.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>

class Controller_ROS
{
public:
    Controller_ROS();
    void odometry_callback(const nav_msgs::Odometry& msg);

private: 
    ros::NodeHandle nh_;
    ros::Subscriber odometry_sub_;
    ros::Publisher command_pub_;
    controller::Controller controller_;
};