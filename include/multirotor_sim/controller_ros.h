#include "controller.h"
#include "dynamics.h"

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Command.h>

class Controller_ROS
{
public:
    Controller_ROS();
    void odometry_callback(const nav_msgs::Odometry& msg);

private: 
    ros::NodeHandle nh_;
    ros::Subscriber odometry_sub_;
    ros::Publisher command_pub_;
    rosflight_msgs::Command command_msg_;
    controller::Controller controller_;
    dynamics::xVector parsed_odom_;
    dynamics::commandVector command_out_;
    ros::Time start_time_;
    bool odom_init_ = false;
}; 