#pragma once

#include "controller.h"
#include "dynamics.h"

#include <iostream>
#include <fstream> //for logging

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Command.h>

// when defined, this will create binary dumps 
// of the input and output of the controller
#define ROS_LOG_DUMP

class Controller_ROS
{
public:
    Controller_ROS();
    ~Controller_ROS();
    void odometry_callback(const nav_msgs::OdometryConstPtr &msg);

private: 
    // logging methods
    void log_controller(const double t);
    void init_log(std::string baseLogDir);
    // describes the logs we're creating
    // used as indicies into our array of output streams
    typedef enum { 
        ODOM_IN,
        COMMAND_OUT,
        TOTAL_LOGS
    } log_t;
    std::ofstream logs_[TOTAL_LOGS];

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