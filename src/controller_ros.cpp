#include <controller_ros.h>

using namespace std;

Controller_ROS::Controller_ROS()
{
    controller_ = controller::Controller();
    odometry_sub_ = nh_.subscribe("odom", 100, &Controller_ROS::odometry_callback, this);
    command_pub_  = nh_.advertise<rosflight_msgs::Command>("command", 1);

    ros::NodeHandle nh_private("~");
    string param_file = "/home/len0rd/catkin_ws/src/multirotor_sim/params/sim_params.yaml";
    ROS_WARN_COND(!nh_private.getParam("param_file", param_file), "Didn't specify `param_file` parameter! Using default location...");

    controller_.load(param_file); //todo: change to ros params?
}

void Controller_ROS::odometry_callback(const nav_msgs::Odometry &msg)
{   
    // parse message
    //todo: TYLER -> why is it msg. instead of msg-> ??
    parsed_odom_(dynamics::PX, 0) = msg.pose.pose.position.x;
    parsed_odom_(dynamics::PY, 0) = msg.pose.pose.position.y;
    parsed_odom_(dynamics::PZ, 0) = msg.pose.pose.position.z;

    parsed_odom_(dynamics::VX, 0) = msg.twist.twist.linear.x;
    parsed_odom_(dynamics::VY, 0) = msg.twist.twist.linear.y;
    parsed_odom_(dynamics::VZ, 0) = msg.twist.twist.linear.z;

    parsed_odom_(dynamics::QW, 0) = msg.pose.pose.orientation.w;
    parsed_odom_(dynamics::QX, 0) = msg.pose.pose.orientation.x;
    parsed_odom_(dynamics::QY, 0) = msg.pose.pose.orientation.y;
    parsed_odom_(dynamics::QZ, 0) = msg.pose.pose.orientation.z;

    parsed_odom_(dynamics::WX, 0) = msg.twist.twist.angular.x;
    parsed_odom_(dynamics::WY, 0) = msg.twist.twist.angular.y;
    parsed_odom_(dynamics::WZ, 0) = msg.twist.twist.angular.z;

    //figure out time stuff:


    //compute control

    // computeControl(parsed_odom_, TIME_STAMPPPPP, command_out);

    // publish resulting command
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multirotor_controller_node");
    Controller_ROS controller_ros;

    ros::spin();

    return(0);
}