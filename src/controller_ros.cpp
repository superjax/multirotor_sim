#include <controller_ros.h>


Controller_ROS::Controller_ROS() :
    nh_("~")
{
    controller_ = controller::Controller();
    controller.load("~/catkin_ws/src/multirotor_sim/params/sim_params.yaml"); //todo: change to ros params?
    odometry_sub_ = nh.subscribe("nav_msgs/Odometry", 100, &odometry_callback, this);
    command_pub_  = nh.advertise<rosflight_io::Command>("command", 100);
}

Controller_ROS::odometry_callback(const nav_msgs::Odometry& msg)
{
    std::cout << "odom callback" << endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multirotor_controller_node");

    Controller_ROS controller_ros;

    ros::spin();

    return(0);
}