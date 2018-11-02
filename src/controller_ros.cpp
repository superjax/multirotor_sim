#include <controller_ros.h>

using namespace std;

Controller_ROS::Controller_ROS() :
    nh_("~")
{
    controller_ = controller::Controller();
    odometry_sub_ = nh_.subscribe("vi_ekf_node/odom", 100, &Controller_ROS::odometry_callback, this);
    command_pub_  = nh_.advertise<rosflight_msgs::Command>("command", 1);

    ros::NodeHandle nh_private("~");
    string param_file = "/home/len0rd/catkin_ws/src/multirotor_sim/params/sim_params.yaml";
    ROS_WARN_COND(!nh_private.getParam("param_file", param_file), "Didn't specify `param_file` parameter! Using default location...");

    controller_.load(param_file); //todo: change to ros params?
}

void Controller_ROS::odometry_callback(const nav_msgs::Odometry& msg)
{
    std::cout << "odom callback" << std::endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multirotor_controller_node");

    Controller_ROS controller_ros;

    ros::spin();

    return(0);
}