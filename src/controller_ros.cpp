#include <controller_ros.h>




void odometry_callback(const nav_msgs::Odometry& msg){

}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multirotor_controller_node");

    ros::NodeHandle nh;

    ros::Subscriber odometry_sub_ = nh.subscribe("nav_msgs/Odometry", 1, &odometry_callback, this);
    ros::Publisher command_pub_ = nh.advertise<rosflight_io::Command>()

    ros::spin();

    return(0);
}