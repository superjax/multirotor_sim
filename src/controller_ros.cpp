#include <controller_ros.h>
#include <ros/package.h>

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

    #ifdef ROS_LOG_DUMP
        std::string defaultLogDir = ros::package::getPath("multirotor_sim") + "/logs"; //+ to_string(ros::Time::now().sec);
        //get or use default log dir
        string logDir;
        nh_private.param<std::string>("log_directory", logDir, defaultLogDir);
        init_log(logDir);
    #endif
}

Controller_ROS::~Controller_ROS()
{
    #ifdef ROS_LOG_DUMP
        // close out our loggers
        for (int i = 0; i < TOTAL_LOGS; i++)
        {
            logs_[i] << endl;
            logs_[i].close();
        }
    #endif
}

void Controller_ROS::log_controller(const double t)
{
    // logs the current input and output to the controller

    // log the input odometry from estimator
    logs_[ODOM_IN].write((char*)&t, sizeof(double));
    logs_[ODOM_IN].write((char*)parsed_odom_.data(), sizeof(double) * parsed_odom_.rows());

    // log the output command from the controller
    logs_[COMMAND_OUT].write((char*)&t, sizeof(double));
    logs_[COMMAND_OUT].write((char*)command_out_.data(), sizeof(double) * command_out_.rows());

}

void Controller_ROS::init_log(string baseLogDir)
{
    // logs_ = ofstream[TOTAL_LOGS];
    // make the log directory if it doesn't already exist
    system(("mkdir -p " + baseLogDir).c_str());

    // initialize the log files
    logs_[ODOM_IN].open(baseLogDir + "/odom.bin", std::ofstream::out | std::ofstream::trunc);
    logs_[COMMAND_OUT].open(baseLogDir + "/command.bin", std::ofstream::out | std::ofstream::trunc);
}

void Controller_ROS::odometry_callback(const nav_msgs::OdometryConstPtr &msg)
{   
    // subscribes to the 'Odom' topic of the estimator. The odometry message
    // here is the output of the estimator
    if (!odom_init_) 
    {
        //this is the first time callback was run, get the start time
        odom_init_ = true;
        start_time_ = msg->header.stamp;
        return;
    }

    // parse message
    parsed_odom_(dynamics::PX, 0) = msg->pose.pose.position.x;
    parsed_odom_(dynamics::PY, 0) = msg->pose.pose.position.y;
    parsed_odom_(dynamics::PZ, 0) = msg->pose.pose.position.z;

    parsed_odom_(dynamics::VX, 0) = msg->twist.twist.linear.x;
    parsed_odom_(dynamics::VY, 0) = msg->twist.twist.linear.y;
    parsed_odom_(dynamics::VZ, 0) = msg->twist.twist.linear.z;

    parsed_odom_(dynamics::QW, 0) = msg->pose.pose.orientation.w;
    parsed_odom_(dynamics::QX, 0) = msg->pose.pose.orientation.x;
    parsed_odom_(dynamics::QY, 0) = msg->pose.pose.orientation.y;
    parsed_odom_(dynamics::QZ, 0) = msg->pose.pose.orientation.z;

    parsed_odom_(dynamics::WX, 0) = msg->twist.twist.angular.x;
    parsed_odom_(dynamics::WY, 0) = msg->twist.twist.angular.y;
    parsed_odom_(dynamics::WZ, 0) = msg->twist.twist.angular.z;

    ros::Time ros_ts = msg->header.stamp;
    const double t = (ros_ts - start_time_).toSec();

    //compute control
    controller_.computeControl(parsed_odom_, t, command_out_);

    // publish resulting command
    command_msg_.header.stamp = ros_ts;
    command_msg_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_msg_.ignore = rosflight_msgs::Command::IGNORE_NONE;
    command_msg_.x = command_out_(dynamics::TAUX);
    command_msg_.y = command_out_(dynamics::TAUY);
    command_msg_.z = command_out_(dynamics::TAUZ);
    command_msg_.F = command_out_(dynamics::THRUST);

    command_pub_.publish(command_msg_);

    // log all this out if we're supposed to:
    #ifdef ROS_LOG_DUMP
        log_controller(t);
    #endif
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "multirotor_controller_node");
    Controller_ROS controller_ros;

    ros::spin();

    return(0);
}