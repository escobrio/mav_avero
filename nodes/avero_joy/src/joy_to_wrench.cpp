#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h> //For Python allocator
#include <eigen3/Eigen/Dense>           //For C++ allocator in future 
#include <eigen3/Eigen/Geometry>

class JoyToWrenchConverter {
public:
    JoyToWrenchConverter() {
      nh_ = ros::NodeHandle("~");
      joy_sub_ = nh_.subscribe("/joy", 1, &JoyToWrenchConverter::joyCallback, this);
      wrench_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/des_wrench", 1);
      current_time = ros::Time::now();
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        std_msgs::Float64MultiArray wrench_msg;
        last_time = current_time;
        current_time = ros::Time::now();
        // wrench_msg.header.stamp = current_time;
        dt = (current_time - last_time).toSec();
        // wrench_msg.header.stamp = current_time;
        // wrench_msg.header.frame_id = "map";
        wrench_msg.data = {-joy_scaler*joy_msg->axes[0], joy_scaler*joy_msg->axes[1], joy_scaler*joy_msg->axes[3], joy_scaler*joy_msg->axes[5], -joy_scaler*joy_msg->axes[4], joy_scaler*joy_msg->axes[2]};
        wrench_pub_.publish(wrench_msg);
    }

    double dt;
    double joy_scaler = 5;
    ros::Time current_time;
    ros::Time last_time;

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher wrench_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_wrench_converter");
    JoyToWrenchConverter converter;
    ros::spin();

    return 0;
}
