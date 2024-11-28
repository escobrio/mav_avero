#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>

//rotations given by the joystick are allways arround the axes in the worls frame
//could be changed in line 81

class JoyToOdometryConverter {
public:
    JoyToOdometryConverter() {
      nh_ = ros::NodeHandle("~");
      joy_sub_ = nh_.subscribe("/joy", 1, &JoyToOdometryConverter::joyCallback, this);
      odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/desired_drone_odometry", 1);
      current_time = ros::Time::now();

      des_total_rot.setIdentity();

      bool all_params_initialised;
      all_params_initialised = true;
      all_params_initialised = all_params_initialised && nh_.getParam("/joystick_params/vel_xy", vel_xy_scaler);
      //std::cout << all_params_initialised << std::endl;
      all_params_initialised = all_params_initialised && nh_.getParam("/joystick_params/vel_z", vel_z_scaler);
      //std::cout << all_params_initialised << std::endl;
      all_params_initialised = all_params_initialised && nh_.getParam("/joystick_params/velrot_xy", velrot_xy_scaler);
      //std::cout << all_params_initialised << std::endl;
      all_params_initialised = all_params_initialised && nh_.getParam("/joystick_params/velrot_z", velrot_z_scaler);
      //std::cout << all_params_initialised << std::endl;
      all_params_initialised = all_params_initialised && nh_.getParam("/joystick_params/enable_velocity_reference", enable_velocity_reference);
        
        if (!all_params_initialised) {
            ROS_ERROR("Not all joystick parameters were set. Please set all parameters in the launch file.");
            ros::shutdown();
        }
      
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        nav_msgs::Odometry odom_msg;
        last_time = current_time;
        current_time = ros::Time::now();
        odom_msg.header.stamp = current_time;
        dt = (current_time - last_time).toSec();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "map";

        //Linear Velocity and position
        //x-axis
        odom_msg.twist.twist.linear.x = -vel_xy_scaler * joy_msg->axes[0];
        des_odom_pos_x += -vel_xy_scaler * joy_msg->axes[0] * dt;
        odom_msg.pose.pose.position.x += des_odom_pos_x;

        //y-axis
        odom_msg.twist.twist.linear.y = vel_xy_scaler * joy_msg->axes[1];
        des_odom_pos_y += vel_xy_scaler * joy_msg->axes[1] * dt;
        odom_msg.pose.pose.position.y += des_odom_pos_y;

        //z-axis
        odom_msg.twist.twist.linear.z = vel_z_scaler * joy_msg->axes[3];
        des_odom_pos_z += vel_z_scaler * joy_msg->axes[3] * dt;
        odom_msg.pose.pose.position.z += des_odom_pos_z;

        //rotational velocity
        //x-axis
        odom_msg.twist.twist.angular.x = -velrot_xy_scaler * joy_msg->axes[5];
        angle_x = -velrot_xy_scaler * joy_msg->axes[5] * dt;
        Eigen::Quaterniond rot_x(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX()));

        //y-axis
        odom_msg.twist.twist.angular.y = -velrot_xy_scaler * joy_msg->axes[4];
        angle_y = -velrot_xy_scaler * joy_msg->axes[4] * dt;
        Eigen::Quaterniond rot_y(Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY()));

        //z-axis
        odom_msg.twist.twist.angular.z = velrot_z_scaler * joy_msg->axes[2];
        angle_z = velrot_z_scaler * joy_msg->axes[2] * dt;
        Eigen::Quaterniond rot_z(Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));

        //if rotations given by the joystick should be in body frabe do:
        //des_total_rot = des_total_rot * rot_x * rot_y * rot_z;
        des_total_rot = rot_x * rot_y * rot_z * des_total_rot;
        odom_msg.pose.pose.orientation.x = des_total_rot.x();
        odom_msg.pose.pose.orientation.y = des_total_rot.y();
        odom_msg.pose.pose.orientation.z = des_total_rot.z();
        odom_msg.pose.pose.orientation.w = des_total_rot.w();

        //disable velocity reference if desired
        if (!enable_velocity_reference) {
            odom_msg.twist.twist.linear.x = 0;
            odom_msg.twist.twist.linear.y = 0;
            odom_msg.twist.twist.linear.z = 0;
            odom_msg.twist.twist.angular.x = 0;
            odom_msg.twist.twist.angular.y = 0;
            odom_msg.twist.twist.angular.z = 0;
        }
        odometry_pub_.publish(odom_msg);
    }

    double vel_xy_scaler;
    double vel_z_scaler;
    double velrot_xy_scaler;
    double velrot_z_scaler;
    double dt;
    double des_odom_pos_x = 0;
    double des_odom_pos_y = 0;
    double des_odom_pos_z = 0.5;
    double angle_x;
    double angle_y;
    double angle_z;
    bool enable_velocity_reference;
    Eigen::Quaterniond des_total_rot;
    ros::Time current_time;
    ros::Time last_time;

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher odometry_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_odometry_converter");
    JoyToOdometryConverter converter;
    ros::spin();

    return 0;
}
