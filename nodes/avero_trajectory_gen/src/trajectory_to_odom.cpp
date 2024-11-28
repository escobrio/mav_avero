// Copied from avero_joy/src/joy_to_odom.cpp
// TODO: clean up

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// Delete:
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>

//rotations given by the joystick are allways arround the axes in the worls frame
//could be changed in line 81

class TrajectoryToOdom {
public:
    TrajectoryToOdom() {
      nh_ = ros::NodeHandle("~");
      trajectory_sub_ = nh_.subscribe("/command/trajectory", 1, &TrajectoryToOdom::trajectoryCallback, this);
      odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/desired_drone_odometry", 1);
      current_time = ros::Time::now();      
    }

    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory& joy_msg) {
        nav_msgs::Odometry odom_msg;
        last_time = current_time;
        current_time = ros::Time::now();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "map";

        // Position and Orientation
        odom_msg.pose.pose.position.x = joy_msg.points[0].transforms[0].translation.x;
        odom_msg.pose.pose.position.y = joy_msg.points[0].transforms[0].translation.y;
        odom_msg.pose.pose.position.z = joy_msg.points[0].transforms[0].translation.z;

        odom_msg.pose.pose.orientation.w = joy_msg.points[0].transforms[0].rotation.w;
        odom_msg.pose.pose.orientation.x = joy_msg.points[0].transforms[0].rotation.x;
        odom_msg.pose.pose.orientation.y = joy_msg.points[0].transforms[0].rotation.y;
        odom_msg.pose.pose.orientation.z = joy_msg.points[0].transforms[0].rotation.z;

        // Linear and angular Velocity
        // Command 0 velocity because the real life controller is only tuned to follow position
        odom_msg.twist.twist.linear.x = 0;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;

        odom_msg.twist.twist.angular.x = 0;
        odom_msg.twist.twist.angular.y = 0;
        odom_msg.twist.twist.angular.z = 0;

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
    ros::Subscriber trajectory_sub_;
    ros::Publisher odometry_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_to_odom_converter");
    TrajectoryToOdom converter;
    ros::spin();

    return 0;
}
