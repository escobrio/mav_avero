#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "mav_msgs/Actuators.h"

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "test_joint_traj_pub");
    ros::NodeHandle nh;

    // Create a publisher for the /command/motor_speed topic
    ros::Publisher motorSpeedPub = nh.advertise<mav_msgs::Actuators>("/command/motor_speed", 10);

    // Create a message object
    mav_msgs::Actuators motorSpeedMsg;
    

    // Set the initial motor speed value
    double initialSpeed = 0.0;
    std::vector<double> angles(9, 0.0);
    std::vector<double> speeds(3, 0.0);

    // Set the rate at which the motor speed increases (in rad/s)
    double increase_rate = 2;
    double publish_freq = 10;

    angles[3] = initialSpeed;
    angles[4] = initialSpeed;

    // Setzen Sie die Winkel und Geschwindigkeiten in der Nachricht
    motorSpeedMsg.angles = angles;
    motorSpeedMsg.angular_velocities = speeds;

    // Publish the message
    motorSpeedPub.publish(motorSpeedMsg);

    // Sleep for a certain time
    ros::Duration sleepDuration(1/publish_freq); // Sleep for 2 seconds
    sleepDuration.sleep();


    // Publish the message at the specified rate
    ros::Rate loopRate(publish_freq);
    while (ros::ok()) {
        // Increase the motor speed value
        initialSpeed += increase_rate / publish_freq;
        angles[3] = initialSpeed;
        angles[4] = initialSpeed;

        // Setzen Sie die Winkel und Geschwindigkeiten in der Nachricht
        motorSpeedMsg.angles = angles;
        motorSpeedMsg.angular_velocities = speeds;

        // Publish the message
        motorSpeedPub.publish(motorSpeedMsg);

        // Sleep for the specified time interval
        loopRate.sleep();
    }

    return 0;
}