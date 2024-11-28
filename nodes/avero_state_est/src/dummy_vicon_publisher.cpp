#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_vicon_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("vrpn_client/estimated_transform", 10);

    ros::Rate rate(10); // Publish at 10 Hz

    geometry_msgs::TransformStamped transformStamped;
    while (ros::ok())
    {
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "robot";

        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        pub.publish(transformStamped);
        ROS_INFO("dummy publisher is running");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}