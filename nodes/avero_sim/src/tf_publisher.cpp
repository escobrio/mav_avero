#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> // Add this line

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) // Change message type here
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  // Set the translation from map to BASE
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)); // Update message structure here
  
  // Set the rotation from map to BASE
  tf::Quaternion q;
  q.setX(msg->pose.pose.orientation.x); // Update message structure here
  q.setY(msg->pose.pose.orientation.y); // Update message structure here
  q.setZ(msg->pose.pose.orientation.z); // Update message structure here
  q.setW(msg->pose.pose.orientation.w); // Update message structure here
  transform.setRotation(q);
  
  // Publish the transform from map to BASE
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/pose_base_link", 10, poseCallback); // Change message type here
  
  ros::spin();
  
  return 0;
}