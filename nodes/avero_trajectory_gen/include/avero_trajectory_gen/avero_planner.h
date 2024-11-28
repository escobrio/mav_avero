#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/io.h>

class AveroPlanner {
 public:
  AveroPlanner(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void joyCallback(const sensor_msgs::Joy joy_msg);

  void setMaxSpeed(double max_v);

  bool plan6DTrajectory(std::string trajectory_vertices_file, mav_trajectory_generation::Trajectory* trajectory);

  // Plans a trajectory through all the vertices in the Vector
  bool plan3DTrajectory(const mav_trajectory_generation::Vertex::Vector vertices,
                      mav_trajectory_generation::Trajectory* trajectory,
                      std::string pos_or_rot);

  // Plans a trajectory to take off from the current position and fly to the goal position
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);

  // Plans a trajectory from a start position to a end position                    
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_markers_, pub_trajectory_;
  ros::Subscriber sub_odom_, sub_joy_;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;

  std::string trajectories_folder_;
};

std::vector<mav_trajectory_generation::Vertex> getPositionVerticesYaml(const std::string& filename);
std::vector<mav_trajectory_generation::Vertex> getRotationVerticesYaml(const std::string& filename);


#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
