#include  "ros/ros.h"
#include "avero_trajectory_gen/avero_planner.h"
#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle nh;
  AveroPlanner planner(nh);

  ros::spin();
  return 0;
}