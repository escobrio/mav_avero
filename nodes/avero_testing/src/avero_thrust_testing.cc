#include "../include/avero_thrust_testing.h"
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "avero_thrust_testing_node");

  AveroThrustTestingNode node{};
  node.start();
  node.thrustTesting();
  node.stop();
}