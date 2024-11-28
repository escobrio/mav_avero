// #include <mav_trajectory_generation_example/example_planner.h>
#include "avero_trajectory_gen/avero_planner.h"
#include <fstream>
#include <yaml-cpp/yaml.h>


AveroPlanner::AveroPlanner(ros::NodeHandle& nh)
    : nh_(nh),
      max_v_(2.0),
      max_a_(2.0),
      max_ang_v_(2.0),
      max_ang_a_(2.0),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_angular_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {

  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_v", max_ang_v_)){
    ROS_WARN("[example_planner] param max_ang_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_a", max_ang_a_)){
    ROS_WARN("[example_planner] param max_ang_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/trajectories_folder", trajectories_folder_)) {
    ROS_ERROR("Failed to get param 'trajectories_folder'");
  }

  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory", 0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &AveroPlanner::uavOdomCallback, this);
  
  sub_joy_ = 
      nh.subscribe("joy", 1, &AveroPlanner::joyCallback, this);
}

// Callback to get current Pose of UAV
void AveroPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current velocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
  tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
}

void AveroPlanner::joyCallback(const sensor_msgs::Joy joy_msg) {
  // Button 7 on the Logitech Extreme 3DPro is actually joy_msg.buttons[6]
  int buttons_first = 0;
  int buttons_last = joy_msg.buttons.size();

  for (int i = buttons_first; i < buttons_last; i++) {
    if (joy_msg.buttons[i]){
      std::string filename = trajectories_folder_ + "trajectory_btn" + std::to_string(i+1) + ".yaml";
      std::ifstream file(filename);
      if (!file.good()) {
        ROS_WARN_STREAM("[AveroPlanner::joyCallback] Could not find " << filename);
        return;
      }
      mav_trajectory_generation::Trajectory trajectory;
      if (i < 6) {
        ROS_WARN("[AveroPlanner::joyCallback] Executing polynomial segments defined in trajectory_btn%d.yaml", i+1);
        mav_trajectory_generation::trajectoryFromFile(filename, &trajectory);
      } else if (i >= 6) {
        ROS_WARN("[AveroPlanner::joyCallback] Executing vertices defined in trajectory_btn%d.yaml", i+1);
        plan6DTrajectory(filename, &trajectory);
      }
      publishTrajectory(trajectory);
    }
  }
}

// Method to set maximum speed.
void AveroPlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

bool AveroPlanner::plan6DTrajectory(std::string trajectory_vertices_file, mav_trajectory_generation::Trajectory* trajectory) {
  
  mav_trajectory_generation::Vertex::Vector vertices_trans = getPositionVerticesYaml(trajectory_vertices_file);
  mav_trajectory_generation::Vertex::Vector vertices_rot = getRotationVerticesYaml(trajectory_vertices_file);


  mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;
  bool success = false;
  // Translation trajectory.
  // Only take the first three entries of vertices
  ROS_INFO_STREAM("vertices_trans: \n" << vertices_trans);
  success = plan3DTrajectory(vertices_trans, &trajectory_trans, "pos");

  // Rotation trajectory.
  ROS_INFO_STREAM("vertices_rot: \n" << vertices_rot);
  success = plan3DTrajectory(vertices_rot, &trajectory_rot, "rot");

  // Combine trajectories.
  success &= trajectory_trans.getTrajectoryWithAppendedDimension(trajectory_rot, &(*trajectory));

  return success;
}

bool AveroPlanner::plan3DTrajectory(
    const mav_trajectory_generation::Vertex::Vector vertices,
    mav_trajectory_generation::Trajectory* trajectory, std::string pos_or_rot) {
  assert(trajectory);
  // trajectory->clear();



  const int dimension = 3;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // estimate initial segment times
  std::vector<double> segment_times;
  if (pos_or_rot == "rot") {
    segment_times = estimateSegmentTimes(vertices, max_ang_v_, max_ang_a_);
  } else {
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
  }

  // Set a minimum segment time to not get the segment_time = 0 error
  double minimum_segment_time = 0.001;
  for (int i = 0; i < segment_times.size(); ++i) {
    if (segment_times[i] < minimum_segment_time) {
      segment_times[i] = minimum_segment_time;
    }
    ROS_INFO_STREAM("segment_time " << i << ": " << segment_times[i]);
  }

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  if (pos_or_rot == "rot") {
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_ang_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_ang_a_);
  } else {
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  }

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  if (pos_or_rot == "rot") {
    trajectory->scaleSegmentTimesToMeetConstraints(max_ang_v_, max_ang_a_);
  } else {
    trajectory->scaleSegmentTimesToMeetConstraints(max_v_, max_a_);
  }
  
  return true;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool AveroPlanner::planTrajectory(
    const Eigen::VectorXd& goal_pos, const Eigen::VectorXd& goal_vel,
    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 6 Dimensional trajectory => through SE(3) space, position and orientation
  const int dimension = goal_pos.size();
  bool success = false;

  if (dimension == 6) 
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;

    // Translation trajectory.
    Eigen::Vector3d goal_position = goal_pos.head(3);
    Eigen::Vector3d goal_lin_vel = goal_vel.head(3);
    success = planTrajectory(
        goal_position, goal_lin_vel, current_pose_.translation(),
        current_velocity_, max_v_, max_a_, &trajectory_trans);

    // Rotation trajectory.
    Eigen::Vector3d goal_rotation = goal_pos.tail(3);
    Eigen::Vector3d goal_ang_vel = goal_vel.tail(3);
    Eigen::Vector3d current_rot_vec;
    mav_msgs::vectorFromRotationMatrix(
        current_pose_.rotation(), &current_rot_vec);
    success &= planTrajectory(
        goal_rotation, goal_ang_vel, current_rot_vec, current_angular_velocity_,
        max_ang_v_, max_ang_a_, &trajectory_rot);

    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(
            trajectory_rot, &(*trajectory));
    return success;
  } 
  else if (dimension == 3) 
  {
    success = planTrajectory(
        goal_pos, goal_vel, current_pose_.translation(), current_velocity_,
        max_v_, max_a_, &(*trajectory));
    return success;
  } 
  else if (dimension == 4) 
  {
    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;
    success = planTrajectory(
        goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
        &(*trajectory));
    return success;
  } 
  else 
  {
    LOG(WARNING) << "Dimension must be 3, 4 or 6 to be valid.";
    return false;
  }
}

// bool AveroPlanner::planTrajectory(const mav_trajectory_generation::Vertex::Vector vertices, mav_trajectory_generation::Trajectory* trajectory) {
//   std::cerr << "TO DO\n";
// }

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool AveroPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = goal_pos.size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);
vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  
  return true;
}
                                    

bool AveroPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "map";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  double marker_lifetime = 30;
  for (int i = 0; i < markers.markers.size(); i++) {
    markers.markers[i].lifetime = ros::Duration(marker_lifetime);
  }
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "map";
  pub_trajectory_.publish(msg);

  return true;
}

std::vector<mav_trajectory_generation::Vertex> getPositionVerticesYaml(const std::string& filename) {
    std::vector<mav_trajectory_generation::Vertex> vertices_position;
    YAML::Node config = YAML::LoadFile(filename);
    ROS_INFO_STREAM("[AveroPlanner::getPositionVerticesYaml] Successfully loaded trajectory yaml file: " + filename);
    if (config["dimensions"].as<int>() == 6){
      int index = 0;
      for (const auto& v : config["vertices"]) {
        mav_trajectory_generation::Vertex vertex(3);
        Eigen::Vector3d position;
        for (int i = 0; i < 3; ++i) {
          position[i] = v["vertex"]["position"][i].as<double>();
        }
        if (index == 0) {
          // Make first vertex a Starting vertex
          vertex.makeStartOrEnd(position, mav_trajectory_generation::derivative_order::SNAP);
        } else {
          vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, position);
        }
        vertices_position.push_back(vertex);
        index++;
      }
      // Make last vertex a End vertex
      Eigen::VectorXd last_position; 
      vertices_position[index - 1].getConstraint(0, &last_position);
      vertices_position[index - 1].makeStartOrEnd(last_position, 4);
    } 
    else {
      ROS_ERROR_STREAM("TODO: Non-6 dimensional vertices");
    }
    return vertices_position;
}

// Could make the code more reusable, this is pretty much the same function as getPositionVerticesYaml
std::vector<mav_trajectory_generation::Vertex> getRotationVerticesYaml(const std::string& filename) {
    std::vector<mav_trajectory_generation::Vertex> vertices_rotation;
    YAML::Node config = YAML::LoadFile(filename);
    if (config["dimensions"].as<int>() == 6){
      int index = 0;
      for (const auto& v : config["vertices"]) {
        Eigen::Vector3d rotation_vec;
        Eigen::Matrix3d rotation_mat;
        mav_trajectory_generation::Vertex vertex(3);

        double roll = v["vertex"]["orientation"][0].as<double>();
        double pitch = v["vertex"]["orientation"][1].as<double>();
        double yaw = v["vertex"]["orientation"][2].as<double>();
        
        rotation_mat = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()) 
                     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());
        mav_msgs::vectorFromRotationMatrix(rotation_mat, &rotation_vec);

        if (index == 0) {
          // Make first vertex a Starting vertex
          vertex.makeStartOrEnd(rotation_vec, mav_trajectory_generation::derivative_order::SNAP);
        } else {
          vertex.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION, rotation_vec);
        }
        vertices_rotation.push_back(vertex);
        index++;
      } 
      // Make last vertex a End vertex
      Eigen::VectorXd last_rotation; 
      vertices_rotation[index - 1].getConstraint(0, &last_rotation);
      vertices_rotation[index - 1].makeStartOrEnd(last_rotation, 4);
    }
    else {
      ROS_ERROR_STREAM("TODO: Non-6 dimensional vertices");
    }    
  return vertices_rotation;
}