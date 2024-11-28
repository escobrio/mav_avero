# How to use this Repository 
See the wiki.

Then feel free to add your packages to the Avero_repos so everybody of the team has access to them.

## Packages in Nodes directory:
- **avero_ctrl** contains the main node which will run on the onboard computer. It is built with the omav_hovery package from ethz_asl and adapted to our hardware (3 Propellers + 6 Dynamixel Motors). It also contains the controller and allocator implementations.
- **avero_joy** is a package to get position and orientation input from a joystick
- **avero_sim** is a package to visualize and simulate the drone with rviz and gazebo.
- **avero_testing** contains programs to test the combination of hardware and software
- **avero_trajectory_gen** produces polynomial trajectories
- **dynamixel_pkg_avero** outputs a single Dynamixel motor's movement to a direction vector input using inverse kinematics.
- **imu_to_nozzle** points the swivel nozzle towards the ground according to the orientation given by an IMU and inverse kinematics.

## ASL_repos
- For more information on repos we copied from ASL search them in the ethz_asl github
