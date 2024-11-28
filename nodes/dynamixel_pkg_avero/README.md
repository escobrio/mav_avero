## How to execute the nodes


**Important:** Before you turn on the dynamixels, make sure they are aligned with the markings on the nozzle!!!
Make sure you have the Python packages "sympy" and "time" installed!


1. Open a terminal (do not start roscore).
2. Launch the dynamixel_pkg: will most definitly not work (still needs to be set up)

    ```bash
    roslaunch dynamixel_pkg_avero dynamic_launch.launch
    ```
2.2 If the launch file didn't work you need to start the 3 nodes by yourself:
    Each in a new terminal.
    
```bash
roscore
```

```bash
rosrun dynamixel_pkg_avero inv_kin_new_angles.py
```

```bash
rosrun dynamixel_pkg_avero dtc_distributor.py 
```


If you have not already, plug in the Nozzle.
```bash
rosrun dynamixel_pkg_avero read_write_node.py
```


3. Open a new terminal.
4. Publish a message to the `/chi_des_endeffector` topic. Replace the values in the `data` field with your input vector:
   Even tough the topic is still named Chi, you need to publish your desired normalvector in cartesian coordinates in the Inertial frame (still need to rename that)

    ```bash
    rostopic pub -1 /chi_des_endeffector std_msgs/Float32MultiArray "layout:
      dim:
      - label: ''
        size: 0
        stride: 0
      data_offset: 0
    data: [0.0, 1.0, 1.0]"
    ```

**Note:** The inverse_kinematics node is not debugged yet.

## How to test and optimize the inverse kinematics node

The File src/nodes/in_kin_new_angles/main_in_kin_sym_test.py is the PPyrhon file used for testing the inverse Kinematics and also optimizing it. You can Execute it directly in your IDE

What still needs tuning ?
Mostly the runntime needs to get better (the Time which is needed to execute the while Loop) i think we can tune these Params, and also the code.
But for the Parameters we could change the **alpha **,  and also the **acceptable_chi_err** - if you change the Python testing file, make sure that you also make the samge changes in the Ros node in src/nodes/in_kin_new_angles/inv_kin_new_angles.py.
