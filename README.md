# Homework2_rl2024
Homework 3 for Robotics Lab 2024/2025

First build all the packages by using:

```
colcon build --packages-select aruco aruco_msgs aruco_ros iiwa_description ros2_kdl_package iiwa_bringup ros2_opencv
```
In each terminal you open, source the install directory:
```
source install/setup.bash
```
# 1. Start Gazebo and spawn the robot with the camera into the new world containing the spherical_object
To launch `iiwa.launch.py` run:
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true"
```
To subscribe to the simulated image, detect the spherical object in it and republish the processed image, run:
```
ros2 run ros2_opencv ros2_opencv_node
```

# 2ai. Task:="positioning"
To launch `iiwa_aruco.launch.py` using as command interface "velocity" and as controller "velocity_controller", run:
```
ros2 launch iiwa_bringup iiwa_aruco.launch.py use_sim:="true" command_interface:="velocity" robot_controller:="velocity_controller" 
```
In another terminal, start the ArUco marker detection node:
```
ros2 launch aruco_ros aruco_cam.launch.py
```
Run rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view 
```
In another terminal launch the vision-based control node:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="velocity" -p task:="positioning" 
```

# 2aii. Task:="look-at-point" and cmd_interface:="velocity"
To launch `iiwa_aruco.launch.py` using as command interface "velocity" and as controller "velocity_controller", run:
```
ros2 launch iiwa_bringup iiwa_aruco.launch.py use_sim:="true" command_interface:="velocity" robot_controller:="velocity_controller" 
```
In another terminal, start the ArUco marker detection node:
```
ros2 launch aruco_ros aruco_cam.launch.py
```
Run rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view 
```
In another terminal launch the vision-based control node:
```
ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args -p cmd_interface:="velocity" -p task:="look-at-point" 
```
# 2b.  Vision-Based Control with Effort Interface

To launch `iiwa_aruco.launch.py` using as command interface "effort" and as controller "effort_controller", run:
```
ros2 launch iiwa_bringup iiwa_aruco.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"  
```

In another terminal, start the ArUco marker detection node:
```
ros2 launch aruco_ros aruco_cam.launch.py
```
Run rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view 
```
To execute the task using time_law:="cubic" and cmd_interface:="effort", run:
```
ros2 run ros2_kdl_package effort_vision_control --ros-args -p cmd_interface:="effort" -p time_law:="cubic" -p task:="look-at-point" 
```
To execute the task using time_law:="trapezoidal" and cmd_interface:="effort", run:
```
ros2 run ros2_kdl_package effort_vision_control --ros-args -p cmd_interface:="effort" -p time_law:="trapezoidal" -p task:="look-at-point" 
```
To execute the task using time_law:="cubic" and cmd_interface:="effort_cartesian", run:
```
ros2 run ros2_kdl_package effort_vision_control --ros-args -p cmd_interface:="effort_cartesian" -p time_law:="cubic" -p task:="look-at-point"
```
To execute the task using time_law:="trapezoidal" and cmd_interface:="effort_cartesian", run:
```
ros2_kdl_package effort_vision_control --ros-args -p cmd_interface:="effort_cartesian" -p time_law:="trapezoidal" -p task:="look-at-point" 
```

 NOTES:
 After running the command provided above, as soon as Gazebo opens, PRESS THE PLAY BUTTON in the lower left corner.This will ensure proper loading and valid activation of the controllers!!!


 
