# Robot-With-Character
This package contains a control structure for the UR5 robotic arm, made for the Close Encounters project in the Adaptive Robotics minor at Fontys Eindhoven.

This contains state machine structure in python files, launch files for launching them, and some handy things like a gripper check and an file creator to record new animations. 

## dependancies:

https://github.com/ros-industrial/ur_modern_driver.git

https://github.com/ros-industrial/universal_robot.git https://github.com/ros-industrial/robotiq.git 

ur5 from either 
https://github.com/robnijssen/Close-Encounters-Robot-arm-with-Character.git
or
https://github.com/robnijssen/Robot-With-Character.git

Make sure to overwrite the files ur_hardware_interface.cpp and ur_ros_wrapper.cpp from this folder to ur_modern_driver/src

## Setting up the ethernet connection:

Plug in a cable from the control box to the computer. If you haven't already: Go to network settings and edit the wired connection that just appeared. Go to IPv4 Settings. Change the method to manual.
Then go to the Addressess box and make these changes:

Adress: Look at what the robot's ip is. Your ip should have the first two parts the same as the bot and the last one different than the robot, for example: bot's ip is 192.168.66.3, enter 192.168.66.4 as your own.

Netmask: 24 

Gateway: 255.255.255.0

## Running this package:

Running a simulation can be done by launching:

` roslaunch close_encounters_ur5 demo_all.launch `

Running the program on the real arm can be done by setting up the ethernet connection and launching:

` roslaunch close_encounters_ur5 run_all.launch ` 

In case run_all.launch doesn't work, running the individual launch files could help:

` roslaunch ur5 ur5_bringup.launch robot_ip:=192.168.66.3 ` 

` roslaunch ur5 ur5_moveit_planning_execution.launch limited:=true robot_ip:=192.168.66.3 ` 

` roslaunch ur5 moveit_rviz.launch config:=true robot_ip:=192.168.66.3 ` 

To create a new .ini file containing an animation with either joint values or pose values, launch: 

` roslaunch close_encounters_ur5 arm_drivers.launch Then move the arm and run: rosrun close_encounters_ur5 pose_joint_printer.py `

To play back this recorded animation, run the movement test node:

` rosrun close_encounters_ur5 movement_test_node.py `

You can either to this with joint goals or by making a cartesian path. Make sure you enter the right path to the map containing the .ini files. 

In our case: ` /home/ubuntu/catkin_ws/src/Robot-With-Character/close_encounters_ur5/inifiles ` 




