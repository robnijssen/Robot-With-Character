# Close encountes: a robot with character
This package contains a control structure for the UR5 robotic arm, made for the Close Encounters project in the Adaptive Robotics minor at Fontys Eindhoven 2018/2019.

This package contains the  state machine structure in python files, launch files for launching them, and some handy things like a gripper check and an file creator to record new animations. 

## Dependencies
` cd ~/<your catkin workspace>/src/ ` 

` git clone <links below> ` 

https://github.com/ros-industrial/ur_modern_driver.git

https://github.com/ros-industrial/universal_robot.git https://github.com/ros-industrial/robotiq.git 

https://github.com/robnijssen/Robot-With-Character.git

Make sure to overwrite the files ur_hardware_interface.cpp and ur_ros_wrapper.cpp from this folder to ur_modern_driver/src


## Installing the package 

` cd ~/<your catkin workspace>/src/ ` 

` git clone https://github.com/robnijssen/Robot-With-Character.git ` 

In order to be able to run the scripts in the package, run the following command, this will make all .py files executable:

` cd ~/<your catkin workspace>/src/Robot-With-Character/close_encounters_ur5/scripts

` chmod +x *py ` 

Finally, compile your workspace: 

` cd ~/<your catkin workspace>/ ` 

` catkin_make ` 


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

## Record and play back movements

If you want to implement more of your own movements, the easiest way is to create a new sequence of pose goals and play it back. After testing the multiple options MOVEIT gives you (joints states, pose goals and cartesian paths), we found out you get the most smooth motion if you plan and execute a cartesian path.

We created a program the eases the process of creating new movements. What this program does is save the current joints states and end effector pose (x, y, z, rx, ry, rz, w). By free jogging the arm and pressing ENTER you can create a new file that contains your recorded sequence. 

If you want more information about this filetype and how to use it in python, please take a look at this link. 

https://docs.python.org/3/library/configparser.html 

### How to record a new animation
To create a new .ini fil
` roslaunch close_encounters_ur5 arm_drivers.launch ` 
e containing an animation with either joint values or pose values, launch: 

After that, run:

` rosrun close_encounters_ur5 joint_angles_to_ini.py `

The program will guide you trough the process. Make sure the actual robot state shows up in rviz. If that is not the case, try to reconnect to the robot and make sure you follow the set-up.

### How to play back a  recorded animation

Run the movement test node with:

` rosrun close_encounters_ur5 movement_test_node.py `

 The movement_test_node will ask you to provide the path to the .ini file you want to play back. Make sure you provide the right path, otherwise the program will not work. 

In our case:
` /home/ubuntu/catkin_ws/src/Robot-With-Character/close_encounters_ur5/inifiles ` 

The program will ask if you want to use either joint states, pose goals or a cartesian path. It will also ask how many keypoints you want to play back and which section name you want to use. Make sure to fill in the amount of keypoints you want to test. 

If you want to use a cartesian path or pose goal, fill in: Pose_Goals for the section name. For joint states, use the section name: Joint_States. 







