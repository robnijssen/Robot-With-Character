This package contains a control structure for the UR5 robotic arm, made for the Close Encounters project in the Adaptive Robotics minor at Fontys Eindhoven.

This contains state machine structure in python files, launch files for launching them, and some handy things like a gripper check.

dependancies:
https://github.com/ros-industrial/ur_modern_driver.git
https://github.com/ros-industrial/universal_robot.git
ur5 from https://github.com/robnijssen/Close-Encounters-Robot-arm-with-Character.git
https://github.com/RoboHubEindhoven/ur3_ros_assignment_2.git <--one file needed: ur_hardware_interface.cpp

overwrite ur_hardware_interface.cpp from ur3_ros_assignment2 to ur_modern_driver/src