This package contains a control structure for the UR5 robotic arm, made for the Close Encounters project in the Adaptive Robotics minor at Fontys Eindhoven.

This contains state machine structure in python files, launch files for launching them, and some handy things like a gripper check.

dependancies:
https://github.com/ros-industrial/ur_modern_driver.git
https://github.com/ros-industrial/universal_robot.git
ur5 from either
    https://github.com/robnijssen/Close-Encounters-Robot-arm-with-Character.git
    https://github.com/robnijssen/Robot-With-Character.git

Overwrite the files ur_hardware_interface.cpp and ur_ros_wrapper.cpp from this folder to ur_modern_driver/src



Setting up the ethernet connection:
Plug in a cable from the control box to the computer.
If you haven't already:
    Go to network settings and edit the connection that just appeared. 
    Go to IPv4 Settings, 
        Address: 
            Look at what the robot's ip is.
            Your ip should have the first two parts the same as the bot and the last one different than the robot, for example: bot's ip is 192.168.66.3, enter 192.168.66.4 as your own.
        Netmask: 24
        Gateway: 255.255.255.0






Running:

Running a simulation can be done by launching:
roslaunch close_encounters_ur5 demo_all.launch

Running the program on the real arm can be done by setting up the ethernet connection and launching:
roslaunch close_encounters_ur5 run_all.launch

To find new joint values to use, launch:
roslaunch close_encounters_ur5 arm_drivers.launch
Then move the arm and run:
rosrun close_encounters_ur5 pose_joint_printer.py