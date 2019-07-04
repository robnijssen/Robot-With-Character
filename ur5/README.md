# UR5 close_encounters

## UR5 dependencies
git clone <links below>
https://github.com/ros-industrial/universal_robot.git
  
## Rviz
To launch the UR5 and the other bodies in the urdf files launch:

` roslaunch ur5 demo.launch limited:=true`

To launch the UR5 and the other bodies in Rviz with a gui to move the joints manually launch:

` roslaunch ur5 demo.launch limited:=true use_gui:=true`

## Gazebo
To launch the UR5 and the other bodies in the urdf launch:

`  roslaunch ur5 ur5.launch limited:=true`

However, there is a problem with using Gazebo. Because the base is underneath the ground_plane, Gazebo does not load the bodies in correctly. The controllers are loaded in just fine.

