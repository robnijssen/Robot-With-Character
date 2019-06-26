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

  
## Package content
This package is made for adding a base to the UR5 arm and to put a gripper on the end-effector of the UR5. Some extra objects, like a camera, are added to the visualization as well.
For the visuals of the UR5, the original files of universal_robot are used. Every file that had to be changed, is in this UR5 package. 
For the visuals and the controller of the gripper, they are added to this package. The same for the visuals of the camera.
The base_link of the UR5 is in the exact same place as world. This is because of the method of recording movements in the close_encounters_ur5 package.

## Gazebo
The complete robot arm is implemented in Gazebo. This means that the seperate parts have inertia and that the controllers are working correctly. The controllers for the arm and for the gripper can be used seperately, with rostopic pub or with making a python script.
The issue with the current implementation in Gazebo is the ground_plane. The base_link of the robot arm is in the exact same spot as world. The top plate of the base that is currently used is slightly lower then base_link. This means that the top plate of the base is underneath the ground_plane. In Gazebo, bodies are not able to move through the ground_plane. This might give problems when trying to pick up object that are not elevated.


