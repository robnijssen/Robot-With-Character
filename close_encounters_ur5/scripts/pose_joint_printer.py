#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs.msg

"""
This program will print the pose of the end effector and the joint values each time you press enter.
"""

# start a new node
rospy.init_node('pose_joint_printer', anonymous=False)
rospy.loginfo("pose/joint printer starting")

# start moveit
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("manipulator")

while not rospy.is_shutdown():
    # wait for enter
    print("Press ENTER to print the pose and joint values once.")
    raw_input()

    # print current position
    print("Current end effector pose:")
    print(group.get_current_pose())

    # print current joint position
    print("Current joint values:")
    print(group.get_current_joint_values())
