#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs.msg
import ConfigParser

Config = ConfigParser.ConfigParser()

"""
This program will print the pose of the end effector and the joint values each time you press enter.
"""

# start a new node
rospy.init_node('pose_joint_printer', anonymous=False)
rospy.loginfo("pose/joint printer starting")

# start moveit
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("manipulator")
file_name = raw_input('How do you want to name the file. Name: ')

cfgfile = open("/home/ubuntu/movements/" + file_name + ".ini",'w')
Config.add_section('Joint_States')
Config.add_section('Pose_Goals')
Config.add_section('Amount_Of_Keypoints')

index = 0

while not rospy.is_shutdown():

    # wait for enter
    raw_input("Press ENTER to print the pose and joint values once.")

    # print current position
    pose = group.get_current_pose().pose
    joint_values = group.get_current_joint_values()
    joints = str(joint_values[0]) + " " + str(joint_values[1]) + " " + str(joint_values[2]) + " " + str(joint_values[3]) + " " + str(joint_values[4]) + " " + str(joint_values[5])
    waypoint = str(pose.position.x) + " " + str(pose.position.y) + " " + str(pose.position.z) + " " + str(pose.orientation.x) + " " + str(pose.orientation.y) + " " + str(pose.orientation.z) + " " + str(pose.orientation.w) 
    index += 1
    print waypoint
    print joints
    print index
    Config.set('Joint_States',str(index),joints)
    Config.set('Pose_Goals',str(index),waypoint)

Config.set('Amount_Of_Keypoints',str(1),str(index))

Config.write(cfgfile)
cfgfile.close()