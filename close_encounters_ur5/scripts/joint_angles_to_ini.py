#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs.msg
import ConfigParser

"""
This program will print the current joint values and send them to an ini file every time enter is pressed.
"""

# start a new node
rospy.init_node('joint_angles_to_ini_node', anonymous=False)
rospy.loginfo("joint angles to ini node starting")

# start moveit
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("manipulator")

# ask for file name
file_name = raw_input('How do you want to name the file? Name: ')

# open file
iniHandler = ConfigParser.ConfigParser()
cfgfile = open("/home/ubuntu/catkin_ws/src/close_encounters_ur5/" + file_name + ".ini",'w')
iniHandler.add_section('Joint_States')
iniHandler.add_section('Pose_Goals')

index = 0

while not rospy.is_shutdown():
    # wait for enter
    print("Press ENTER to print the pose and joint values once.")
    raw_input()
    if rospy.is_shutdown():
        break

    index += 1

    # set values
    iniHandler.set('Joint_States',str(index),str(group.get_current_joint_values()))
    iniHandler.set('Pose_Goals',str(index),str(group.get_current_pose().pose))
    print index

# write to file before actually shutting down
iniHandler.write(cfgfile)
cfgfile.close()