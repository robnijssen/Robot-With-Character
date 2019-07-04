#!/usr/bin/env python
# roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.66.3
import sys
import rospy
import time
import geometry_msgs.msg
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

"""
This script contains a class that publishes commands to the robotiq gripper via USB.
"""

class GripperCommand:
    def __init__(self):
        # init the grab time constant
        self.grab_time = 0.2
        # start the publisher for the gripper command
        self.gripper_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        # publish activate
        self.gripper_command.rACT = 1
        self.gripper_command.rGTO = 1
        self.gripper_command.rSP  = 255
        self.gripper_command.rFR  = 0
        self.gripper_publisher.publish(self.gripper_command)
        time.sleep(0.2)
        self.gripper_command.rACT = 0
        self.gripper_publisher.publish(self.gripper_command)
        time.sleep(0.2)
        self.gripper_command.rACT = 1
        self.gripper_publisher.publish(self.gripper_command)
        time.sleep(0.2)
        rospy.loginfo("activated gripper")
    def grab(self, input):
        if(input == True):
            # give grab command
            self.gripper_command.rPR = 255
            self.gripper_publisher.publish(self.gripper_command)
            # wait for grabbing to be complete
            time.sleep(self.grab_time)
            rospy.loginfo("grabbed")
        else:
            # give release command
            self.gripper_command.rPR = 0
            self.gripper_publisher.publish(self.gripper_command)
            # wait for grabbing to be complete
            time.sleep(self.grab_time)
            rospy.loginfo("released")
    def position(self, position):
        # go to input position
        self.gripper_command.rPR = position
        self.gripper_publisher.publish(self.gripper_command)
        # wait for grabbing to be complete
        time.sleep(self.grab_time)
        rospy.loginfo("moved gripper")