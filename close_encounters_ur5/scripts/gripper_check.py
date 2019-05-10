#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

"""
This program can be used to check the robotiq gripper.
It will activate, release, and wait for ENTER to be pressed, using the most basic program possible using ros.
When ENTER is pressed, it'll open or close.
"""

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('gripper_check_node', anonymous=False)
        rospy.loginfo("gripper check node starting")
        
        # start the publisher for the gripper command
        gripper_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        # publish activate
        gripper_command.rACT = 1
        gripper_command.rGTO = 1
        gripper_command.rSP  = 255
        gripper_command.rFR  = 0
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        gripper_command.rACT = 0
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        gripper_command.rACT = 1
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        rospy.loginfo("activated gripper")

        while True:
            # open the gripper
            gripper_command.rPR = 0
            gripper_publisher.publish(gripper_command)
            # wait for grabbing to be complete
            rospy.sleep(2)
            rospy.loginfo("released")
            rospy.loginfo("press ENTER to grab")
            raw_input()

            if rospy.is_shutdown():
                break

            # close gripper
            gripper_command.rPR = 255
            gripper_publisher.publish(gripper_command)
            # wait for grabbing to be complete
            rospy.sleep(2)
            rospy.loginfo("grabbed")
            rospy.loginfo("press ENTER to release")
            raw_input()

            if rospy.is_shutdown():
                break

    except rospy.ROSInterruptException:
        pass
