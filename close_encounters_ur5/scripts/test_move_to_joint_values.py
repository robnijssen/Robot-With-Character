#!/usr/bin/env python
import sys
import rospy
import time
from random import randint
# moveit stuff
import moveit_commander
import moveit_msgs
import geometry_msgs.msg

"""
This program is to test what joint responds to what exactly.
"""

# constants and variables used in state machine

class Constants:
    # starting joint values
    start_joint_values = [-2.2711683902660402, -1.0178875713292797, -2.5750410798412053, 0.5520000038105956, -4.749878720248225, -3.159967983084798]
    # instead of creating a reference, create a copy (by adding list())
    end_joint_values = list(start_joint_values)
    end_joint_values[4] += 0.4

def go_to_joint_values(goal_joint_values):
    # compute a plan to get these joint values
    group.set_joint_value_target(goal_joint_values)
    # go to the planned position
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('test_move_to_joint_values_node', anonymous=False)
        rospy.loginfo("test for moving to joint values starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander("manipulator")

        constants = Constants()
        
        while not rospy.is_shutdown():
            rospy.loginfo("Press ENTER to move one iteration.")
            raw_input()
            rospy.loginfo("Moving to start.")
            go_to_joint_values(constants.start_joint_values)
            rospy.loginfo("Moving to end.")
            go_to_joint_values(constants.end_joint_values)

    except rospy.ROSInterruptException:
        pass
