#!/usr/bin/env python
import sys
import rospy
import time
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic published on by the statemachine_control.py

When a person is detected, the bot will invite them to come closer and play
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # for debugging, time between detecting a face and continuing with the next state
    debugtime = 3

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 0
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track of how far away the face is
    inviteDistance_to_face = 0

# functions used in state machine

class Callbacks:
    def state(self, state):
        inviteVariables.inviteCmd_state = state.data
    def inviteDistance_to_face(self, distance):
        inviteVariables.inviteDistance_to_face = distance.data

# state machine

class InviteMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Not active.")
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
        # publish state 0
        cmd_invite_publisher.publish(0)
        # publish feedback 0 for debugging (no person within 1 meters)
        fb_invite_publisher.publish(0)
    def next(self):
        if(inviteVariables.cmd_state == 1):
            return InviteMachine.checkForPeople
        else:
            return InviteMachine.idle

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Checking for people.")
    def mainRun(self):
        rospy.sleep(inviteConstants.debugtime)
        # check for people executed here
        self.inviteDistance = inviteVariables.inviteDistance_to_face
        if self.inviteDistance == 2:
            # person within 2 meters
            fb_invite_publisher.publish(0)
        elif self.inviteDistance == 1:
            # person within 1 meter
            fb_invite_publisher.publish(1)
        else:
            # no person within 2 meters
            fb_invite_publisher.publish(2)
    def next(self):
        if self.distance == 2:
            return InviteMachine.inviteByLooking
            # to do: add more possibilities and selecting them at random
        else:
            rospy.sleep(inviteConstants.sleeptime)
            return InviteMachine.idle

class InviteByLooking(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Inviting person by at looking and shifting between cup and person.")
    def mainRun(self):
        # to do: add the invite by looking move here
        rospy.sleep(inviteConstants.debugtime) # for debugging, wait a bit instead of moving
    def next(self):
        return InviteMachine.checkForPeople

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_invite_node', anonymous=True)
        rospy.loginfo("invite actions node starting")

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /cmd_invite publisher
        cmd_invite_publisher = rospy.Publisher('/cmd_invite', Int8, queue_size=1)

        # init /fb_invite publisher to give feedback to the main control
        fb_invite_publisher = rospy.Publisher('/fb_invite', Int8, queue_size=1)

        # init subscriber
        inviteVariables = Variables()
        inviteCallbacks = Callbacks()
        inviteConstants = Constants()
        inviteCmd_state = rospy.Subscriber("/cmd_state", Int8, inviteCallbacks.state)
        inviteDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, inviteCallbacks.distance_to_face)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        InviteMachine.idle = Idle()
        InviteMachine.checkForPeople = CheckForPeople()
        InviteMachine.inviteByLooking = InviteByLooking()
        InviteMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
