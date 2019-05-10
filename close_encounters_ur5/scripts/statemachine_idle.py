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
This stays in idle, till it's commanded to do something by the /cmd_state.
This topic published on by the statemachine_control.py.

As long as nobody is close to play with, it will try to entertain itself.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # for debugging, play alone delay time
    debugtime = 5
    # time to track before checking around again
    #track_time = 15 # high for debugging tracker, ~ 5 for actual running

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # a variable to keep track if a person is detected
    person_detected = False
    # feedback if the check for people move is done
    fb_check_for_people_done = 0
    # a variable to keep track of the distance to face topic
    distance_to_face = 0

# functions used in state machine

class Callbacks:
    def state(self, state):
        idleVariables.cmd_state = state.data
    def fb_check_for_people_done(self, fb_done):
        idleVariables.fb_check_for_people_done = fb_done.data
    def distance_to_face(self, distance):
        idleVariables.distance_to_face = distance.data

# state machine

class IdleMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Not active.")
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
        # publish state 0
        cmd_idle_publisher.publish(0)
        # publish feedback 0
        fb_idle_publisher.publish(0)
    def next(self):
        if(idleVariables.cmd_state == 0):
            return IdleMachine.checkForPeople
        else:
            return IdleMachine.idle

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Checking for people.")
        # check for people executed here
        cmd_idle_publisher.publish(1)
        idleVariables.person_detected = False
    def mainRun(self):
        if idleVariables.distance_to_face > 0:
            idleVariables.person_detected = True
            fb_idle_publisher.publish(1)
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.person_detected == False:
            if idleVariables.fb_check_for_people_done == False:
                return IdleMachine.checkForPeople
            elif idleVariables.distance_to_face == 0:
                return IdleMachine.followPeople
            else:
                # to do: add more possibilities
                #random_result = randint(0,1)
                #if random_result == 0:
                #   ...
                return IdleMachine.playAlone
        else:
            return IdleMachine.idle

class FollowPeople(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Following people for a bit")
    def mainRun(self):
        cmd_idle_publisher.publish(2)
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.distance_to_face > 0:
            fb_idle_publisher.publish(1)
            rospy.sleep(idleConstants.sleeptime)
            return IdleMachine.idle
        elif idleVariables.distance_to_face == 0:
            return IdleMachine.followPeople
        else:
            return IdleMachine.checkForPeople

class PlayAlone(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Playing by myself for a bit")
    def mainRun(self):
        # to do: add the playing by myself move here
        cmd_idle_publisher.publish(0)
        # wait for a bit
        rospy.sleep(idleConstants.debugtime)
    def next(self):
        return IdleMachine.checkForPeople

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_idle_node', anonymous=True)
        rospy.loginfo("idle actions node starting")

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /cmd_idle publisher
        cmd_idle_publisher = rospy.Publisher('/cmd_idle', Int8, queue_size=1)

        # init /fb_idle publisher to give feedback to the main control
        fb_idle_publisher = rospy.Publisher('/fb_idle', Int8, queue_size=1)

        idleVariables = Variables()
        idleCallbacks = Callbacks()
        idleConstants = Constants()

        # init subscribers
        idleCmd_state = rospy.Subscriber("/cmd_state", Int8, idleCallbacks.state)
        idleFb_check_for_people = rospy.Subscriber("/fb_check_for_people", Int8, idleCallbacks.fb_check_for_people_done)
        idleDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, idleCallbacks.distance_to_face)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        IdleMachine.idle = Idle()
        IdleMachine.checkForPeople = CheckForPeople()
        IdleMachine.followPeople = FollowPeople()
        IdleMachine.playAlone = PlayAlone()
        IdleMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
