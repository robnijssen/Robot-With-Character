#!/usr/bin/env python
import sys
import rospy
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
#from random import randint # random generator
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8

"""
This is the control program that checks which node should be running.

The program waits for a person to come within 2 meters (or so),
then it'll try to invite the person to a game of dice.
If the person comes within 1 meter, it'll try to play a game with the person.
When done playing, it'll react on the outcome.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3

class Variables:
    # variables to keep track of the feedback
    idle_feedback = 0
    invite_feedback = 0
    play_feedback = 0
    react_feedback = 0

# functions used in state machine

class Callbacks:
    def idle(self, feedback):
        controlVariables.idle_feedback = feedback.data
    def invite(self, feedback):
        controlVariables.invite_feedback = feedback.data
    def play(self, feedback):
        controlVariables.play_feedback = feedback.data
    def react(self, feedback):
        controlVariables.react_feedback = feedback.data

# state machine

class MainControlMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Control: Idle state should be running now.")
    def mainRun(self):
        # publish state 0
        cmd_state_publisher.publish(0)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.idle_feedback == 0):
            # still no person detected within 2 meters
            return MainControlMachine.idle
        elif(controlVariables.idle_feedback == 1):
            # person detected within 2 meters
            return MainControlMachine.invite
        else:
            rospy.logerr("Control: Transition from state idle not found.")
            return MainControlMachine.idle

class Invite(State):
    def transitionRun(self):
        rospy.loginfo("Control: Invite people state should be running now.")
    def mainRun(self):
        # publish state 1
        cmd_state_publisher.publish(1)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.invite_feedback == 0):
            # person still detected within 2 meters
            return MainControlMachine.invite
        elif(controlVariables.invite_feedback == 1):
            # person detected within 1 meter
            return MainControlMachine.play
        elif(controlVariables.invite_feedback == 2):
            # no person detected within 2 meters
            return MainControlMachine.idle
        else:
            rospy.logerr("Control: Transition from state invite not found.")
            return MainControlMachine.invite

class Play(State):
    def transitionRun(self):
        rospy.loginfo("Control: Play game state should be running now.")
    def mainRun(self):
        # publish state 2
        cmd_state_publisher.publish(2)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.play_feedback == 0):
            # (not done playing) and (person still within 2 meter)
            return MainControlMachine.play
        elif(controlVariables.play_feedback == 1):
            # (not done playing) and (no person within 2 meters)
            return MainControlMachine.idle
        elif(controlVariables.play_feedback == 2):
            # (done playing) and (person within 2 meter)
            return MainControlMachine.react
        elif(controlVariables.play_feedback == 3):
            # (done playing) and (no person within 2 meter)
            return MainControlMachine.idle
        else:
            rospy.logerr("Control: Transition from state play not found.")
            return MainControlMachine.play

class React(State):
    def transitionRun(self):
        rospy.loginfo("Control: React on outcome should be running now.")
    def mainRun(self):
        # publish state 3
        cmd_state_publisher.publish(3)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.react_feedback == 0):
            # not done reacting
            return MainControlMachine.react
        elif(controlVariables.react_feedback == 1):
            # (done reacting) and (person within 2 meter)
            return MainControlMachine.invite
        elif(controlVariables.react_feedback == 2):
            # (done reacting) and (no person within 2 meter)
            return MainControlMachine.idle
        else:
            rospy.logerr("Control: Transition from state react not found.")
            return MainControlMachine.react

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_control_node', anonymous=True)
        # wait 10 seconds to be sure everything started before commands are given
        rospy.sleep(10)
        rospy.loginfo("statemachine control starting")

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /cmd_state publisher
        cmd_state_publisher = rospy.Publisher('/cmd_state', Int8, queue_size=1)

        # init subscribers
        controlVariables = Variables()
        controlConstants = Constants()
        controlCallbacks = Callbacks()
        controlFb_idle = rospy.Subscriber("/fb_idle", Int8, controlCallbacks.idle)
        controlFb_invite = rospy.Subscriber("/fb_invite", Int8, controlCallbacks.invite)
        controlFb_play = rospy.Subscriber("/fb_play", Int8, controlCallbacks.play)
        controlFb_react = rospy.Subscriber("/fb_react", Int8, controlCallbacks.react)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        MainControlMachine.idle = Idle()
        MainControlMachine.invite = Invite()
        MainControlMachine.play = Play()
        MainControlMachine.react = React()
        MainControlMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
