#!/usr/bin/env python
import sys
import rospy
import time
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8

"""
This stays in idle, till it's commanded to do something by the /cmd_react
It will pick a random sad move when /cmd_react is set to 4.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    # Adjust values according to playfield!

    sleeptime = 0.1
    start_joint_values = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.9526210069656372, -4.67750066915621, -3.170588795338766]
    head_joint_values = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.4526210069656372, -4.67750066915621, -3.170588795338766]
    body_joint_values = [-2.315057341252462, -1.554232374774378, -2.5245259443866175, 0.9526210069656372, -4.67750066915621, -3.170588795338766]
    head_up_joint_values = [-2.315057341252462, -1.0054232374774378, -2.5245259443866175, 0.4526210069656372, -4.67750066915621, -3.170588795338766]
    #head_up_joint_values = Looking at face
    
class Variables:
    # a variable to keep track of what state the react is in
    sadCmd_react = 1

class Functions:
    def go_to_joint_values(self, goal_joint_values):
        # compute a plan to get these joint values
        sadGroup.set_joint_value_target(goal_joint_values)
        # go to the planned position
        plan = sadGroup.go(wait=True)
        sadGroup.stop()
        sadGroup.clear_pose_targets()

# functions used in state machine

class Callbacks:
    def react(self, react):
        sadVariables.sadCmd_react = react.data

# state machine

class SadMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Sad: Not active.")
    def mainRun(self):
        # publish feedback 0 (not active / not done being sad)
        fb_sad_publisher.publish(0)
        rospy.sleep(sadConstants.sleeptime)
    def next(self):
        if(sadVariables.sadCmd_react == 4):
            return SadMachine.goToStartPosition
        else:
            return SadMachine.idle

class GoToStartPosition(State):
    def transitionRun(self):
        rospy.loginfo("Sad: Trying to go to the starting position.")
    def mainRun(self):
        # go to start position
        sadFunctions.go_to_joint_values(sadConstants.start_joint_values)
    def next(self):
        random_result = randint(0, 2)
        if random_result == 0:
            return SadMachine.sad0
        elif random_result == 1:
            return SadMachine.sad1
        else:
            return SadMachine.sad2

class Sad0(State):
    def transitionRun(self):
        rospy.loginfo("Sad: Trying being sad 0.")
    def mainRun(self):
        # do sad 0
        sadFunctions.go_to_joint_values(sadConstants.start_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.head_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.body_joint_values)
        # publish feedback 1 (done being sad)
        fb_sad_publisher.publish(1)
        rospy.sleep(sadConstants.sleeptime)
    def next(self):
        return SadMachine.idle

class Sad1(State):
    def transitionRun(self):
        rospy.loginfo("Sad: Trying sad 1.")
    def mainRun(self):
        # do sad 1
        sadFunctions.go_to_joint_values(sadConstants.start_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.body_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.head_up_joint_values)
        # publish feedback 1 (done being sad)
        fb_sad_publisher.publish(1)
        rospy.sleep(sadConstants.sleeptime)
    def next(self):
        return SadMachine.idle

class Sad2(State):
    def transitionRun(self):
        rospy.loginfo("Sad: Trying sad 2.")
    def mainRun(self):
        # do sad 2        
        sadFunctions.go_to_joint_values(sadConstants.start_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.body_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.head_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.head_up_joint_values)
        sadFunctions.go_to_joint_values(sadConstants.head_joint_values)
        # publish feedback 1 (done being sad)
        fb_sad_publisher.publish(1)
        rospy.sleep(sadConstants.sleeptime)
    def next(self):
        return SadMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('function_emotional_sad_node', anonymous=True)
        rospy.loginfo("emotional sad function node starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        sadGroup = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_react publisher to give feedback to the react node
        fb_sad_publisher = rospy.Publisher('/fb_sad_reaction', Int8, queue_size=1)

        sadFunctions = Functions()
        sadVariables = Variables()
        sadCallbacks = Callbacks()
        sadConstants = Constants()

        # init subscriber
        sadCmd_react = rospy.Subscriber("/cmd_react", Int8, sadCallbacks.react) #namechange subscriber?
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        SadMachine.idle = Idle()
        SadMachine.goToStartPosition = GoToStartPosition()
        SadMachine.sad0 = Sad0()
        SadMachine.sad1 = Sad1()
        SadMachine.sad2 = Sad2()
        SadMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass