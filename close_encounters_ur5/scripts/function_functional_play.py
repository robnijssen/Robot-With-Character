#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8
# import service types
# from close_encounters_ur5.srv import SetJointValues, SetJointValuesRequest, SetJointValuesResponse, GetJointValues, GetJointValuesRequest, GetJointValuesResponse

"""
This stays in idle, till it's commanded to do something by the /cmd_react
It will pick a play move when /cmd_react is set to 3.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # max speed&acceleration (1=100% 0.5=50%)
    max_speed = 0.8
    max_acceleration = 0.8
    # movement distance multipliers
    global_multiplier = 0.05
    x_multiplier_0 = 0.4
    x_multiplier_4 = 0.6
    # boundaries to be sure it won't move too far
    left_boundary_0 = -0.455052677785055
    left_boundary_4 = -6.210995327924387
    right_boundary_0 = -3.9625261465655726
    right_boundary_4 = -3.5705881754504603
    
class Variables:
    # a variable to keep track of what state the react is in
    cmd_react = 1
    # joint values for the Play
    # Change values according to playfield!
    # Go to position for cup detection / get cup detection value from previous node

    start_joint_values = [-2.2938314119922083, -1.116389576588766, -2.527221981679098, 0.5780388116836548, -4.732337776814596, -3.170588795338766]
    left_joint_values = [-1.9355886618243616, -1.116389576588766, -2.527485195790426, 0.5780388116836548, -5.126012508069174, -3.1705647150622767]
    right_joint_values = [-2.4651175181018274, -1.1164854208575647, -2.5320265928851526, 0.5780268907546997, -4.435282174740927, -3.170588795338766]
    
class Functions:
    def follow_joint_values_waypoints(self, goal_joint_values_waypoints):
        n = 0
        for i in goal_joint_values_waypoints:
            # compute a plan to get these joint values
            playGroup.set_joint_value_target(goal_joint_values_waypoints[n])
            # go to the planned position
            playGroup.go(wait=True)
            n += 1
        playGroup.stop()
        playGroup.clear_pose_targets()

# functions used in state machine

class Callbacks:
    def react(self, react):
        playVariables.cmd_react = react.data

# state machine

class PlayMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("play: Not active.")
        # publish feedback 1 (done rolling)
        fb_play_publisher.publish(1)
        rospy.sleep(playConstants.sleeptime)
    def mainRun(self):
        # publish feedback 0 (not active / not done rolling)
        fb_play_publisher.publish(0)
        rospy.sleep(playConstants.sleeptime)
    def next(self):
        if(playVariables.cmd_react == 3):
            return PlayMachine.goToStartPosition
        else:
            return PlayMachine.idle

class GoToStartPosition(State):
    def transitionRun(self):
        rospy.loginfo("play: Trying to go to the starting position. Looking at last known cup position.")
        # update Play direction and calulate relative joint angles for the Plays
        #playFunctions.calculate_joint_values()
        # compute a plan to get the start joint values
        playGroup.set_joint_value_target(playVariables.start_joint_values)
        # go to the planned position
        playGroup.go(wait=True)
        playGroup.stop()
        playGroup.clear_pose_targets()
        # set max speed&acceleration (after going to start, so the start is still at low speed settings)
        playGroup.set_max_velocity_scaling_factor(playConstants.max_speed)
        playGroup.set_max_acceleration_scaling_factor(playConstants.max_acceleration)
    def mainRun(self):
        pass
    def next(self):
        random_result = 0
        if random_result == 0:
            return PlayMachine.play0

class Play0(State):
    def transitionRun(self):
        rospy.loginfo("play: Trying Play 0.")
        # do Play 0
        playFunctions.follow_joint_values_waypoints([playVariables.left_joint_values, 
            playVariables.right_joint_values, 
            playVariables.left_joint_values, 
            playVariables.start_joint_values])
    def mainRun(self):
        pass
    def next(self):
        return PlayMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('function_emotional_play_node', anonymous=True)
        rospy.loginfo("emotional play function node starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        playGroup = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_react publisher to give feedback to the react node
        fb_play_publisher = rospy.Publisher('/fb_play_reaction', Int8, queue_size=1)

        playFunctions = Functions()
        playVariables = Variables()
        playCallbacks = Callbacks()
        playConstants = Constants()

        # init subscriber
        playGroupCmd_react = rospy.Subscriber("/cmd_react", Int8, playCallbacks.react)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        PlayMachine.idle = Idle()
        PlayMachine.goToStartPosition = GoToStartPosition()
        PlayMachine.play0 = Play0()

        ''' # init get cup position service from the position server
        rospy.wait_for_service('/get_cup_detection')
        playGetCupDetection = rospy.ServiceProxy('/get_cup_detection', GetJointValues)'''

        # run
        PlayMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
