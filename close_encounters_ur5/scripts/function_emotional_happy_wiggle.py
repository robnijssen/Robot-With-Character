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
from close_encounters_ur5.srv import SetJointValues, SetJointValuesRequest, SetJointValuesResponse, GetJointValues, GetJointValuesRequest, GetJointValuesResponse

"""
This stays in idle, till it's commanded to do something by the /cmd_react
It will pick a random happy move when /cmd_react is set to 3.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # max speed&acceleration (1=100% 0.5=50%)
    max_speed = 0.8
    max_acceleration = 0.8
    # movement distance multipliers
    global_multiplier = 0.1
    x_multiplier_0 = 0.3
    x_multiplier_4 = 0.7
    # boundaries to be sure it won't move too far
    # left [-0.455052677785055, -1.1653388182269495, -2.5022528807269495, 0.5620001554489136, -6.210995327924387, -3.169678036366598]
    # right [-3.9625261465655726, -1.1644156614886683, -2.5134311358081263, 0.5618680715560913, -3.5705881754504603, -3.1697259585009974]
    left_boundary_0 = -0.455052677785055
    left_boundary_4 = -6.210995327924387
    right_boundary_0 = -3.9625261465655726
    right_boundary_4 = -3.5705881754504603

    
class Variables:
    # a variable to keep track of what state the react is in
    cmd_react = 1
    # joint values for the wiggle
    start_joint_values = [-2.2938314119922083, -1.116389576588766, -2.527221981679098, 0.5780388116836548, -4.732337776814596, -3.170588795338766]
    left_joint_values = [-1.9355886618243616, -1.116389576588766, -2.527485195790426, 0.5780388116836548, -5.126012508069174, -3.1705647150622767]
    right_joint_values = [-2.4651175181018274, -1.1164854208575647, -2.5320265928851526, 0.5780268907546997, -4.435282174740927, -3.170588795338766]
    
class Functions:
    def go_to_face_joint_values(self):
        # request the joint values from the server
        req = GetJointValuesRequest()
        req.request = True
        res = GetJointValuesResponse()
        res = happyWiggleGetFaceJointAngles(req)
        face_values = [res.angle_0, res.angle_1, res.angle_2, res.angle_3, res.angle_4, res.angle_5]
        # compute a plan to get these joint values
        happyWiggleGroup.set_joint_value_target(face_values)
        # go to the planned position
        happyWiggleGroup.go(wait=True)
        happyWiggleGroup.stop()
        happyWiggleGroup.clear_pose_targets()
    def calculate_joint_values(self, start_joint_values):
        # overwrite with current face position values
        happyWiggleVariables.start_joint_values = start_joint_values
        happyWiggleVariables.left_joint_values = start_joint_values
        happyWiggleVariables.right_joint_values = start_joint_values
        # alter some x rotation values
        happyWiggleVariables.left_joint_values[0] = happyWiggleConstants.x_multiplier_0 * happyWiggleConstants.global_multiplier
        happyWiggleVariables.left_joint_values[4] = -1 * happyWiggleConstants.x_multiplier_4 * happyWiggleConstants.global_multiplier
        happyWiggleVariables.right_joint_values[0] = -1 * happyWiggleConstants.x_multiplier_0 * happyWiggleConstants.global_multiplier
        happyWiggleVariables.right_joint_values[4] = happyWiggleConstants.x_multiplier_4 * happyWiggleConstants.global_multiplier
        # make sure the x rotation won't go out of bound
        if happyWiggleVariables.left_joint_values[0] < happyWiggleConstants.left_boundary_0:
            happyWiggleVariables.left_joint_values[0] = happyWiggleConstants.left_boundary_0
        if happyWiggleVariables.left_joint_values[4] < happyWiggleConstants.left_boundary_4:
            happyWiggleVariables.left_joint_values[4] = happyWiggleConstants.left_boundary_4
        if happyWiggleVariables.right_joint_values[0] < happyWiggleConstants.right_boundary_0:
            happyWiggleVariables.right_joint_values[0] = happyWiggleConstants.right_boundary_0
        if happyWiggleVariables.right_joint_values[4] < happyWiggleConstants.right_boundary_4:
            happyWiggleVariables.right_joint_values[4] = happyWiggleConstants.right_boundary_4
    def follow_joint_values_waypoints(self, goal_joint_values_waypoints):
        n = 0
        for i in goal_joint_values_waypoints:
            # compute a plan to get these joint values
            happyWiggleGroup.set_joint_value_target(goal_joint_values_waypoints[n])
            # go to the planned position
            happyWiggleGroup.go(wait=True)
            n += 1
        happyWiggleGroup.stop()
        happyWiggleGroup.clear_pose_targets()

# functions used in state machine

class Callbacks:
    def react(self, react):
        happyWiggleVariables.cmd_react = react.data

# state machine

class WiggleMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("HappyWiggle: Not active.")
    def mainRun(self):
        # publish feedback 0 (not active / not done wiggling)
        fb_happy_publisher.publish(0)
        rospy.sleep(happyWiggleConstants.sleeptime)
    def next(self):
        if(happyWiggleVariables.cmd_react == 3):
            return WiggleMachine.goToStartPosition
        else:
            return WiggleMachine.idle

class GoToStartPosition(State):
    def transitionRun(self):
        rospy.loginfo("HappyWiggle: Trying to go to the starting position. Looking at last known face position.")
        # set max speed&acceleration
        happyWiggleGroup.set_max_velocity_scaling_factor(happyWiggleConstants.max_speed)
        happyWiggleGroup.set_max_acceleration_scaling_factor(happyWiggleConstants.max_acceleration)
    def mainRun(self):
        # go to start position
        happyWiggleFunctions.go_to_face_joint_values()
        #happyWiggleFunctions.go_to_joint_values(happyWiggleVariables.start_joint_values)
    def next(self):
        random_result = randint(0, 2)
        if random_result == 0:
            return WiggleMachine.wiggle0
        elif random_result == 1:
            return WiggleMachine.wiggle1
        else:
            return WiggleMachine.wiggle2

class Wiggle0(State):
    def transitionRun(self):
        rospy.loginfo("HappyWiggle: Trying wiggle 0.")
    def mainRun(self):
        # calulate relative joint angles
        happyWiggleFunctions.calculate_joint_values(happyWiggleGroup.get_current_joint_values())
        # do happy wiggle 0
        happyWiggleFunctions.follow_joint_values_waypoints([happyWiggleVariables.left_joint_values, 
            happyWiggleVariables.right_joint_values, 
            happyWiggleVariables.left_joint_values, 
            happyWiggleVariables.start_joint_values])
        # publish feedback 1 (done wiggling)
        fb_happy_publisher.publish(1)
        rospy.sleep(happyWiggleConstants.sleeptime)
    def next(self):
        return WiggleMachine.idle

class Wiggle1(State):
    def transitionRun(self):
        rospy.loginfo("HappyWiggle: Trying wiggle 1.")
    def mainRun(self):
        # calulate relative joint angles
        happyWiggleFunctions.calculate_joint_values(happyWiggleGroup.get_current_joint_values())
        # do happy wiggle 1
        happyWiggleFunctions.follow_joint_values_waypoints([happyWiggleVariables.right_joint_values, 
            happyWiggleVariables.left_joint_values, 
            happyWiggleVariables.right_joint_values, 
            happyWiggleVariables.start_joint_values])
        # publish feedback 1 (done wiggling)
        fb_happy_publisher.publish(1)
        rospy.sleep(happyWiggleConstants.sleeptime)
    def next(self):
        return WiggleMachine.idle

class Wiggle2(State):
    def transitionRun(self):
        rospy.loginfo("HappyWiggle: Trying wiggle 2.")
    def mainRun(self):
        # calulate relative joint angles
        happyWiggleFunctions.calculate_joint_values(happyWiggleGroup.get_current_joint_values())
        # do happy wiggle 2
        happyWiggleFunctions.follow_joint_values_waypoints([happyWiggleVariables.left_joint_values, 
            happyWiggleVariables.right_joint_values, 
            happyWiggleVariables.left_joint_values, 
            happyWiggleVariables.right_joint_values, 
            happyWiggleVariables.left_joint_values,
            happyWiggleVariables.start_joint_values])
        # publish feedback 1 (done wiggling)
        fb_happy_publisher.publish(1)
        rospy.sleep(happyWiggleConstants.sleeptime)
    def next(self):
        return WiggleMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('function_emotional_wiggle_node', anonymous=True)
        rospy.loginfo("emotional wiggle function node starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        happyWiggleGroup = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_react publisher to give feedback to the react node
        fb_happy_publisher = rospy.Publisher('/fb_happy_reaction', Int8, queue_size=1)

        happyWiggleFunctions = Functions()
        happyWiggleVariables = Variables()
        happyWiggleCallbacks = Callbacks()
        happyWiggleConstants = Constants()

        # init subscriber
        happyWiggleGroupCmd_react = rospy.Subscriber("/cmd_react", Int8, happyWiggleCallbacks.react)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        WiggleMachine.idle = Idle()
        WiggleMachine.goToStartPosition = GoToStartPosition()
        WiggleMachine.wiggle0 = Wiggle0()
        WiggleMachine.wiggle1 = Wiggle1()
        WiggleMachine.wiggle2 = Wiggle2()

        # init get face position service from the position server
        rospy.wait_for_service('/get_face_position')
        happyWiggleGetFaceJointAngles = rospy.ServiceProxy('/get_face_position', GetJointValues)

        # run
        WiggleMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
