#!/usr/bin/env python
import sys
import rospy
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8
from random import randint
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic is published by the statemachine_control.py

The bot will move so the person feels invited to take the cup and roll the dice.
It'll then wait for the person to do so (or for the person to walk away).
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # for debugging, time per turn
    debugtime = 3
    # number of total turns (best out of 3 is max 6 turns)
    max_turns = 6
    # movement values
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    tolerance = 0.001
    # default positions
    default_cup_position = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.77920324007] # for now, default looking straight in front values are in here. to do: change to actual values
    default_tray_position = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.77920324007] # for now, default looking straight in front values are in here. to do: change to actual values

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track of how far away the face is
    distance_to_face = -1
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    # amount of dice in the tray
    number_of_dice = 0
    # last known face position's joint values
    face_position = PositionList()
    face_position.angles = [-2.22257644335, -0.57710868517, -2.30035955111, -0.333354775106, 1.80458164215, -1.49498016039]
    face_position.pose = [0.137672240123, 0.0319267662058, 0.512328840913, -0.578146457675, -0.346376922049, 0.421628642553, 0.606629202338]

# functions used in state machine

class Callbacks:
    def state(self, state):
        waitForTurnVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        waitForTurnVariables.fb_move_executor = feedback.data
    def distance_to_face(self, coordinates):
        waitForTurnVariables.distance_to_face = coordinates.d
    def face_angles_update(self, position):
        waitForTurnVariables.face_position.angles = position.angles
        waitForTurnVariables.face_position.pose = position.pose
    def number_of_dice(self, data):
        waitForTurnVariables.number_of_dice = data.data

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = waitForTurnIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class WaitForTurnMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Not active.")
        # turn off the vision
        waitForTurnVariables.vision_request.mode = 0
        waitForTurnVisionChecks(waitForTurnVariables.vision_request)
        # publish feedback 0
        fb_wait_for_turn_publisher.publish(0)
        rospy.sleep(waitForTurnConstants.sleeptime)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.cmd_state == 4:
            return WaitForTurnMachine.goToFace
        else:
            return WaitForTurnMachine.idle

class GoToFace(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Moving to look at the player.")
        # send to move queue
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request.goal = waitForTurnVariables.face_position.angles
        waitForTurnOverwriteGoal(request)
        # tell vision to look for a face
        waitForTurnVariables.vision_request.mode = 1
        waitForTurnVisionChecks(waitForTurnVariables.vision_request)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor != 1:
            return WaitForTurnMachine.goToFace
        else:
            return WaitForTurnMachine.goToTray

class GoToTray(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Moving to look at the tray.")
        # send to move queue
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request.goal = waitForTurnFunctions.read_from_ini('go_to_tray_joint', '1')
        waitForTurnOverwriteGoal(request)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor != 1:
            return WaitForTurnMachine.goToTray
        else:
            return WaitForTurnMachine.checkTray

class CheckTray(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Checking if there are dice in the tray.")
        # tell vision to look for dice
        waitForTurnVariables.vision_request.mode = 2
        waitForTurnVisionChecks(waitForTurnVariables.vision_request)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.number_of_dice != 0:
            return WaitForTurnMachine.goToFaceAsking
        else:
            # publish done with waiting for turn and player is still here
            fb_wait_for_turn_publisher.publish(2)
            rospy.sleep(waitForTurnConstants.sleeptime)
            return WaitForTurnMachine.idle

class GoToFaceAsking(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Moving to look at the player as if asking.")
        # send to move queue
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request.goal = waitForTurnVariables.face_position.angles
        waitForTurnOverwriteGoal(request)
        if randint(0, 1) == 1:
            request.goal[5] += float(randint(3, 10)) / 10.0
        else:
            request.goal[5] -= float(randint(3, 10)) / 10.0
        waitForTurnAddGoal(request)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor != 1:
            return WaitForTurnMachine.goToFaceAsking
        else:
            return WaitForTurnMachine.goToTray

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_wait_for_turn_node', anonymous=True)
        rospy.loginfo("Wait for turn: Node starting.")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_wait_for_turn publisher to give feedback to the main control
        fb_wait_for_turn_publisher = rospy.Publisher('/fb_wait_for_turn', Int8, queue_size=1)

        waitForTurnVariables = Variables()
        waitForTurnCallbacks = Callbacks()
        waitForTurnFunctions = Functions()
        waitForTurnConstants = Constants()

        # init ini reading/writing
        waitForTurnIniHandler = ConfigParser()
        waitForTurnIniPath = rospy.get_param('~wait_for_turn_path')
        rospy.loginfo("Wait for turn: Using file: " + waitForTurnIniPath)
        waitForTurnIniHandler.read(waitForTurnIniPath)

        # init subscribers
        waitForTurnCmd_state = rospy.Subscriber("/cmd_state", Int8, waitForTurnCallbacks.state)
        waitForTurnFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, waitForTurnCallbacks.fb_move_executor)
        waitForTurnDistance_to_face = rospy.Subscriber("/vision_face_coordinates", FaceCoordinates, waitForTurnCallbacks.distance_to_face)
        waitForTurnFace_position = rospy.Subscriber("/face_position", PositionList, waitForTurnCallbacks.face_angles_update)
        waitForTurnNumber_of_dice = rospy.Subscriber("/vision_number_of_dice", Int8, waitForTurnCallbacks.number_of_dice)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        waitForTurnOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        waitForTurnAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        waitForTurnVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        WaitForTurnMachine.idle = Idle()
        WaitForTurnMachine.goToFace = GoToFace()
        WaitForTurnMachine.goToTray = GoToTray()
        WaitForTurnMachine.checkTray = CheckTray()
        WaitForTurnMachine.goToFaceAsking = GoToFaceAsking()
        WaitForTurnMachine().runAll()

    except rospy.ROSInterruptException:
        pass
