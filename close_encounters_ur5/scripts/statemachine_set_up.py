#!/usr/bin/env python
import sys
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic is published by the statemachine_control.py

The bot will try to set up the board by putting the dice in the cup.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # time to check the tray
    checktime = 5.0
    # time for the gripper to grab/release
    grabtime = 2
    # movement values
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    tolerance = 0.0001

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # a variable to keep track of the amount of dice in the cup
    dice_in_cup = 0
    # a variable to keep track of the amount of dice in the tray
    dice_in_tray = 0
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    
# functions used in state machine

class Callbacks:
    def state(self, state):
        setUpVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        setUpVariables.fb_move_executor = feedback.data
    def distance_to_face(self, coordinates):
        setUpVariables.distance_to_face = coordinates.d
    def face_angles_update(self, angles):
        setUpVariables.face_joint_angles.angles = angles.angles
    def number_of_dice(self, data):
        setUpVariables.dice_in_tray = data.data

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = setUpIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class SetUpMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Not active.")
        # tell the vision node to stop looking for dice or anything
        setUpVariables.vision_request.mode = 0
        setUpVisionChecks(setUpVariables.vision_request)
        # publish feedback 1 at the end of the cycle
        fb_set_up_publisher.publish(1)
        rospy.sleep(setUpConstants.sleeptime * 2)
        # publish feedback 0
        fb_set_up_publisher.publish(0)
        rospy.sleep(setUpConstants.sleeptime)
        # reset dice in cup
        dice_in_cup = 0
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.cmd_state == 2):
            rospy.sleep(setUpConstants.sleeptime)
            return SetUpMachine.goToDiceCheckingPosition
        else:
            return SetUpMachine.idle
            
class GoToDiceCheckingPosition(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Moving to dice checking position.")
        # send move to queue
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, setUpConstants.general_max_speed, setUpConstants.general_max_acceleration, setUpConstants.tolerance, 0.01
        request.goal = setUpFunctions.read_from_ini('check_for_dice_joint', '1')
        setUpOverwriteGoal(request)
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.fb_move_executor != 1):
            return SetUpMachine.goToDiceCheckingPosition
        else:
            return SetUpMachine.checkForDice

class CheckForDice(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Checking for dice.")
    def mainRun(self):
        rospy.sleep(setUpConstants.checktime)
        # tell the vision node to look for dice
        setUpVariables.vision_request.mode = 2
        setUpVisionChecks(setUpVariables.vision_request)
        rospy.sleep(setUpConstants.checktime)
        # tell the vision node to stop looking for dice
        setUpVariables.vision_request.mode = 0
        setUpVisionChecks(setUpVariables.vision_request)
    def next(self):
        print(setUpVariables.dice_in_tray)
        if setUpVariables.dice_in_tray > 0:
            return SetUpMachine.askForDiceInCup
        else:
            return SetUpMachine.idle

class AskForDiceInCup(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Asking to put the dice in the cup.")
        # send the asking movement to the move queue
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 2, setUpConstants.general_max_speed, setUpConstants.general_max_acceleration, setUpConstants.tolerance, 0.01
        request.goal = setUpFunctions.read_from_ini('ask_for_dice_pose', '1')
        setUpOverwriteGoal(request)
        for i in range(2, 7):
            request.goal = setUpFunctions.read_from_ini('ask_for_dice_pose', str(i))
            setUpAddGoal(request)
        request.goal = []
        setUpAddGoal(request) # request to start the cartesian path
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if setUpVariables.fb_move_executor != 1:
            return SetUpMachine.askForDiceInCup
        else:
            return SetUpMachine.goToDiceCheckingPosition

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_set_up_node', anonymous=True)
        rospy.loginfo("Set up: Node starting.")

        # start the publisher for the gripper command
        setUpGripperGripperPublisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        setUpGripperCommand = outputMsg.Robotiq2FGripper_robot_output()

        # init /fb_set_up publisher to give feedback to the main control
        fb_set_up_publisher = rospy.Publisher('/fb_set_up', Int8, queue_size=1)

        setUpVariables = Variables()
        setUpCallbacks = Callbacks()
        setUpFunctions = Functions()
        setUpConstants = Constants()

        # init ini reading/writing
        setUpIniHandler = ConfigParser()
        setUpIniPath = rospy.get_param('~set_up_path')
        rospy.loginfo("Set up: Using file: " + setUpIniPath)
        setUpIniHandler.read(setUpIniPath)

        # init subscribers
        setUpCmd_state = rospy.Subscriber("/cmd_state", Int8, setUpCallbacks.state)
        setUpFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, setUpCallbacks.fb_move_executor)
        setUpNumber_of_dice = rospy.Subscriber("/vision_amount", Int8, setUpCallbacks.number_of_dice)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        setUpOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        setUpAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        setUpVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        SetUpMachine.idle = Idle()
        SetUpMachine.goToDiceCheckingPosition = GoToDiceCheckingPosition()
        SetUpMachine.checkForDice = CheckForDice()
        SetUpMachine.askForDiceInCup = AskForDiceInCup()
        SetUpMachine().runAll()

    except rospy.ROSInterruptException:
        pass
