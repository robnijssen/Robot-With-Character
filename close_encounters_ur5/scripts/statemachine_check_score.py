#!/usr/bin/env python
import sys
import rospy
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state.
This topic is published by the statemachine_control.py

This program will move the bot to a score checking position and return the score on /fb_check_score
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.1
    # time it needs for checking the score
    checktime = 1.0
    # for debugging, play alone delay time
    debugtime = 5
    # max speed/acceleration
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    # tolerance in joints
    tolerance = 0.00001

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    # a variable to check the amount of pips in frame
    score = 0
    
# functions used in state machine

class Callbacks:
    def state(self, state):
        checkScoreVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        checkScoreVariables.fb_move_executor = feedback.data
    def vision_score(self, score):
        checkScoreVariables.score = score.data

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = checkScoreIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class CheckScoreMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Check score: Not active.")
        # publish feedback 0
        fb_check_score_publisher.publish(0)
        rospy.sleep(checkScoreConstants.sleeptime)
    def mainRun(self):
        rospy.sleep(checkScoreConstants.sleeptime)
    def next(self):
        if(checkScoreVariables.cmd_state == 5):
            rospy.sleep(checkScoreConstants.sleeptime)
            return CheckScoreMachine.goToPosition
        else:
            return CheckScoreMachine.idle

class GoToPosition(State):
    def transitionRun(self):
        rospy.loginfo("Check score: Moving to score checking position.")
        # send to move queue
        request = SendGoalRequest()
        request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = checkScoreFunctions.read_from_ini('check_score_pose', '1'), 2, checkScoreConstants.general_max_speed, checkScoreConstants.general_max_acceleration, checkScoreConstants.tolerance, 0.01
        checkScoreOverwriteGoal(request)
        for i in range(1, 5): # note: position 4 is a valid looking position
            request.goal = checkScoreFunctions.read_from_ini('check_score_pose', str(i))
            checkScoreAddGoal(request)
        request.goal = []
        checkScoreAddGoal(request)
        # add the check score position in joint values
        request.goal, request.type = checkScoreFunctions.read_from_ini('check_score_end_position_joint', '1'), 0
        checkScoreAddGoal(request)
        # tell the vision node start checking for the score
        checkScoreVariables.vision_request.mode = 2
        checkScoreVisionChecks(checkScoreVariables.vision_request)
        rospy.sleep(checkScoreConstants.sleeptime)
    def mainRun(self):
        rospy.sleep(checkScoreConstants.sleeptime)
    def next(self):
        if(checkScoreVariables.fb_move_executor != 2):
            return CheckScoreMachine.goToPosition
        else:
            # give the vision time to do the checks
            rospy.sleep(checkScoreConstants.checktime)
            # tell the vision node to stop checking for the score
            checkScoreVariables.vision_request.mode = 0
            checkScoreVisionChecks(checkScoreVariables.vision_request)
            # publish the score as feedback
            fb_check_score_publisher.publish(checkScoreVariables.score)
            rospy.sleep(checkScoreConstants.sleeptime * 2)
            return CheckScoreMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_check_score_node', anonymous=True)
        rospy.loginfo("Check score: Node starting.")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_check_score publisher to give feedback to the main control
        fb_check_score_publisher = rospy.Publisher('/fb_check_score', Int8, queue_size=1)

        checkScoreVariables = Variables()
        checkScoreCallbacks = Callbacks()
        checkScoreFunctions = Functions()
        checkScoreConstants = Constants()

        # init ini reading/writing
        checkScoreIniHandler = ConfigParser()
        checkScoreIniPath = rospy.get_param('~check_score_path')
        rospy.loginfo("Check score: Using file: " + checkScoreIniPath)
        checkScoreIniHandler.read(checkScoreIniPath)

        # init subscribers
        checkScoreCmd_state = rospy.Subscriber("/cmd_state", Int8, checkScoreCallbacks.state)
        checkScoreFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, checkScoreCallbacks.fb_move_executor)
        checkScoreVision_score = rospy.Subscriber("/vision_score", Int8, checkScoreCallbacks.vision_score)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        checkScoreOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        checkScoreAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        checkScoreVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        CheckScoreMachine.idle = Idle()
        CheckScoreMachine.goToPosition = GoToPosition()
        CheckScoreMachine().runAll()

    except rospy.ROSInterruptException:
        pass
