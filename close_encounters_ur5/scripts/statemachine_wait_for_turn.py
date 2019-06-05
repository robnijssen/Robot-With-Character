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
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.srv import SetVisionMode, SetVisionModeRequest, SetVisionModeResponse
from close_encounters_ur5.msg import AnglesList

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
    general_max_speed = 0.1
    general_max_acceleration = 0.1
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
    # last known face position's joint values
    face_joint_angles = AnglesList()
    face_joint_angles.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.77920324007]

# functions used in state machine

class Callbacks:
    def state(self, state):
        waitForTurnVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        waitForTurnVariables.fb_move_executor = feedback.data
    def distance_to_face(self, distance):
        waitForTurnVariables.distance_to_face = distance.data
    def face_angles_update(self, angles):
        waitForTurnVariables.face_joint_angles.angles = angles.angles

# state machine

class WaitForTurnMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Not active.")
        # publish done with waiting for turn
        fb_wait_for_turn_publisher.publish(1)
        rospy.sleep(waitForTurnConstants.sleeptime)
        # publish feedback 0
        fb_wait_for_turn_publisher.publish(0)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.cmd_state == 4:
            return WaitForTurnMachine.inviteForTurn
        else:
            return WaitForTurnMachine.idle

class InviteForTurn(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Inviting player to take a turn.")
        # produce requests for the move queue
        request0, request1, request2, request3, request4 = SendGoalRequest(), SendGoalRequest(), SendGoalRequest(), SendGoalRequest(), SendGoalRequest()
        request0.goal, request0.speed, request0.acceleration, request0.tolerance, request0.delay = waitForTurnVariables.face_joint_angles.angles, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request1.goal, request1.speed, request1.acceleration, request1.tolerance, request1.delay = waitForTurnConstants.default_cup_position, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request2.goal, request2.speed, request2.acceleration, request2.tolerance, request2.delay = waitForTurnConstants.default_tray_position, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        request3.goal, request3.speed, request3.acceleration, request3.tolerance, request3.delay = waitForTurnVariables.face_joint_angles.angles, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        # send request list to the move queue
        waitForTurnOverwriteGoals(request0)
        waitForTurnAddGoal(request1)
        waitForTurnAddGoal(request2)
        waitForTurnAddGoal(request0)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor == 4:
            return WaitForTurnMachine.checkForCup
        else:
            return WaitForTurnMachine.inviteForTurn

class CheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Checking if the cup is picked.")
        # send to move queue here
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor == 4:
            return WaitForTurnMachine.trackCup
        else:
            return WaitForTurnMachine.checkForCup

class TrackCup(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Tracking the cup in the player's hand.")
        # send to move queue here
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor == 4:
            return WaitForTurnMachine.checkForDice
        else:
            return WaitForTurnMachine.trackCup

class CheckForDice(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Checking for dice in the tray.")
        # send to move queue here
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        if waitForTurnVariables.fb_move_executor == 4:
            return WaitForTurnMachine.checkScore
        else:
            return WaitForTurnMachine.checkForDice

class GoToScoreCheckingPosition(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Moving to position for checking score.")
        # send to move queue
        request = SendGoalRequest()
        goal = [-2.1188, -1.5585, -1.5440, -1.5046, -4.7562, -1.3496]
        request.goal, request.speed, request.acceleration, request.tolerance, request.delay = goal, waitForTurnConstants.general_max_speed, waitForTurnConstants.general_max_acceleration, waitForTurnConstants.tolerance, waitForTurnConstants.sleeptime
        waitForTurnOverwriteGoals(request)
    def mainRun(self):
        rospy.sleep(waitForTurnConstants.sleeptime * 2)
    def next(self):
        if waitForTurnVariables.fb_move_executor == 1:
            return WaitForTurnMachine.checkScore
        else:
            return WaitForTurnMachine.goToScoreCheckingPosition

class CheckScore(State):
    def transitionRun(self):
        rospy.loginfo("Wait for turn: Checking score.")
    def mainRun(self):
        # set vision mode to dice recognition for position and score
        # pass the player score on to control
        # set vision mode to not check anything
        rospy.sleep(waitForTurnConstants.sleeptime)
    def next(self):
        return WaitForTurnMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_wait_for_turn_node', anonymous=True)
        rospy.loginfo("wait for turn actions node starting")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_wait_for_turn publisher to give feedback to the main control
        fb_wait_for_turn_publisher = rospy.Publisher('/fb_wait_for_turn', Int8, queue_size=1)

        waitForTurnVariables = Variables()
        waitForTurnCallbacks = Callbacks()
        waitForTurnConstants = Constants()

        # init subscribers
        waitForTurnCmd_state = rospy.Subscriber("/cmd_state", Int8, waitForTurnCallbacks.state)
        waitForTurnFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, waitForTurnCallbacks.fb_move_executor)
        waitForTurnDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, waitForTurnCallbacks.distance_to_face)
        waitForTurnFace_joint_angles = rospy.Subscriber("/face_joint_angles", AnglesList, waitForTurnCallbacks.face_angles_update)

        # init services
        rospy.wait_for_service('/overwrite_goals')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        waitForTurnOverwriteGoals = rospy.ServiceProxy('/overwrite_goals', SendGoal)
        waitForTurnAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        waitForTurnVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        WaitForTurnMachine.idle = Idle()
        WaitForTurnMachine.inviteForTurn = InviteForTurn()
        WaitForTurnMachine.checkForCup = CheckForCup()
        WaitForTurnMachine.trackCup = TrackCup()
        WaitForTurnMachine.checkForDice = CheckForDice()
        WaitForTurnMachine.goToScoreCheckingPosition = GoToScoreCheckingPosition()
        WaitForTurnMachine.checkScore = CheckScore()
        WaitForTurnMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
