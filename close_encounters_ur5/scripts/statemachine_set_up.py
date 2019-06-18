#!/usr/bin/env python
import sys
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.msg import AnglesList
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
    # time for the gripper to grab/release
    grabtime = 2
    # movement values
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    tolerance = 0.001

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # a variable to keep track of the amount of dice in the cup
    dice_in_cup = 0
    # a variable to keep track of the amount of dice in the tray
    dice_in_tray = 0
        
class Requests:
    # a position where the playing fied is completely visible
    check_for_dice_angles = [-2.1188, -1.5585, -1.5440, -1.5046, -4.7562, -1.3496] # to do: check these angles
    check_for_cup_angles = [-2.0824106, -1.16059, -2.5390597, -0.0665124, -4.364429, -1.5840481] # to do: check these angles
    place_a_die_angles_0 = [-2.0570381, -1.49767238, -1.844947, -1.063991, -4.778829, -1.608445] # to do: check these angles
    place_a_die_angles_1 = [-1.765497, -1.32242423, -1.841498, -1.2224, -4.72007, -1.608445] # to do: check these angles
    place_a_die_angles_2 = [-1.806762, -1.404764, -1.854193, -1.2226, -4.717292, -1.71169] # to do: check these angles
    # a joint angle request ready with constants
    joint_request = SendGoalRequest()
    joint_request.goal, joint_request.type, joint_request.speed, joint_request.acceleration, joint_request.tolerance, joint_request.delay = check_for_dice_angles, 0, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    # a pose within reach to return to
    defaultPose = [] # to do: add this pose
    # a pose request ready with constants
    pose_request = SendGoalRequest()
    pose_request.goal, pose_request.type, pose_request.speed, pose_request.acceleration, pose_request.tolerance, pose_request.delay = defaultPose, 1, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    
    
# functions used in state machine

class Callbacks:
    def state(self, state):
        setUpVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        setUpVariables.fb_move_executor = feedback.data
    def distance_to_face(self, distance):
        setUpVariables.distance_to_face = distance.data
    def face_angles_update(self, angles):
        setUpVariables.face_joint_angles.angles = angles.angles

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
            return SetUpMachine.goToDiceCheckingPosition
        else:
            return SetUpMachine.idle
            
class GoToDiceCheckingPosition(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Moving to dice checking position.")
        # send move to queue
        setUpRequests.joint_request.goal = setUpRequests.check_for_dice_angles
        setUpOverwriteGoal(setUpRequests.joint_request)
        setUpVariables.dice_in_cup += 1
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if setUpVariables.dice_in_cup == 2:
            return SetUpMachine.idle
        if(setUpVariables.fb_move_executor == 1):
            return SetUpMachine.checkForDice
        else:
            return SetUpMachine.goToDiceCheckingPosition

class CheckForDice(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Checking for dice.")
        # to do: tell vision node to look for dice
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if setUpVariables.dice_in_tray > 0:
            return SetUpMachine.pickADie
        else:
            return SetUpMachine.idle

class PickADie(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Picking a die.")
        # send to move queue
        setUpRequests.pose_request.goal = setUpRequests.defaultPose
        setUpOverwritePoseGoal(setUpRequests.pose_request)
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.fb_move_executor == 1):
            return SetUpMachine.grabADie
        else:
            return SetUpMachine.pickADie

class GrabADie(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Closing the gripper.")
        # tell gripper to close
        setUpGripperCommand.rPR = 250
        setUpGripperGripperPublisher.publish(takeTurnGripperCommand)
    def mainRun(self):
        rospy.sleep(setUpConstants.grabtime)
    def next(self):
        return SetUpMachine.checkForCup

class MoveToCheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Moving to check for the cup.")
        # send move to queue
        setUpRequests.joint_request.goal = setUpRequests.check_for_cup_angles
        setUpOverwriteGoal(setUpRequests.joint_request)
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.fb_move_executor == 1):
            return SetUpMachine.checkForCup
        else:
            return SetUpMachine.moveToCheckForCup
        
class CheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Checking for the cup.")
        # to do: tell vision node to check for a cup
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
        # to do: do a check for a detected cup
    def next(self):
        return SetUpMachine.placeADie
        # to do: make the return dependant on if the cup was detected

class AskForCup(State):
    def transitionRun(self):
        rospy.loginfo("Set Up: Asking for the cup.")
        # to do: send to the move queue
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.fb_move_executor == 1):
            return SetUpMachine.checkForCup
        else:
            return SetUpMachine.askForCup

class PlaceADie(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Placing a die.")
        # send to move queue
        setUpRequests.pose_request.goal = setUpRequests.place_a_die_angles_0
        setUpOverwriteGoal(setUpRequests.pose_request)
        setUpRequests.pose_request.goal = setUpRequests.place_a_die_angles_1
        setUpAddGoal(setUpRequests.pose_request)
        setUpRequests.pose_request.goal = setUpRequests.place_a_die_angles_2
        setUpAddGoal(setUpRequests.pose_request)
    def mainRun(self):
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.fb_move_executor == 3):
            return SetUpMachine.releaseADie
        else:
            return SetUpMachine.placeADie

class ReleaseADie(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Opening the gripper.")
        # tell gripper to open
        takeTurnGripperCommand.rPR = 0
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
    def mainRun(self):
        rospy.sleep(setUpConstants.grabtime)
    def next(self):
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
        setUpRequests = Requests()
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
        #setUpNumber_of_dice = rospy.Subscriber("/")

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        setUpOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        setUpAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        SetUpMachine.idle = Idle()
        SetUpMachine.goToDiceCheckingPosition = GoToDiceCheckingPosition()
        SetUpMachine.checkForDice = CheckForDice()
        
        SetUpMachine.pickADie = PickADie()
        SetUpMachine.grabADie = GrabADie()
        SetUpMachine.moveToCheckForCup = MoveToCheckForCup()
        SetUpMachine.checkForCup = CheckForCup()
        SetUpMachine.askForCup = AskForCup()
        SetUpMachine.placeADie = PlaceADie()
        SetUpMachine.releaseADie = ReleaseADie()
        
        SetUpMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
