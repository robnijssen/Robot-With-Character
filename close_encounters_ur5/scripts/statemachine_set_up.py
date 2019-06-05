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
    general_max_speed = 0.1
    general_max_acceleration = 0.1
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
    place_a_die_angles = [-2.1188, -1.5585, -1.5440, -1.5046, -4.7562, -1.3496] # to do: update these angles
    # a joint angle request ready with constants
    joint_request = SendGoalRequest()
    joint_request.goal, joint_request.speed, joint_request.acceleration, joint_request.tolerance, joint_request.delay = check_for_dice_angles, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    # a pose within reach to return to
    defaultPose = [] # to do: add this pose
    # a pose request ready with constants
    pose_request = SendGoalRequest()
    pose_request.goal, pose_request.speed, pose_request.acceleration, pose_request.tolerance, pose_request.delay = defaultPose, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    
    
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
        # reset dice in cup
        dice_in_cup = 0
    def mainRun(self):
        # publish feedback 0
        fb_set_up_publisher.publish(0)
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
        """
        if(setUpVariables.fb_move_executor == 1):
            return SetUpMachine.checkForCup
        else:
            return SetUpMachine.askForCup
        """
        return SetUpMachine.CheckForCup
        # to do: check fb_move_executor

class PlaceADie(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Placing a die.")
        # send to move queue
        setUpRequests.pose_request.goal = setUpRequests.place_a_die_angles
        setUpOverwriteGoal(setUpRequests.pose_request)
    def mainRun(self):
        pass
    def next(self):
        return SetUpMachine.releaseADie

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
        rospy.loginfo("set up actions node starting")

        # start the publisher for the gripper command
        setUpGripperGripperPublisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        setUpGripperCommand = outputMsg.Robotiq2FGripper_robot_output()

        # init /fb_set_up publisher to give feedback to the main control
        fb_set_up_publisher = rospy.Publisher('/fb_set_up', Int8, queue_size=1)

        setUpVariables = Variables()
        setUpRequests = Requests()
        setUpCallbacks = Callbacks()
        setUpConstants = Constants()

        # init subscribers
        setUpCmd_state = rospy.Subscriber("/cmd_state", Int8, setUpCallbacks.state)
        setUpFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, setUpCallbacks.fb_move_executor)

        # init services
        rospy.wait_for_service('/overwrite_goals')
        rospy.wait_for_service('/overwrite_pose_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/add_pose_goal')
        setUpOverwriteGoal = rospy.ServiceProxy('/overwrite_goals', SendGoal)
        setUpOverwritePoseGoal = rospy.ServiceProxy('/overwrite_pose_goal', SendGoal)
        setUpAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        setUpAddPoseGoal = rospy.ServiceProxy('/add_pose_goal', SendGoal)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        SetUpMachine.idle = Idle()
        SetUpMachine.goToDiceCheckingPosition = GoToDiceCheckingPosition()
        SetUpMachine.checkForDice = CheckForDice()
        SetUpMachine.pickADie = PickADie()
        SetUpMachine.grabADie = GrabADie()
        SetUpMachine.checkForCup = CheckForCup()
        SetUpMachine.askForCup = AskForCup()
        SetUpMachine.placeADie = PlaceADie()
        SetUpMachine.releaseADie = ReleaseADie()
        SetUpMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
