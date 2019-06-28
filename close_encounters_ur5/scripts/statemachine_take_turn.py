#!/usr/bin/env python
import sys
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic is published by the statemachine_control.py

The bot will take a turn by picking the cup (with dice) and rolling the dice in the tray.
This program will then return to idle.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # time it has for checking for the cup
    checktime = 1.0
    # time it takes to open/close the gripper
    griptime = 2
    # movement values
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    #shake_max_speed = 0.9
    #shake_max_acceleration = 0.9
    shake_max_speed = 1.0
    shake_max_acceleration = 1.0
    tolerance = 0.001
    
class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # a variable that tells if the cup is currently in frame
    cup_in_frame = 0
    # a request for goal sending
    goal_req = SendGoalRequest()
    goal_req.speed, goal_req.acceleration, goal_req.tolerance, goal_req.delay = Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, 0.001
    # joint angles where the face was last spotted
    face_joint_angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573] # for now, default values are in here. to do: change to actual values

class Callbacks:
    def state(self, state):
        takeTurnVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        takeTurnVariables.fb_move_executor = feedback.data
    def get_status(self, data):
        # Get current data of gripper
        gripper_input.gACT = data.gACT
        gripper_input.gGTO = data.gGTO
        gripper_input.gSTA = data.gSTA
        gripper_input.gOBJ = data.gOBJ  
        gripper_input.gFLT = data.gFLT
        gripper_input.gPR = data.gPR
        gripper_input.gPO = data.gPO
        gripper_input.gCU = data.gCU
    def cup_detected(self, data):
        takeTurnVariables.cup_in_frame = data.data

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = takeTurnIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class TakeTurnMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Not active.")
        # publish done with taking turn
        fb_take_turn_publisher.publish(1)
        rospy.sleep(takeTurnConstants.sleeptime * 2)
        # publish feedback 0
        fb_take_turn_publisher.publish(0)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.cmd_state == 3:
            return TakeTurnMachine.goToCupCheckPosition
        else:
            return TakeTurnMachine.idle

class GoToCupCheckPosition(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Moving to cup checking position.")
        # send to move queue
        takeTurnVariables.goal_req.goal, takeTurnVariables.goal_req.type = takeTurnFunctions.read_from_ini('check_for_cup_joint', '1'), 0
        takeTurnOverwriteGoal(takeTurnVariables.goal_req)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.checkForCup
        else:
            return TakeTurnMachine.goToCupCheckPosition

class CheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Checking for cup.")
        # tell vision to look for the cup
        takeTurnVisionChecks(3)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.checktime)
    def next(self):
        if takeTurnVariables.cup_in_frame == 1:
            takeTurnVisionChecks(0)
            return TakeTurnMachine.pickCup
        else:
            takeTurnVisionChecks(0)
            return TakeTurnMachine.askForCup

class AskForCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Asking for cup.")
        # send to move queue
        takeTurnVariables.goal_req.goal, takeTurnVariables.goal_req.type = takeTurnFunctions.read_from_ini('ask_for_cup_pose', '1'), 2
        takeTurnOverwriteGoal(takeTurnVariables.goal_req)
        for i in range(1, 4):
            takeTurnVariables.goal_req.goal = takeTurnFunctions.read_from_ini('ask_for_cup_pose', str(i))
            takeTurnAddGoal(takeTurnVariables.goal_req)
        takeTurnVariables.goal_req.goal = []
        takeTurnAddGoal(takeTurnVariables.goal_req)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.goToCupCheckPosition
        else:
            return TakeTurnMachine.askForCup

class PickCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Picking the cup.")
        # send to move queue
        takeTurnVariables.goal_req.goal, takeTurnVariables.goal_req.type = takeTurnFunctions.read_from_ini('pick_cup_pose', '1'), 2
        takeTurnOverwriteGoal(takeTurnVariables.goal_req)
        takeTurnVariables.goal_req.goal = takeTurnFunctions.read_from_ini('pick_cup_pose', '2')
        takeTurnAddGoal(takeTurnVariables.goal_req)
        takeTurnVariables.goal_req.goal = []
        takeTurnAddGoal(takeTurnVariables.goal_req)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.grab
        else:
            return TakeTurnMachine.pickCup

class Grab(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Closing the gripper.")
        # tell gripper to close
        takeTurnGripperCommand.rPR = 200
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.griptime)
    def next(self):
        return TakeTurnMachine.roll
        # check pressure on the gripper
        #if gripper_input.gOBJ != 2:
        #    # tell gripper to open
        #    takeTurnGripperCommand.rPR = 0
        #    takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
        #    return TakeTurnMachine.askForCup
        #else:
        #    return TakeTurnMachine.roll
            
class Roll(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Rolling dice using the cup.")
        # send to move queue
        takeTurnVariables.goal_req.goal, takeTurnVariables.goal_req.type = takeTurnFunctions.read_from_ini('roll_1_pose', '1'), 2
        takeTurnOverwriteGoal(takeTurnVariables.goal_req)
        for i in range(2, 10):
            takeTurnVariables.goal_req.goal = takeTurnFunctions.read_from_ini('roll_1_pose', str(i))
            takeTurnAddGoal(takeTurnVariables.goal_req)
        takeTurnVariables.goal_req.goal = []
        takeTurnAddGoal(takeTurnVariables.goal_req)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.release
        else:
            return TakeTurnMachine.roll

class Release(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Opening the gripper.")
        # tell gripper to open
        takeTurnGripperCommand.rPR = 0
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.griptime)
    def next(self):
        return TakeTurnMachine.moveClear

class MoveClear(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Getting clear.")
        takeTurnVariables.goal_req.goal, takeTurnVariables.goal_req.type = takeTurnFunctions.read_from_ini('get_clear_joint', '1'), 0
        takeTurnOverwriteGoal(takeTurnVariables.goal_req)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.griptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.idle
        else:
            return TakeTurnMachine.moveClear

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_take_turn_node', anonymous=True)
        rospy.loginfo("Take turn: Node starting.")

        # start the publisher for the gripper command
        takeTurnGripperGripperPublisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        takeTurnGripperCommand = outputMsg.Robotiq2FGripper_robot_output()

        # activate gripper
        takeTurnGripperCommand.rACT = 1
        takeTurnGripperCommand.rGTO = 1
        takeTurnGripperCommand.rSP  = 50
        takeTurnGripperCommand.rFR  = 0
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
        rospy.sleep(0.2)
        takeTurnGripperCommand.rACT = 0
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
        rospy.sleep(0.2)
        takeTurnGripperCommand.rACT = 1
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
        rospy.sleep(0.2)

        # init /fb_take_turn publisher to give feedback to the main control
        fb_take_turn_publisher = rospy.Publisher('/fb_take_turn', Int8, queue_size=1)

        takeTurnVariables = Variables()
        takeTurnCallbacks = Callbacks()
        takeTurnFunctions = Functions()
        takeTurnConstants = Constants()

        # init ini reading/writing
        takeTurnIniHandler = ConfigParser()
        takeTurnIniPath = rospy.get_param('~take_turn_path')
        rospy.loginfo("Take turn: Using file: " + takeTurnIniPath)
        takeTurnIniHandler.read(takeTurnIniPath)

        # init subscribers
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, takeTurnCallbacks.get_status) # Subscribe to the gripper registers to get the status
        gripper_input = inputMsg.Robotiq2FGripper_robot_input()
        takeTurnCmd_state = rospy.Subscriber("/cmd_state", Int8, takeTurnCallbacks.state)
        takeTurnFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, takeTurnCallbacks.fb_move_executor)
        rospy.Subscriber('/vision_cup_detected', Int8, takeTurnCallbacks.cup_detected)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        takeTurnVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)
        takeTurnOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        takeTurnAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        TakeTurnMachine.idle = Idle()
        TakeTurnMachine.goToCupCheckPosition = GoToCupCheckPosition()
        TakeTurnMachine.checkForCup = CheckForCup()
        TakeTurnMachine.askForCup = AskForCup()
        TakeTurnMachine.pickCup = PickCup()
        TakeTurnMachine.grab = Grab()
        TakeTurnMachine.roll = Roll()
        TakeTurnMachine.release = Release()
        TakeTurnMachine.moveClear = MoveClear()
        TakeTurnMachine().runAll()

    except rospy.ROSInterruptException:
        pass
