#!/usr/bin/env python
import sys
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.msg import AnglesList

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
    # time it takes to open/close the gripper
    griptime = 2
    # movement values
    general_max_speed = 0.1
    general_max_acceleration = 0.1
    #shake_max_speed = 0.9
    #shake_max_acceleration = 0.9
    shake_max_speed = 0.1
    shake_max_acceleration = 0.1
    tolerance = 0.001
    
class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_executor = 0
    # a variable that tells if the cup is currently in frame
    cup_in_frame = False
    # a variable for checking if the cup is present
    cup_detected = False
    # joint angles where the face was last spotted
    face_joint_angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573] # for now, default values are in here. to do: change to actual values
    
class Requests:
    # joint angles where the cup should be in frame
    start_joint_angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573] # for now, default values are in here. to do: change to actual values
    # joint angles asking for the cup
    ask_joint_angles = [-2.0824106375323694, -1.1605928579913538, -2.5390597025500696, -0.06651240984071904, -4.364429179822103, -1.5051539579974573]
    # joint angles for picking, shaking, rolling, and placing
    startJointValues = [-1.95703870455, -1.24395019213, -2.39311916033, -0.233633343373, -4.47188872496, -1.77920324007]
    pickAngles = [-1.9672425428973597, -1.7987373510943812, -2.5357888380633753, 0.9961973428726196, -4.408938948308126, -1.6784494558917444]
    shakeAngles0 = [-1.9673622290240687, -1.6156447569476526, -2.505547348652975, 0.6628247499465942, -4.409106794987814, -1.6784732977496546]
    shakeAngles1 = [-2.171168629323141, -1.4821727911578577, -2.4504461924182337, 0.7767406702041626, -5.008367482815878, -2.258251969014303]
    shakeAngles2 = [-2.171168629323141, -1.4821727911578577, -2.4504461924182337, 0.7767406702041626, -5.008367482815878, -2.258251969014303]
    shakeAngles3 = [-2.1284917036639612, -1.5152094999896448, -2.450386349354879, 0.776788592338562, -4.580230657254354, -1.0931628386126917]
    rollAngles0 = [-2.367655102406637, -1.7729218641864222, -2.2259696165667933, 0.15553498268127441, -4.878737513219015, -2.0050671736346644]
    rollAngles1 = [-2.128000561391012, -1.9640992323504847, -2.2051385084735315, 0.5728284120559692, -5.225958649312155, -4.191371266041891]
    placeAngles0 = [-2.179335419331686, -1.6346343199359339, -2.4752472082721155, 0.7155462503433228, -4.206889454518453, -1.5]
    placeAngles1 = [-1.9673622290240687, -1.6156447569476526, -2.505547348652975, 0.6628247499465942, -4.409106794987814, -1.6784732977496546]
    checkScoreAngles = [-2.1188, -1.5585, -1.5440, -1.5046, -4.7562, -1.3496]
    # a request ready with constants and the start position
    request = SendGoalRequest()
    request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = start_joint_angles, 0, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    
# functions used in state machine

class Callbacks:
    def state(self, state):
        takeTurnVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        takeTurnVariables.fb_move_executor = feedback.data
    def getStatus (self, data):
        # Get current data of gripper
        gripper_input.gACT = data.gACT
        gripper_input.gGTO = data.gGTO
        gripper_input.gSTA = data.gSTA
        gripper_input.gOBJ = data.gOBJ  
        gripper_input.gFLT = data.gFLT
        gripper_input.gPR = data.gPR
        gripper_input.gPO = data.gPO
        gripper_input.gCU = data.gCU

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
            return TakeTurnMachine.goToStart
        else:
            return TakeTurnMachine.idle

class GoToStart(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Moving to start position.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.start_joint_angles
        takeTurnOverwriteGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.checkForCup
        else:
            return TakeTurnMachine.goToStart

class CheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Checking for cup.")
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
        if takeTurnVariables.cup_in_frame == True:
            # it's spotted while facing the cup stand, so it's assumed to be in place
            takeTurnVariables.cup_detected = True
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            if takeTurnVariables.cup_detected == True:
                return TakeTurnMachine.pickCup
            else:
                #return TakeTurnMachine.askForCup
                # for the first simple playthrough, just pick the cup
                return TakeTurnMachine.pickCup
        else:
            return TakeTurnMachine.checkForCup

class AskForCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Asking for cup.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnVariables.face_joint_angles
        takeTurnOverwriteGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.ask_joint_angles
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnVariables.face_joint_angles
        takeTurnAddGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 3:
            return TakeTurnMachine.checkForCup
        else:
            return TakeTurnMachine.askForCup

class PickCup(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Picking cup.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.startJointValues
        takeTurnOverwriteGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.pickAngles
        takeTurnAddGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 2:
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
        return TakeTurnMachine.shake

class Shake(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Shaking cup.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles0
        takeTurnOverwriteGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles1
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal, takeTurnRequests.request.speed, takeTurnRequests.request.acceleration, takeTurnRequests.request.tolerance, takeTurnRequests.request.delay = takeTurnRequests.start_joint_angles, Constants().shake_max_speed, Constants().shake_max_acceleration, Constants().tolerance, Constants().sleeptime
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles1
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles2
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles1
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.shakeAngles2
        takeTurnAddGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal, takeTurnRequests.request.speed, takeTurnRequests.request.acceleration, takeTurnRequests.request.tolerance, takeTurnRequests.request.delay = takeTurnRequests.start_joint_angles, Constants().general_max_speed, Constants().general_max_acceleration, Constants().tolerance, Constants().sleeptime
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime)
    def next(self):
        if takeTurnVariables.fb_move_executor == 6:
            return TakeTurnMachine.roll
        else:
            return TakeTurnMachine.shake
            
class Roll(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Rolling dice with cup.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.rollAngles0
        takeTurnOverwriteGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.rollAngles1
        takeTurnAddGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime * 2)
    def next(self):
        if takeTurnVariables.fb_move_executor == 2:
            return TakeTurnMachine.place
        else:
            return TakeTurnMachine.roll
            
class Place(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Placing cup back.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.placeAngles0
        takeTurnOverwriteGoal(takeTurnRequests.request)
        takeTurnRequests.request.goal = takeTurnRequests.placeAngles1
        takeTurnAddGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime * 2)
    def next(self):
        if takeTurnVariables.fb_move_executor == 2:
            return TakeTurnMachine.release
        else:
            return TakeTurnMachine.place

class Release(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Opening gripper.")
        # tell gripper to open
        takeTurnGripperCommand.rPR = 0
        takeTurnGripperGripperPublisher.publish(takeTurnGripperCommand)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.griptime)
    def next(self):
        return TakeTurnMachine.goToScoreCheckingPosition

class GoToScoreCheckingPosition(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Moving to check for score position.")
        # send to move queue
        takeTurnRequests.request.goal = takeTurnRequests.checkScoreAngles
        takeTurnOverwriteGoal(takeTurnRequests.request)
    def mainRun(self):
        rospy.sleep(takeTurnConstants.sleeptime * 2)
    def next(self):
        if takeTurnVariables.fb_move_executor == 1:
            return TakeTurnMachine.checkScore
        else:
            return TakeTurnMachine.goToScoreCheckingPosition

class CheckScore(State):
    def transitionRun(self):
        rospy.loginfo("Take turn: Checking score.")
    def mainRun(self):
        # set vision mode to dice recognition for position and score
        # pass the bot score on to control
        # set vision mode to not check anything
        rospy.sleep(takeTurnConstants.sleeptime * 2)
    def next(self):
        return TakeTurnMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_take_turn_node', anonymous=True)
        rospy.loginfo("take turn actions node starting")

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
        takeTurnConstants = Constants()
        takeTurnRequests = Requests()

        # init subscribers
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, takeTurnCallbacks.getStatus) # Subscribe to the gripper registers to get the status
        gripper_input = inputMsg.Robotiq2FGripper_robot_input()
        takeTurnCmd_state = rospy.Subscriber("/cmd_state", Int8, takeTurnCallbacks.state)
        takeTurnFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, takeTurnCallbacks.fb_move_executor)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        takeTurnOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        takeTurnAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        TakeTurnMachine.idle = Idle()
        TakeTurnMachine.goToStart = GoToStart()
        TakeTurnMachine.checkForCup = CheckForCup()
        TakeTurnMachine.askForCup = AskForCup()
        TakeTurnMachine.pickCup = PickCup()
        TakeTurnMachine.grab = Grab()
        TakeTurnMachine.shake = Shake()
        TakeTurnMachine.roll = Roll()
        TakeTurnMachine.place = Place()
        TakeTurnMachine.release = Release()
        TakeTurnMachine.goToScoreCheckingPosition = GoToScoreCheckingPosition()
        TakeTurnMachine.checkScore = CheckScore()
        TakeTurnMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
