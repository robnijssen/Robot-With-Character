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
    # movement values
    general_max_speed = 0.1
    general_max_acceleration = 0.1
    tolerance = 0.001

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_queue = 0

# functions used in state machine

class Callbacks:
    def state(self, state):
        setUpVariables.cmd_state = state.data
    def fb_move_queue(self, feedback):
        setUpVariables.fb_move_queue = feedback.data
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
    def mainRun(self):
        # publish feedback 0
        fb_set_up_publisher.publish(0)
        rospy.sleep(setUpConstants.sleeptime)
    def next(self):
        if(setUpVariables.cmd_state == 2):
            return SetUpMachine.activated
        else:
            return SetUpMachine.idle

class Activated(State):
    def transitionRun(self):
        rospy.loginfo("Set up: Setting up board.")
        # for now, publish done
        rospy.sleep(setUpConstants.sleeptime)
        fb_set_up_publisher.publish(1)
        rospy.sleep(setUpConstants.sleeptime)
        # to do: expand this state into a way to set up the board
    def mainRun(self):
        pass
    def next(self):
        return SetUpMachine.idle
            
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
        setUpCallbacks = Callbacks()
        setUpConstants = Constants()

        # init subscribers
        setUpCmd_state = rospy.Subscriber("/cmd_state", Int8, setUpCallbacks.state)
        setUpFb_move_queue = rospy.Subscriber("/fb_move_queue", Int8, setUpCallbacks.fb_move_queue)

        # init services
        rospy.wait_for_service('/overwrite_goals')
        rospy.wait_for_service('/add_goal')
        setUpOverwriteGoal = rospy.ServiceProxy('/overwrite_goals', SendGoal)
        setUpAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        SetUpMachine.idle = Idle()
        SetUpMachine.activated = Activated()
        SetUpMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
