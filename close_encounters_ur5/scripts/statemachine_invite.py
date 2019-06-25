#!/usr/bin/env python
import sys
import rospy
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.srv import SetVisionMode, SetVisionModeRequest, SetVisionModeResponse
from close_encounters_ur5.msg import AnglesList
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic is published by the statemachine_control.py

When a person is detected, the bot will invite them to come closer and play
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # for debugging, time between detecting a face and continuing with the next state
    debugtime = 3
    # movement values
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    tolerance = 0.001


class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 0
    # a variable to keep track of which move the move queue is doing
    fb_move_executor = 0
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track of how far away the face is
    distance_to_face = 0
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    # last known face position's joint values
    face_joint_angles = AnglesList()
    face_joint_angles.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573]

# functions used in state machine

class Callbacks:
    def state(self, state):
        inviteVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        inviteVariables.fb_move_executor = feedback.data
    def distance_to_face(self, distance):
        inviteVariables.distance_to_face = distance.data
    def face_angles_update(self, angles):
        inviteVariables.face_joint_angles.angles = angles.angles

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = inviteIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class InviteMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Not active.")
    def mainRun(self):
        # publish feedback 0 for debugging (no person within 1 meters)
        fb_invite_publisher.publish(0)
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        if(inviteVariables.cmd_state == 1):
            return InviteMachine.checkForPeople
        else:
            return InviteMachine.idle

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Checking for people.")
        request = SendGoalRequest()
        request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = inviteVariables.face_joint_angles.angles, 0, inviteConstants.general_max_speed, inviteConstants.general_max_acceleration, inviteConstants.tolerance, 0.01
        inviteOverwriteGoal(request)
    def mainRun(self):
        rospy.sleep(inviteConstants.debugtime)
        # check for people executed here
        inviteVariables.vision_request.mode = 1
        inviteVisionChecks(inviteVariables.vision_request)
        rospy.sleep(inviteConstants.sleeptime)
        self.distance = inviteVariables.distance_to_face
        if self.distance == 2:
            # person within 2 meters
            fb_invite_publisher.publish(0)
        elif self.distance == 1:
            # person within 1 meter
            fb_invite_publisher.publish(1)
        else:
            # no person within 2 meters
            fb_invite_publisher.publish(2)
        inviteVariables.vision_request.mode = 0
        inviteVisionChecks(inviteVariables.vision_request)
    def next(self):
        if self.distance == 2:
            return InviteMachine.inviteByLooking
            # to do: add more possibilities and selecting them at random
        else:
            rospy.sleep(inviteConstants.sleeptime)
            return InviteMachine.idle

class InviteByLooking(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Inviting person by at looking and shifting between cup and person.")
    def mainRun(self):
        # to do: add the invite by looking move here
        rospy.sleep(inviteConstants.debugtime) # for debugging, wait a bit instead of moving
    def next(self):
        return InviteMachine.checkForPeople

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_invite_node', anonymous=True)
        rospy.loginfo("Invite: Node starting.")

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_invite publisher to give feedback to the main control
        fb_invite_publisher = rospy.Publisher('/fb_invite', Int8, queue_size=1)

        inviteVariables = Variables()
        inviteCallbacks = Callbacks()
        inviteFunctions = Functions()
        inviteConstants = Constants()

        # init ini reading/writing
        inviteIniHandler = ConfigParser()
        inviteIniPath = rospy.get_param('~invite_path')
        rospy.loginfo("Invite: Using file: " + inviteIniPath)
        inviteIniHandler.read(inviteIniPath)

        # init subscribers
        inviteCmd_state = rospy.Subscriber("/cmd_state", Int8, inviteCallbacks.state)
        inviteFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, inviteCallbacks.fb_move_executor)
        inviteDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, inviteCallbacks.distance_to_face)
        inviteFace_joint_angles = rospy.Subscriber("/face_joint_angles", AnglesList, inviteCallbacks.face_angles_update)
        
        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        inviteOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        inviteAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        inviteVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        InviteMachine.idle = Idle()
        InviteMachine.checkForPeople = CheckForPeople()
        InviteMachine.inviteByLooking = InviteByLooking()
        InviteMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
