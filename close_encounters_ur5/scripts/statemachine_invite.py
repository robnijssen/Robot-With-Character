#!/usr/bin/env python
import sys
import rospy
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint, choice, uniform
from std_msgs.msg import Int8
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic is published by the statemachine_control.py

When a person is detected, the bot will invite them to come closer and play
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.1
    # sleep between overwriting the look at face goals
    looktime = 1.0
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
    vision_face_coordinates = FaceCoordinates()
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    # last known face position's joint values
    face_position = PositionList()
    face_position.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573]

# functions used in state machine

class Callbacks:
    def state(self, state):
        inviteVariables.cmd_state = state.data
    def fb_move_executor(self, feedback):
        inviteVariables.fb_move_executor = feedback.data
    def vision_face_coordinates(self, coordinates):
        inviteVariables.vision_face_coordinates.x = coordinates.x
        inviteVariables.vision_face_coordinates.y = coordinates.y
        inviteVariables.vision_face_coordinates.d = coordinates.d
        inviteVariables.vision_face_coordinates.e = coordinates.e
    def face_position_update(self, position):
        inviteVariables.face_position.angles = position.angles
        inviteVariables.face_position.pose = position.pose

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
        # reset feedback
        fb_invite_publisher.publish(0)
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        if inviteVariables.cmd_state == 1:
            return InviteMachine.lookAtFace
        else:
            return InviteMachine.idle

class LookAtFace(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Looking at the face.")
        # make the vision node look for people
        inviteVariables.vision_request.mode = 1
        inviteVisionChecks(inviteVariables.vision_request)
        rospy.sleep(inviteConstants.sleeptime)
        # look at face position
        self.request = SendGoalRequest()
        self.request.goal, self.request.type, self.request.tolerance = inviteVariables.face_position.angles, 0, inviteConstants.tolerance
        inviteOverwriteGoal(self.request)
        self.request.type = 1
    def mainRun(self):
        rospy.sleep(inviteConstants.looktime)
    def next(self):
        if inviteVariables.fb_move_executor != 1:
            return InviteMachine.lookAtFace
        else:
            # make the vision node stop looking for people
            inviteVariables.vision_request.mode = 0
            inviteVisionChecks(inviteVariables.vision_request)
            rospy.sleep(inviteConstants.sleeptime)
            return InviteMachine.checkForPeople
        #if inviteVariables.vision_face_coordinates.e != -1 and inviteVariables.vision_face_coordinates.e > 30:
        #    # send a new goal to look better at the face before moving on to the next state
        #    self.request.goal = inviteVariables.face_position.pose
        #    inviteOverwriteGoal(self.request)
        #    return InviteMachine.lookAtFace
        #else:
        #    # make the vision node stop looking for people
        #    inviteVariables.vision_request.mode = 0
        #    inviteVisionChecks(inviteVariables.vision_request)
        #    rospy.sleep(inviteConstants.sleeptime)
        #    return InviteMachine.checkForPeople

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Checking for people.")
        # tell vision to look for faces
        inviteVariables.vision_request.mode = 1
        inviteVisionChecks(inviteVariables.vision_request)
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        # tell vision to stop looking for faces
        inviteVariables.vision_request.mode = 0
        inviteVisionChecks(inviteVariables.vision_request)
        if inviteVariables.vision_face_coordinates.d == 2:
            # person within 2 meters (still inviting)
            return InviteMachine.inviteMovement
        elif inviteVariables.vision_face_coordinates.d == 1:
            # person within 1 meter (command can move on with set up)
            fb_invite_publisher.publish(1)
            rospy.sleep(inviteConstants.sleeptime)
        else:
            # no person within 2 meters (command has to go back to idle)
            fb_invite_publisher.publish(2)
            rospy.sleep(inviteConstants.sleeptime)
        return InviteMachine.idle

class InviteMovement(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Doing an inviting movement.")
        # close in on the face
        request = SendGoalRequest()
        request.type, request.tolerance = 2, inviteConstants.tolerance
        request.goal = inviteFunctions.read_from_ini('invite_3_pose', '1')
        inviteOverwriteGoal(request)
        for i in range(2, 6): # note: skipping the last waypoint (which is the face position)
            request.goal = inviteFunctions.read_from_ini('invite_3_pose', str(i))
            inviteAddGoal(request)
        request.goal = []
        inviteAddGoal(request)
        rospy.sleep(inviteConstants.sleeptime * 2)
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        if inviteVariables.fb_move_executor != 1:
            return InviteMachine.inviteMovement
        else:
            return InviteMachine.lookAtFaceAskingFirst

class LookAtFaceAskingFirst(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Looking at the face before doing the asking position.")
        # make the vision node look for people
        inviteVariables.vision_request.mode = 1
        inviteVisionChecks(inviteVariables.vision_request)
        rospy.sleep(inviteConstants.sleeptime)
        # go to face position
        request = SendGoalRequest()
        request.goal, request.type, request.tolerance = inviteVariables.face_position.angles, 0, inviteConstants.tolerance
        inviteOverwriteGoal(request)
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        if inviteVariables.fb_move_executor != 1:
            return InviteMachine.lookAtFaceAskingFirst
        else:
            # make the vision node stop looking for people
            inviteVariables.vision_request.mode = 0
            inviteVisionChecks(inviteVariables.vision_request)
            rospy.sleep(inviteConstants.sleeptime)
            return InviteMachine.lookAtFaceAskingSecond

class LookAtFaceAskingSecond(State):
    def transitionRun(self):
        rospy.loginfo("Invite: Rotate the end effector to emulate asking.")
        # rotate the 'head' of the robot to emulate asking
        request = SendGoalRequest()
        request.goal, request.type, request.tolerance = list(inviteVariables.face_position.angles), 0, inviteConstants.tolerance 
        # random rotation within certain bounds
        if randint(0, 1) == 0:
            request.goal[5] += float(randint(3, 10)) / 10
        else:
            request.goal[5] -= float(randint(3, 10)) / 10
        inviteOverwriteGoal(request)
        rospy.sleep(inviteConstants.sleeptime * 2)
    def mainRun(self):
        rospy.sleep(inviteConstants.sleeptime)
    def next(self):
        if inviteVariables.fb_move_executor != 1:
            return InviteMachine.lookAtFaceAskingSecond
        else:
            return InviteMachine.lookAtFace

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_invite_node', anonymous=True)
        rospy.loginfo("Invite: Node starting.")

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
        inviteDistance_to_face = rospy.Subscriber("/vision_face_coordinates", FaceCoordinates, inviteCallbacks.vision_face_coordinates)
        inviteFace_position = rospy.Subscriber("/face_position", PositionList, inviteCallbacks.face_position_update)
        
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
        InviteMachine.lookAtFace = LookAtFace()
        InviteMachine.checkForPeople = CheckForPeople()
        InviteMachine.inviteMovement = InviteMovement()
        InviteMachine.lookAtFaceAskingFirst = LookAtFaceAskingFirst()
        InviteMachine.lookAtFaceAskingSecond = LookAtFaceAskingSecond()
        InviteMachine().runAll()

    except rospy.ROSInterruptException:
        pass
