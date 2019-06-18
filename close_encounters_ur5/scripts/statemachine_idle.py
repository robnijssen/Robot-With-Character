#!/usr/bin/env python
import sys
import rospy
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8
from close_encounters_ur5.srv import GetJointValues, GetJointValuesRequest, GetJointValuesResponse
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.srv import SetVisionMode, SetVisionModeRequest, SetVisionModeResponse
from close_encounters_ur5.msg import AnglesList
from ConfigParser import ConfigParser # ini file reading/writing

"""
This stays in idle, till it's commanded to do something by the /cmd_state.
This topic is published by the statemachine_control.py

As long as nobody is close to play with, it will try to entertain itself.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.1
    # for debugging, play alone delay time
    debugtime = 5
    # max speed/acceleration
    general_max_speed = 1.0
    general_max_acceleration = 1.0
    # tolerance in joints
    tolerance = 0.0001

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # a variable to keep track if a person is detected
    person_detected = False
    # feedback if the check for people move is done
    fb_check_for_people_done = 0
    # a variable to keep track of the distance to face topic
    distance_to_face = -1
    # feedback from the move queue
    fb_move_executor = 0
    # last known face position's joint values
    face_joint_angles = AnglesList()
    face_joint_angles.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, 1.4817, -1.5051539579974573]
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    
# functions used in state machine

class Callbacks:
    def state(self, state):
        idleVariables.cmd_state = state.data
    def fb_check_for_people_done(self, fb_done):
        idleVariables.fb_check_for_people_done = fb_done.data
    def fb_move_executor(self, feedback):
        idleVariables.fb_move_executor = feedback.data
    def face_angles_update(self, angles):
        idleVariables.face_joint_angles.angles = angles.angles
    def distance_to_face(self, distance):
        idleVariables.distance_to_face = distance.data

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = idleIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list

# state machine

class IdleMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Not active.")
        rospy.sleep(idleConstants.sleeptime)
        # publish feedback 0
        fb_idle_publisher.publish(0)
    def mainRun(self):
        pass
    def next(self):
        if(idleVariables.cmd_state == 0):
            return IdleMachine.checkForPeople
        else:
            return IdleMachine.idle

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Checking for people.")
        # reset this variable
        idleVariables.person_detected = False
        # tell the vision node to check for faces
        idleVariables.vision_request.mode = 1
        idleVisionChecks(idleVariables.vision_request)
        # send to move queue
        request = SendGoalRequest()
        request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = idleVariables.face_joint_angles.angles, 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
        idleOverwriteGoal(request)
        request.goal = idleFunctions.read_from_ini('check_for_people_joint', '1')
        idleAddGoal(request)
        request.goal = idleFunctions.read_from_ini('check_for_people_joint', '2')
        idleAddGoal(request)
        request.goal = idleFunctions.read_from_ini('check_for_people_joint', '3')
        idleAddGoal(request)
        request.goal = idleFunctions.read_from_ini('check_for_people_joint', '1')
        idleAddGoal(request)
    def mainRun(self):
        if idleVariables.distance_to_face > 0:
            idleVariables.person_detected = True 
            # tell the vision node to stop checking for faces
            idleVariables.vision_request.mode = 0
            idleVisionChecks(idleVariables.vision_request)
            # go to face position
            request = SendGoalRequest()
            request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = idleVariables.face_joint_angles.angles, 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            idleOverwriteGoal(request)
            fb_idle_publisher.publish(1)
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.person_detected == False:
            if idleVariables.fb_move_executor < 5:
                # keep going till the fifth move is complete
                return IdleMachine.checkForPeople
            elif idleVariables.distance_to_face == 0:
                return IdleMachine.followPeople
            else:
                # to do: add more possibilities
                #random_result = randint(0,1)
                #if random_result == 0:
                #   ...
                return IdleMachine.playAlone
        else:
            return IdleMachine.idle

class FollowPeople(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Following people for a bit")
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.distance_to_face > 0:
            fb_idle_publisher.publish(1)
            rospy.sleep(idleConstants.sleeptime)
            return IdleMachine.idle
        elif idleVariables.distance_to_face == 0:
            return IdleMachine.followPeople
        else:
            return IdleMachine.checkForPeople

class PlayAlone(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Playing by myself for a bit")
    def mainRun(self):
        # to do: add the playing by myself move here
        # wait for a bit
        rospy.sleep(idleConstants.debugtime)
    def next(self):
        return IdleMachine.checkForPeople

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_idle_node', anonymous=True)
        rospy.loginfo("Idle: Node starting.")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_idle publisher to give feedback to the main control
        fb_idle_publisher = rospy.Publisher('/fb_idle', Int8, queue_size=1)

        idleVariables = Variables()
        idleCallbacks = Callbacks()
        idleFunctions = Functions()
        idleConstants = Constants()

        # init ini reading/writing
        idleIniHandler = ConfigParser()
        idleIniPath = rospy.get_param('~idle_path')
        rospy.loginfo("Idle: Using file: " + idleIniPath)
        idleIniHandler.read(idleIniPath)

        # init subscribers
        idleCmd_state = rospy.Subscriber("/cmd_state", Int8, idleCallbacks.state)
        idleFb_move_executor = rospy.Subscriber("/fb_move_executor", Int8, idleCallbacks.fb_move_executor)
        idleDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, idleCallbacks.distance_to_face)
        idleFace_joint_angles = rospy.Subscriber("/face_joint_angles", AnglesList, idleCallbacks.face_angles_update)
        idleFb_check_for_people = rospy.Subscriber("/fb_check_for_people", Int8, idleCallbacks.fb_check_for_people_done)

        # init services
        rospy.wait_for_service('/overwrite_goal')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        idleOverwriteGoal = rospy.ServiceProxy('/overwrite_goal', SendGoal)
        idleAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        idleVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        IdleMachine.idle = Idle()
        IdleMachine.checkForPeople = CheckForPeople()
        IdleMachine.followPeople = FollowPeople()
        IdleMachine.playAlone = PlayAlone()
        IdleMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
