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
    face_position = PositionList()
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
    def face_position_update(self, position):
        idleVariables.face_position.angles = position.angles
        idleVariables.face_position.pose = position.pose
    def distance_to_face(self, coordinates):
        idleVariables.distance_to_face = coordinates.d

class Functions:
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = idleIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list
    def random_idle_move(self):
        random_result = randint(0, 2)
        if random_result == 0:
            return IdleMachine.curiousAboutCup

# state machine

class IdleMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Not active.")
        # publish feedback 0
        fb_idle_publisher.publish(0)
        rospy.sleep(idleConstants.sleeptime)
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if(idleVariables.cmd_state == 0):
            rospy.sleep(idleConstants.sleeptime)
            return IdleMachine.lookStraightAhead
        else:
            return IdleMachine.idle

class LookStraightAhead(State):
    def transitionRun(self):
        rospy.loginfo("Idle: Moving to look straight ahead.")
        request = SendGoalRequest()
        request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = idleVariables.face_position.angles, 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
        idleOverwriteGoal(request)
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.fb_move_executor != 1:
            return IdleMachine.lookStraightAhead
        else:
            return IdleMachine.checkForPeople

class CheckForPeople(State):
    def transitionRun(self):
        # prepare a request
        request = SendGoalRequest()
        request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = idleVariables.face_position.angles, 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
        # determine which check to pick randomly
        # note that the first few are disabled in randint(3, 5)
        random_result = randint(3, 5)
        if random_result == 0:
            rospy.loginfo("Idle: Doing check_for_people_0.")
            self.number_of_moves = 2
            self.number_of_vision_suitable_moves = 2
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(1, 6):
                request.goal = idleFunctions.read_from_ini('check_for_people_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 1:
            rospy.loginfo("Idle: Doing check_for_people_1.")
            self.number_of_moves = 2
            self.number_of_vision_suitable_moves = 2
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(1, 6):
                request.goal = idleFunctions.read_from_ini('check_for_people_1_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 2:
            rospy.loginfo("Idle: Doing check_for_people_2.")
            self.number_of_moves = 2
            self.number_of_vision_suitable_moves = 2
            idleOverwriteGoal(request)
            request.type = 0
            for i in range(1, 7):
                request.goal = idleFunctions.read_from_ini('check_for_people_2_joint', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 3:
            rospy.loginfo("Idle: Doing check_for_people_3.")
            self.number_of_moves = 9
            self.number_of_vision_suitable_moves = 5
            idleOverwriteGoal(request)
            request.type = 0
            for i in range(1, 9):
                request.goal = idleFunctions.read_from_ini('check_for_people_3_joint', str(i))
                idleAddGoal(request)
        elif random_result == 4:
            rospy.loginfo("Idle: Doing check_for_people_4.")
            self.number_of_moves = 5
            self.number_of_vision_suitable_moves = 1
            idleOverwriteGoal(request)
            request.type = 0
            for i in range(1, 4):
                request.goal = idleFunctions.read_from_ini('check_for_people_4_joint', str(i))
                idleAddGoal(request)
            request.type = 2
            for i in range(4, 10):
                request.goal = idleFunctions.read_from_ini('check_for_people_4_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        else:
            rospy.loginfo("Idle: Doing check_for_people_5.")
            self.number_of_moves = 10
            self.number_of_vision_suitable_moves = 4
            idleOverwriteGoal(request)
            request.type = 0
            for i in range(1, 10):
                request.goal = idleFunctions.read_from_ini('check_for_people_5_joint', str(i))
                idleAddGoal(request)
        # reset this variable
        idleVariables.person_detected = False
        # tell the vision node to check for faces
        idleVariables.vision_request.mode = 1
        idleVisionChecks(idleVariables.vision_request)
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
        if idleVariables.fb_move_executor > self.number_of_vision_suitable_moves:
            # stop checking for a face
            idleVariables.vision_request.mode = 0
            idleVisionChecks(idleVariables.vision_request)
        elif idleVariables.distance_to_face > 0:
            # stop and go to face position
            idleVariables.person_detected = True 
            request = SendGoalRequest()
            request.goal, request.type, request.speed, request.acceleration, request.tolerance, request.delay = idleVariables.face_position.angles, 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            idleOverwriteGoal(request)
    def next(self):
        if idleVariables.person_detected == False:
            if idleVariables.fb_move_executor != self.number_of_moves:
                # not done and nobody near --> keep checking
                return IdleMachine.checkForPeople
            else:
                # tell the vision node to stop checking for faces
                idleVariables.vision_request.mode = 0
                idleVisionChecks(idleVariables.vision_request)
                # go to this state to determine what move to do
                return IdleMachine.randomSetOfMoves
        else:
            # face detected --> tell the vision node to stop checking for faces
            idleVariables.vision_request.mode = 0
            idleVisionChecks(idleVariables.vision_request)
            fb_idle_publisher.publish(1)
            rospy.sleep(idleConstants.sleeptime * 2)
            return IdleMachine.idle

class RandomSetOfMoves(State):
    def transitionRun(self):
        request = SendGoalRequest()
        request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
        # determine which movement to execute
        random_result = randint(0, 8)
        if random_result == 0:
            rospy.loginfo("Idle: Doing curious_for_playing_field.")
            self.number_of_moves = 2
            request.goal = idleFunctions.read_from_ini('curious_for_playing_field_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 8):
                request.goal = idleFunctions.read_from_ini('curious_for_playing_field_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 1:
            rospy.loginfo("Idle: Doing curious_for_playing_field_1.")
            self.number_of_moves = 2
            request.goal = idleFunctions.read_from_ini('curious_for_playing_field_1_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 4):
                request.goal = idleFunctions.read_from_ini('curious_for_playing_field_1_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 2:
            rospy.loginfo("Idle: Doing curious_for_playing_field_2.")
            self.number_of_moves = 2
            request.goal = idleFunctions.read_from_ini('curious_for_playing_field_2_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 9):
                request.goal = idleFunctions.read_from_ini('curious_for_playing_field_2_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 3:
            rospy.loginfo("Idle: Doing curious_for_cup.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('curious_for_cup_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 8):
                request.goal = idleFunctions.read_from_ini('curious_for_cup_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 4:
            rospy.loginfo("Idle: Doing curious_for_cup_1.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('curious_for_cup_1_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 5):
                request.goal = idleFunctions.read_from_ini('curious_for_cup_1_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 5:
            rospy.loginfo("Idle: Doing curious_for_cup_2.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('curious_for_cup_2_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 7):
                request.goal = idleFunctions.read_from_ini('curious_for_cup_2_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 6:
            rospy.loginfo("Idle: Doing bored_1.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('bored_1_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 6):
                request.goal = idleFunctions.read_from_ini('bored_1_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        elif random_result == 7:
            rospy.loginfo("Idle: Doing bored_3.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('bored_3_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 7):
                request.goal = idleFunctions.read_from_ini('bored_3_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
        else:
            rospy.loginfo("Idle: Doing flip_cup.")
            self.number_of_moves = 2
            request = SendGoalRequest()
            request.type, request.speed, request.acceleration, request.tolerance, request.delay = 0, idleConstants.general_max_speed, idleConstants.general_max_acceleration, idleConstants.tolerance, 0.01
            request.goal = idleFunctions.read_from_ini('flip_cup_joint', '1')
            idleOverwriteGoal(request)
            request.type = 2
            for i in range(2, 9):
                request.goal = idleFunctions.read_from_ini('flip_cup_pose', str(i))
                idleAddGoal(request)
            request.goal = []
            idleAddGoal(request)
    def mainRun(self):
        rospy.sleep(idleConstants.sleeptime)
    def next(self):
        if idleVariables.fb_move_executor != self.number_of_moves:
            return IdleMachine.randomSetOfMoves
        else:
            return IdleMachine.lookStraightAhead

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
        idleDistance_to_face = rospy.Subscriber("/vision_face_coordinates", FaceCoordinates, idleCallbacks.distance_to_face)
        idleFace_position = rospy.Subscriber("/face_position", PositionList, idleCallbacks.face_position_update)
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
        IdleMachine.lookStraightAhead = LookStraightAhead()
        IdleMachine.checkForPeople = CheckForPeople()
        IdleMachine.randomSetOfMoves = RandomSetOfMoves()
        IdleMachine().runAll()

    except rospy.ROSInterruptException:
        pass
