#!/usr/bin/env python
import sys
import rospy
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8
# import service types
from close_encounters_ur5.srv import GetJointValues, GetJointValuesRequest, GetJointValuesResponse
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.srv import SetVisionMode, SetVisionModeRequest, SetVisionModeResponse
from close_encounters_ur5.msg import AnglesList

"""
This stays in idle, till it's commanded to do something by the /cmd_state
This topic published on by the statemachine_control.py

After playing a game, it will try to show emotions.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.1
    # for debugging, delay time in empty reaction state
    debugtime = 3
    # max speed&acceleration
    general_max_speed = 0.1
    general_max_acceleration = 0.1
    happy_wiggle_max_speed = 0.8
    happy_wiggle_max_acceleration = 0.8
    # tolerance in joints
    tolerance = 0.001
    # happy wiggle movement distance multipliers
    happy_wiggle_global_multiplier = 0.4
    happy_wiggle_x_multiplier_0 = 0.4
    happy_wiggle_x_multiplier_4 = 0.6
    # happy wiggle boundaries to be sure it won't move too far
    happy_wiggle_left_boundary_0 = -0.455052677785055
    happy_wiggle_left_boundary_4 = -6.210995327924387
    happy_wiggle_right_boundary_0 = -3.9625261465655726
    happy_wiggle_right_boundary_4 = -3.5705881754504603

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # feedback from the move queue
    fb_move_queue = 0
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track if the bot won or lost
    won = False
    # a variable to keep track if cheating was a success
    success = False
    # prepare request for vision node
    vision_request = SetVisionModeRequest()
    # a variable to keep track of how far away the face is
    distance_to_face = 0
    # last known face position's joint values
    face_joint_angles = AnglesList()
    face_joint_angles.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -3.170588795338766]
    # happy wiggle values
    happy_wiggle_default = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -3.170588795338766]
    happy_wiggle_start = list(happy_wiggle_default)
    happy_wiggle_left = list(happy_wiggle_default)
    happy_wiggle_right = list(happy_wiggle_default)

# functions used in state machine

class Functions:
    def happy_determine_wiggle_values(self):
        # calculate corrections for pitch and yaw approximation from the face position
        correction_0 = reactVariables.happy_wiggle_start[0] - reactVariables.face_joint_angles.angles[0]
        correction_1 = reactVariables.happy_wiggle_start[1] - reactVariables.face_joint_angles.angles[1]
        correction_2 = reactVariables.happy_wiggle_start[2] - reactVariables.face_joint_angles.angles[2]
        correction_3 = reactVariables.happy_wiggle_start[3] - reactVariables.face_joint_angles.angles[3]
        correction_4 = reactVariables.happy_wiggle_start[4] - reactVariables.face_joint_angles.angles[4]
        # reset the start values with default values
        reactVariables.happy_wiggle_start = list(reactVariables.happy_wiggle_default)
        # set approximated pitch and yaw of the center over default values
        reactVariables.happy_wiggle_start[1] -= correction_1 + correction_2 - correction_3
        reactVariables.happy_wiggle_start[0] -= correction_4 + correction_0
        # copy values from the start to left and right
        reactVariables.happy_wiggle_left = list(reactVariables.happy_wiggle_start)
        reactVariables.happy_wiggle_right = list(reactVariables.happy_wiggle_start)
        # alter some yaw values to produce left and right
        reactVariables.happy_wiggle_left[0] += reactConstants.happy_wiggle_x_multiplier_0 * reactConstants.happy_wiggle_global_multiplier
        reactVariables.happy_wiggle_left[4] -= reactConstants.happy_wiggle_x_multiplier_4 * reactConstants.happy_wiggle_global_multiplier
        reactVariables.happy_wiggle_right[0] -= reactConstants.happy_wiggle_x_multiplier_0 * reactConstants.happy_wiggle_global_multiplier
        reactVariables.happy_wiggle_right[4] += reactConstants.happy_wiggle_x_multiplier_4 * reactConstants.happy_wiggle_global_multiplier
        # make sure the yaw won't go out of bounds
        if reactVariables.happy_wiggle_left[0] > reactConstants.happy_wiggle_left_boundary_0:
            rospy.logwarn("React: HappyWiggle: left_joint_values[0] tried to go out of bounds.")
            reactVariables.happy_wiggle_left[0] = reactConstants.happy_wiggle_left_boundary_0
        if reactVariables.happy_wiggle_left[4] < reactConstants.happy_wiggle_left_boundary_4:
            rospy.logwarn("React: HappyWiggle: left_joint_values[4] tried to go out of bounds.")
            reactVariables.happy_wiggle_left[4] = reactConstants.happy_wiggle_left_boundary_4
        if reactVariables.happy_wiggle_right[0] < reactConstants.happy_wiggle_right_boundary_0:
            rospy.logwarn("React: HappyWiggle: right_joint_values[0] tried to go out of bounds.")
            reactVariables.happy_wiggle_right[0] = reactConstants.happy_wiggle_right_boundary_0
        if reactVariables.happy_wiggle_right[4] > reactConstants.happy_wiggle_right_boundary_4:
            rospy.logwarn("React: HappyWiggle: right_joint_values[4] tried to go out of bounds.")
            reactVariables.happy_wiggle_right[4] = reactConstants.happy_wiggle_right_boundary_4

class Callbacks:
    def state(self, state):
        reactVariables.cmd_state = state.data
    def fb_move_queue(self, feedback):
        reactVariables.fb_move_queue = feedback.data
    def distance_to_face(self, distance):
        reactVariables.distance_to_face = distance.data
    def score(self, won):
        if won == 0:
            reactVariables.won = False
        else:
            reactVariables.won = True
    def face_angles_update(self, angles):
        reactVariables.face_joint_angles.angles = angles.angles

# state machine

class ReactMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("React: Not active.")
    def mainRun(self):
        # publish feedback 0 (not active / not done reacting)
        fb_react_publisher.publish(0)
        rospy.sleep(reactConstants.sleeptime)
    def next(self):
        if(reactVariables.cmd_state == 3):
            return ReactMachine.check
        else:
            return ReactMachine.idle

class Check(State):
    def transitionRun(self):
        rospy.loginfo("React: Checking for people.")
        # tell vision node to look for a face
        reactVariables.vision_request.mode = 1
        reactVisionChecks(reactVariables.vision_request)
        rospy.sleep(reactConstants.sleeptime)
    def mainRun(self):
        # check for people
        reactVariables.person_detected = False
        '''
        while not variables.fb_check_for_people_done == 1:
            if variables.distance_to_face > 0:
                variables.person_detected = True
        '''
        if reactVariables.distance_to_face > 0:
            reactVariables.person_detected = True
    def next(self):
        # tell vision node to stop looking for a face
        reactVariables.vision_request.mode = 0
        reactVisionChecks(reactVariables.vision_request)
        if reactVariables.person_detected == False:
            return ReactMachine.reactDissappointed
        elif reactVariables.won == True:
            return ReactMachine.reactHappy
        else:
            return ReactMachine.reactSad

class ReactDissappointed(State):
    def transitionRun(self):
        rospy.loginfo("React: Reacting disappointed.")
        # to do: send disappointed move here
    def mainRun(self):
        rospy.sleep(reactConstants.sleeptime)
    def next(self):
        # publish done with reaction move
        if reactVariables.person_detected == True:
            fb_react_publisher.publish(1)
        else:
            fb_react_publisher.publish(2)
        rospy.sleep(2 * reactConstants.sleeptime)
        return ReactMachine.idle

class ReactHappy(State):
    def transitionRun(self):
        rospy.loginfo("React: Reacting happy.")
        # to do: add more variants and a little random
        # determine how to do the wiggle
        reactFunctions.happy_determine_wiggle_values()
        # produce requests for the move queue
        request0, request1, request2, request3, request4 = SendGoalRequest(), SendGoalRequest(), SendGoalRequest(), SendGoalRequest(), SendGoalRequest()
        request0.goal, request0.speed, request0.acceleration, request0.tolerance, request0.delay = reactVariables.happy_wiggle_start, reactConstants.general_max_speed, reactConstants.general_max_acceleration, reactConstants.tolerance, 0.01
        request1.goal, request1.speed, request1.acceleration, request1.tolerance, request1.delay = reactVariables.happy_wiggle_left, reactConstants.happy_wiggle_max_speed, reactConstants.happy_wiggle_max_acceleration, reactConstants.tolerance, 0.001
        request2.goal, request2.speed, request2.acceleration, request2.tolerance, request2.delay = reactVariables.happy_wiggle_right, reactConstants.happy_wiggle_max_speed, reactConstants.happy_wiggle_max_acceleration, reactConstants.tolerance, 0.001
        request3.goal, request3.speed, request3.acceleration, request3.tolerance, request3.delay = reactVariables.happy_wiggle_left, reactConstants.happy_wiggle_max_speed, reactConstants.happy_wiggle_max_acceleration, reactConstants.tolerance, 0.001
        request4.goal, request4.speed, request4.acceleration, request4.tolerance, request4.delay = reactVariables.happy_wiggle_start, reactConstants.happy_wiggle_max_speed, reactConstants.happy_wiggle_max_acceleration, reactConstants.tolerance, 0.001
        # send request list to the move queue
        reactOverwriteGoals(request0)
        reactAddGoal(request1)
        reactAddGoal(request2)
        reactAddGoal(request3)
        reactAddGoal(request4)
    def mainRun(self):
        rospy.sleep(reactConstants.sleeptime)
    def next(self):
        if reactVariables.fb_move_queue == 5:
            # publish done with reaction move after 5th move is complete
            if reactVariables.person_detected == True:
                fb_react_publisher.publish(1)
            else:
                fb_react_publisher.publish(2)
            rospy.sleep(2 * reactConstants.sleeptime)
            return ReactMachine.idle
        else:
            return ReactMachine.reactHappy

class ReactSad(State):
    def transitionRun(self):
        rospy.loginfo("React: Reacting sad.")
        # to do: send sad move here
    def mainRun(self):
        rospy.sleep(reactConstants.sleeptime)
    def next(self):
        rospy.sleep(reactConstants.sleeptime)
        if reactVariables.person_detected == True:
            fb_react_publisher.publish(1)
        else:
            fb_react_publisher.publish(2)
        rospy.sleep(reactConstants.debugtime)
        return ReactMachine.idle

class Cheat(State):
    def transitionRun(self):
        rospy.loginfo("React: Cheating.")
        # to do: send cheating moves
    def mainRun(self):
        rospy.sleep(reactConstants.sleeptime)
        # check for success
        # for debugging, y=success n=no_success
        rospy.loginfo("React: Cheated. Was it a success? (y/n) (only for debugging)")
        tmp_input = raw_input()
        if tmp_input == 'y':
            reactVariables.success = True
        elif tmp_input == 'n':
            reactVariables.success = False
        else:
            rospy.logwarn("React: The input wasn't y or n. Taking that as a no.")
    def next(self):
        if reactVariables.success == True:
            return ReactMachine.reactHappy
        else:
            return ReactMachine.reactSad

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_react_node', anonymous=True)
        rospy.loginfo("react actions node starting")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_react publisher to give feedback to the main control
        fb_react_publisher = rospy.Publisher('/fb_react', Int8, queue_size=1)

        reactVariables = Variables()
        reactCallbacks = Callbacks()
        reactConstants = Constants()
        reactFunctions = Functions()

        # init subscribers
        reactCmd_state = rospy.Subscriber("/cmd_state", Int8, reactCallbacks.state)
        reactFb_move_queue = rospy.Subscriber("/fb_move_queue", Int8, reactCallbacks.fb_move_queue)
        reactDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, reactCallbacks.distance_to_face)
        reactFinal_score = rospy.Subscriber("/final_score", Int8, reactCallbacks.score)
        reactFace_joint_angles = rospy.Subscriber("/face_joint_angles", AnglesList, reactCallbacks.face_angles_update)

        # init services
        rospy.wait_for_service('/overwrite_goals')
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/vision_checks')
        reactOverwriteGoals = rospy.ServiceProxy('/overwrite_goals', SendGoal)
        reactAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        reactVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        ReactMachine.idle = Idle()
        ReactMachine.check = Check()
        ReactMachine.reactDissappointed = ReactDissappointed()
        ReactMachine.reactHappy = ReactHappy()
        ReactMachine.reactSad = ReactSad()
        ReactMachine.cheat = Cheat()
        ReactMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
