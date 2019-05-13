#!/usr/bin/env python
import sys
import rospy
import time
#import moveit_commander # moveit stuff
#import moveit_msgs # moveit stuff
#import geometry_msgs.msg
#import gripper_command # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from random import randint
from std_msgs.msg import Int8

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

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track if the bot won or lost
    won = False
    # a variable to keep track if cheating was a success
    success = False
    # a variable to keep track if the wiggle is complete
    fb_happy_reaction = False
    # a variable to keep track of how far away the face is
    distance_to_face = 0

# functions used in state machine

class Callbacks:
    def state(self, state):
        reactVariables.cmd_state = state.data
    def happy(self, happy):
        reactVariables.fb_happy_reaction = happy.data
    def distance_to_face(self, distance):
        reactVariables.distance_to_face = distance.data
    def score(self, won):
        if won == 0:
            reactVariables.won = False
        else:
            reactVariables.won = True

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
        # publish state 0
        cmd_react_publisher.publish(0)
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
    def mainRun(self):
        # publish state 1
        cmd_react_publisher.publish(1)
        # check for people
        reactVariables.person_detected = False
        '''
        while not variables.fb_check_for_people_done == 1:
            if variables.distance_to_face > 0:
                variables.person_detected = True
        '''
        if reactVariables.distance_to_face > 0:
            reactVariables.person_detected = True
        '''
        # for debugging, y=person n=no_person
        rospy.loginfo("React: Is there a person within 2 meters? (y/n) (only for debugging)")
        tmp_input = raw_input()
        if tmp_input == 'y':
            variables.person_detected = True
        elif tmp_input == 'n':
            variables.person_detected = False
            rospy.loginfo("React: No person within 2 meters anymore.")
        else:
            rospy.logwarn("React: The input wasn't y or n. Taking that as a no.")
        '''
        '''
        # check if the bot won
        # for debugging, y=bot_won n=bot_lost
        rospy.logwarn("React: Did the bot win? (y/n) (only for debugging)")
        tmp_input = raw_input()
        if tmp_input == 'y':
            variables.won = True
        elif tmp_input == 'n':
            variables.won = False
        else:
            rospy.logwarn("React: The input wasn't y or n. Taking that as a no.")
        '''
    def next(self):
        if reactVariables.person_detected == False:
            return ReactMachine.reactDissappointed
        elif reactVariables.won == True:
            return ReactMachine.reactHappy
        else:
            return ReactMachine.reactSad

class ReactDissappointed(State):
    def transitionRun(self):
        rospy.loginfo("React: Reacting disappointed.")
    def mainRun(self):
        # do disappointed move here
        # publish state 2
        cmd_react_publisher.publish(2)
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
    def mainRun(self):
        # publish state 3 (do happy move in function_emotional_happy_wiggle)
        cmd_react_publisher.publish(3)
        rospy.sleep(reactConstants.sleeptime)
    def next(self):
        if reactVariables.fb_happy_reaction == True:
            # publish done with reaction move
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
    def mainRun(self):
        # publish state 4
        cmd_react_publisher.publish(4)
        rospy.sleep(reactConstants.sleeptime)
        # do sad move here
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
    def mainRun(self):
        # publish state 5
        cmd_react_publisher.publish(5)
        rospy.sleep(reactConstants.sleeptime)
        # do cheating moves here
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

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /cmd_react publisher
        cmd_react_publisher = rospy.Publisher('/cmd_react', Int8, queue_size=1)

        # init /fb_react publisher to give feedback to the main control
        fb_react_publisher = rospy.Publisher('/fb_react', Int8, queue_size=1)

        reactVariables = Variables()
        reactCallbacks = Callbacks()
        reactConstants = Constants()

        # init subscribers
        reactCmd_state = rospy.Subscriber("/cmd_state", Int8, reactCallbacks.state)
        reactFb_happy_reaction = rospy.Subscriber("/fb_happy_reaction", Int8, reactCallbacks.happy)
        reactDistance_to_face = rospy.Subscriber("/vision_face_d", Int8, reactCallbacks.distance_to_face)
        reactFinal_score = rospy.Subscriber("/final_score", Int8, reactCallbacks.score)
        
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
