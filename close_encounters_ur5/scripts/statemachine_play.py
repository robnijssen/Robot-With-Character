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

When a person is very close, the bot will try playing a game with that person.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # for debugging, time per turn
    debugtime = 3
    # number of total turns (best out of 3 is max 6 turns)
    max_turns = 6

class Variables:
    # a variable to keep track of what state the control is in
    cmd_state = 1
    # a variable to keep track if the board is set up or not
    board_set_up = False
    # a variable to keep track if a person is detected
    person_detected = False
    # a variable to keep track of the turns
    turn_number = 0
    # a variable to keep track of how far away the face is
    distance_to_face = 0
    # score variables
    fb_player_score = 0
    fb_bot_score = 0
    player_total_score = 0
    bot_total_score = 0

# functions used in state machine

class Callbacks:
    def state(self, state):
        variables.cmd_state = state.data
    def distance_to_face(self, distance):
        variables.distance_to_face = distance.data
    def vision_score(self, score):
        if variables.turn_number % 2 != 0:
            variables.fb_player_score = score.data
        else:
            variables.fb_bot_score = score.data

# state machine

class PlayMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Play: Not active.")
        # reset values
        variables.turn_number = 1
        variables.player_total_score = 0
        variables.bot_total_score = 0
    def mainRun(self):
        # publish state 0
        cmd_play_publisher.publish(0)
        # publish feedback 0
        fb_play_publisher.publish(0)
        rospy.sleep(constants.sleeptime)
    def next(self):
        if(variables.cmd_state == 2):
            return PlayMachine.setUpBoard
        else:
            return PlayMachine.idle

class SetUpBoard(State):
    def transitionRun(self):
        rospy.loginfo("Play: Setting up board.")
    def mainRun(self):
        # publish command 1
        cmd_play_publisher.publish(1)
        variables.board_set_up = True
    def next(self):
        return PlayMachine.check

class Check(State):
    def transitionRun(self):
        rospy.loginfo("Play: Checking for people and/or score")
        if variables.turn_number % 2 == 0:
            if variables.fb_player_score > variables.fb_bot_score:
                # player won this round
                variables.player_total_score += 1
            elif variables.fb_player_score < variables.fb_bot_score:
                # bot won this round
                variables.bot_total_score += 1
            else:
                # tie
                # to do: implement tie and replaying the round
                # for now, this is counted as a bot win
                variables.bot_total_score += 1
    def mainRun(self):
        # do checks
        variables.person_detected = False
        '''
        while not variables.fb_check_for_people_and_score_done == 1:
            if variables.distance_to_face > 0:
                variables.person_detected = True
        '''
        if variables.distance_to_face > 0:
            variables.person_detected = True
    def next(self):
        if variables.person_detected == False:
            if variables.turn_number <= constants.max_turns:
                # person left during the game
                fb_play_publisher.publish(1)
                rospy.sleep(constants.sleeptime)
                return PlayMachine.idle
            elif variables.turn_number == constants.max_turns + 1:
                # person left after the game
                fb_play_publisher.publish(3)
                rospy.sleep(constants.sleeptime)
                return PlayMachine.idle
            else:
                fatal = "Play: Turn number can't be this high" + str(variables.turn_number)
                rospy.logfatal(fatal)
                return PlayMachine.idle
        else:
            if variables.player_total_score == 2:
                # player won
                fb_play_publisher.publish(2)
                final_score.publish(0)
                rospy.sleep(constants.sleeptime)
                return PlayMachine.idle
            elif variables.bot_total_score == 2:
                # bot won
                fb_play_publisher.publish(2)
                final_score.publish(1)
                rospy.sleep(constants.sleeptime)
                return PlayMachine.idle
            else:
                if variables.board_set_up == True:
                    if variables.turn_number % 2 != 0:
                        # uneven turn_number --> player's turn
                        return PlayMachine.waitForPlayersMove
                    else:
                        # even turn_number --> bot's turn
                        return PlayMachine.takeTurn
                else:
                    return PlayMachine.setUpBoard

class WaitForPlayersMove(State):
    def transitionRun(self):
        rospy.loginfo("Play: Waiting for player to take a turn.")
    def mainRun(self):
        # publish command 2
        cmd_play_publisher.publish(2)
        rospy.sleep(constants.sleeptime)
        rospy.sleep(constants.debugtime)
    def next(self):
        if variables.fb_player_score != 0:
            return PlayMachine.waitForPlayersMove
        else:
            variables.turn_number += 1
            return PlayMachine.check

class TakeTurn(State):
    def transitionRun(self):
        rospy.loginfo("Play: Taking my turn.")
    def mainRun(self):
        # publish command 3
        cmd_play_publisher.publish(3)
        rospy.sleep(constants.sleeptime)
        rospy.sleep(constants.debugtime)
    def next(self):
        if variables.fb_bot_score != 0:
            return PlayMachine.takeTurn
        else:
            variables.turn_number += 1
            return PlayMachine.check
            
if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_play_node', anonymous=True)
        rospy.loginfo("play actions node starting")

        # start moveit
        #moveit_commander.roscpp_initialize(sys.argv)
        #group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /cmd_play publisher
        cmd_play_publisher = rospy.Publisher('/cmd_play', Int8, queue_size=1)

        # init /fb_play publisher to give feedback to the main control
        fb_play_publisher = rospy.Publisher('/fb_play', Int8, queue_size=1)

        # init /final_score publisher to tell the react statemachine if the bot won
        final_score = rospy.Publisher('/final_score', Int8, queue_size=1)

        # init subscriber
        variables = Variables()
        callbacks = Callbacks()
        constants = Constants()
        cmd_state = rospy.Subscriber("/cmd_state", Int8, callbacks.state)
        distance_to_face = rospy.Subscriber("/vision_face_d", Int8, callbacks.distance_to_face)
        vision_score = rospy.Subscriber("/vision_score", Int8, callbacks.vision_score)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        PlayMachine.idle = Idle()
        PlayMachine.setUpBoard = SetUpBoard()
        PlayMachine.check = Check()
        PlayMachine.waitForPlayersMove = WaitForPlayersMove()
        PlayMachine.takeTurn = TakeTurn()
        PlayMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
