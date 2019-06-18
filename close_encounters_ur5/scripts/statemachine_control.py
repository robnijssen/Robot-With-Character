#!/usr/bin/env python
import sys
import rospy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from std_msgs.msg import Int8

"""
This is the control program that checks which node should be running.

The program waits for a person to come within 2 meters (or so),
then it'll try to invite the person to a game of dice.
If the person comes within 1 meter, it'll try to play a game with the person.
When done playing, it'll react on the outcome.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3

class Variables:
    # variables to keep track of the feedback
    idle_feedback = 0
    invite_feedback = 0
    setUp_feedback = 0
    takeTurn_feedback = 0
    waitForTurn_feedback = 0
    react_feedback = 0
    # a variable to keep track of the turns
    turn_number = 0
    # score variables
    player_score = 0
    bot_score = 1       # bot_score must be updated via topics later. for now, this makes sure the bot will count 2 wins in a row and move on to react
    player_total_score = 0
    bot_total_score = 0
    final_score = 0     # 0 = bot won, 1 = bot lost

# functions used in state machine

class Callbacks:
    def idle(self, feedback):
        controlVariables.idle_feedback = feedback.data
    def invite(self, feedback):
        controlVariables.invite_feedback = feedback.data
    def setUp(self, feedback):
        controlVariables.setUp_feedback = feedback.data
    def takeTurn(self, feedback):
        controlVariables.takeTurn_feedback = feedback.data
    def waitForTurn(self, feedback):
        controlVariables.waitForTurn_feedback = feedback.data
    def react(self, feedback):
        controlVariables.react_feedback = feedback.data

# state machine

class MainControlMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: Idle state should be running now.")
    def mainRun(self):
        # publish state 0
        cmd_state_publisher.publish(0)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.idle_feedback == 0):
            # still no person detected within 2 meters
            return MainControlMachine.idle
        elif(controlVariables.idle_feedback == 1):
            # person detected within 2 meters
            return MainControlMachine.invite
        else:
            rospy.logerr("CONTROL: Transition from state idle not found.")
            return MainControlMachine.idle

class Invite(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: Invite people state should be running now.")
    def mainRun(self):
        # publish state 1
        cmd_state_publisher.publish(1)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.invite_feedback == 0):
            # person still detected within 2 meters
            return MainControlMachine.invite
        elif(controlVariables.invite_feedback == 1):
            # person detected within 1 meter
            return MainControlMachine.setUp
        elif(controlVariables.invite_feedback == 2):
            # no person detected within 2 meters
            return MainControlMachine.idle
        else:
            rospy.logerr("CONTROL: Transition from state invite not found.")
            return MainControlMachine.invite

class SetUp(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: Set up board state should be running now.")
        if controlVariables.turn_number % 2 == 0:
            # end of a round (a round is 2 turns; one for the bot and one for the player)
            if controlVariables.player_score > controlVariables.bot_score:
                # player won this round
                controlVariables.player_total_score += 1
            elif controlVariables.player_score < controlVariables.bot_score:
                # bot won this round
                controlVariables.bot_total_score += 1
            else:
                # tie
                # to do: implement tie and replaying the round
                # for now, the bot will continue without counting this round
                pass
    def mainRun(self):
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if (controlVariables.player_total_score == 2) or (controlVariables.bot_total_score == 2):
            # player or bot won
            # reset turn number and move on to react
            controlVariables.turn_number = 0
            return MainControlMachine.react
        elif controlVariables.setUp_feedback == 0:
            # no winner yet and not done with set up; publish state 2 to set up the board
            cmd_state_publisher.publish(2)
            rospy.sleep(controlConstants.sleeptime)
            return MainControlMachine.setUp
        elif controlVariables.setUp_feedback == 1:
            # add one to the turn number before checking
            controlVariables.turn_number += 1
            if controlVariables.turn_number % 2 != 0:
                # uneven turn_number --> player's turn
                return MainControlMachine.waitForTurn
            elif controlVariables.turn_number % 2 == 0:
                # even turn_number --> bot's turn
                return MainControlMachine.takeTurn
        elif controlVariables.setUp_feedback == 2:
            # player walked away
            return MainControlMachine.idle
        else:
            rospy.logerr("CONTROL: Transition from state set up not found.")
            return MainControlMachine.setUp

class TakeTurn(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: Take my turn state should be running now.")
    def mainRun(self):
        # publish state 3
        cmd_state_publisher.publish(3)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.takeTurn_feedback == 0):
            # not done taking turn
            return MainControlMachine.takeTurn
        elif(controlVariables.takeTurn_feedback == 1):
            # done taking turn
            return MainControlMachine.setUp
        else:
            rospy.logerr("CONTROL: Transition from state take turn not found.")
            return MainControlMachine.takeTurn

class WaitForTurn(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: Wait for player to take a turn state should be running now.")
    def mainRun(self):
        # publish state 4
        cmd_state_publisher.publish(4)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.waitForTurn_feedback == 0):
            # (player's turn isn't finished) and (person within 2 meters)
            return MainControlMachine.waitForTurn
        elif(controlVariables.waitForTurn_feedback == 1):
            # no person within 2 meters
            return MainControlMachine.setUp
        elif(controlVariables.waitForTurn_feedback == 2):
            # (player's turn finished) and (person within 2 meters)
            return MainControlMachine.setUp
        else:
            rospy.logerr("CONTROL: Transition from state wait for turn not found.")
            return MainControlMachine.waitForTurn

class React(State):
    def transitionRun(self):
        rospy.loginfo("CONTROL: React on outcome should be running now.")
    def mainRun(self):
        # publish depending on final score
        cmd_state_publisher.publish(5 + controlVariables.final_score)
        rospy.sleep(controlConstants.sleeptime)
    def next(self):
        if(controlVariables.react_feedback == 0):
            # not done reacting
            return MainControlMachine.react
        elif(controlVariables.react_feedback == 1):
            # (done reacting) and (person within 2 meter)
            return MainControlMachine.invite
        elif(controlVariables.react_feedback == 2):
            # (done reacting) and (no person within 2 meter)
            return MainControlMachine.idle
        else:
            rospy.logerr("CONTROL: Transition from state react not found.")
            return MainControlMachine.react

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_control_node', anonymous=True)
        # wait 10 seconds to be sure everything started before commands are given
        rospy.sleep(10)
        rospy.loginfo("CONTROL: Node starting.")

        # start the publisher for the gripper command
        gripper_publisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        # activate gripper
        gripper_command.rACT = 1
        gripper_command.rGTO = 1
        gripper_command.rSP  = 50
        gripper_command.rFR  = 0
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        gripper_command.rACT = 0
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        gripper_command.rACT = 1
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        # open the gripper
        gripper_command.rPR = 0
        gripper_publisher.publish(gripper_command)
        rospy.sleep(0.2)
        rospy.loginfo("CONTROL: Gripper activated.")

        # init /cmd_state publisher
        cmd_state_publisher = rospy.Publisher('/cmd_state', Int8, queue_size=1)

        controlVariables = Variables()
        controlConstants = Constants()
        controlCallbacks = Callbacks()

        # init subscribers
        fb_idle = rospy.Subscriber("/fb_idle", Int8, controlCallbacks.idle)
        fb_invite = rospy.Subscriber("/fb_invite", Int8, controlCallbacks.invite)
        fb_set_up = rospy.Subscriber("/fb_set_up", Int8, controlCallbacks.setUp)
        fb_take_turn = rospy.Subscriber("/fb_take_turn", Int8, controlCallbacks.takeTurn)
        fb_wait_for_turn = rospy.Subscriber("/fb_wait_for_turn", Int8, controlCallbacks.waitForTurn)
        fb_react = rospy.Subscriber("/fb_react", Int8, controlCallbacks.react)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        MainControlMachine.idle = Idle()
        MainControlMachine.invite = Invite()
        MainControlMachine.setUp = SetUp()
        MainControlMachine.takeTurn = TakeTurn()
        MainControlMachine.waitForTurn = WaitForTurn()
        MainControlMachine.react = React()
        MainControlMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
