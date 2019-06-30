#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
from copy import deepcopy # makes sure a copy is made without making a lasting reference
from std_msgs.msg import Int8
from random import randint
# import tf stuff for roll pitch yaw for pose goals
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import roslib; roslib.load_manifest('robotiq_2f_gripper_control') # gripper stuff
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg # gripper stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This program is made to have only one node connect to the move group and have vision implemented.
"""

class Variables:
    # game option: amount of rounds
    #   1 --> react after every round
    #   3 --> react after best out of 3
    #   5 --> react after best out of 5
    #   ...
    #       to do: detect this with gestures?
    amount_of_rounds = 1
    winning_amount_of_wins = (amount_of_rounds / 2) + 0.5
    # variables for movements
    tolerance = 0.005
    waypoints = []
    turn_number = 0 # 0=player_turn, 1=bot_turn
    # score variables
    bot_score = 0
    bot_total_score = 0
    player_score = 0
    player_total_score = 0
    # variables updated by vision
    face_d = -1
    amount_of_dice = -1
    amount_of_pips = -1
    cup_detected = 0
    #
    face_position = 1
    face_near = False

class Callbacks:
    def vision_face(self, coordinates):
        demoVariables.face_d = coordinates.d
    def vision_amount_of_dice(self, amount):
        demoVariables.amount_of_dice = amount.data
    def vision_amount_of_pips(self, amount):
        demoVariables.amount_of_pips = amount.data
    def vision_cup(self, data):
        demoVariables.cup_detected = data.data

class Functions:
    def check_for_yes(self, check): ###
        if check == 'y' or check == 'Y' or check == 'yes' or check == 'affirmative':
            return True
        else:
            return False
    def read_from_ini(self, section_to_read, key_to_read):
        goal_string = demoIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list
    def execute_goal(self, type_to_read, section_to_read, key_to_read):
        if type_to_read == 0:
            # run as joint goal
            rospy.loginfo("Demo: Executing joint movement.")
            # read from the file
            goal = demoFunctions.read_from_ini(section_to_read, key_to_read)
            # compute a plan
            group.set_joint_value_target(goal)
            # go to the planned position
            group.go(wait=True)
            demoFunctions.stop()
        elif type_to_read == 1:
            # run as pose goal
            rospy.loginfo("Demo: Executing pose movement.")
            # read from the file
            goal = demoFunctions.read_from_ini(section_to_read, key_to_read)
            # put the values in the goal
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = goal[0], goal[1], goal[2]
            pose_goal.orientation.x, pose_goal.orientation.y = goal[3], goal[4]
            pose_goal.orientation.z, pose_goal.orientation.w = goal[5], goal[6]
            # send the completed goal
            group.set_pose_target(pose_goal)
            # go to the planned position
            group.go(wait=True)
            demoFunctions.stop()
        elif type_to_read == 2:
            # run as cartesian path
            rospy.loginfo("Demo: Executing cartesian path.")
            # add starting position to waypoints
            demoVariables.waypoints.append(deepcopy(group.get_current_pose().pose))
            # add all waypoints by reading from the ini file
            for i in range(0, int(key_to_read)):
                goal = demoFunctions.read_from_ini(section_to_read, i + 1)
                # put the values in the goal
                pose_goal = geometry_msgs.msg.Pose()
                pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = goal[0], goal[1], goal[2]
                pose_goal.orientation.x, pose_goal.orientation.y = goal[3], goal[4]
                pose_goal.orientation.z, pose_goal.orientation.w = goal[5], goal[6]
                # append the pose goal
                demoVariables.waypoints.append(pose_goal)
            # plan path with the waypoints, interpolated at every 10 cm, and the jump threshold disabled
            cartesian_plan, fraction = group.compute_cartesian_path(demoVariables.waypoints, 0.01, 0.0)
            # execute the cartesian path that was planned
            group.execute(cartesian_plan, wait=True)
            demoFunctions.stop()
        else:
            rospy.logerr("Demo: Wrong type to read entered in execute_goal.")
    def execute_goals(self, type_to_read, section_to_read, keys_to_read):
        if type_to_read == 2:
            # make sure a cartesian path isn't executed more than once if started with this function
            self.execute_goal(type_to_read, section_to_read, keys_to_read)
        else:
            for i in range(0, keys_to_read):
                self.execute_goal(type_to_read, section_to_read, i + 1)
    def stop(self):
        # make sure to stop the residual movements
        group.stop()
        group.clear_pose_targets()
        # clear the waypoints
        del demoVariables.waypoints[:]
        demoVariables.waypoints_cleared = True
    def check_number_of_dice(self):
        # go to checking position
        demoFunctions.execute_goal(0, 'check_for_dice_joint', 1)
        # tell vision to check for dice
        demoVisionChecks(2)
        # wait for vision to check
        rospy.sleep(2)
        # check the number of dice
        if demoVariables.amount_of_dice > 0:
            # tell vision to stop checking for dice
            demoVisionChecks(0)
            # dice found
            rospy.loginfo("Demo: Dice found.")
            return True
        else:
            # tell vision to stop checking for dice
            demoVisionChecks(0)
            # no dice found
            rospy.loginfo("Demo: No dice found.")
            return False
    def ask_face_angles(self, face_position):
        # read face position in joint angles
        face_angles = self.read_from_ini('check_for_people_joint', face_position)
        # move to the normal face position in using joint values
        group.set_joint_value_target(face_angles)
        group.go(wait=True)
        demoFunctions.stop()
        # change the value of joint 5 a little to emulate tilting of the 'head'
        if randint(0, 1) == 0:
            face_angles[5] += float(randint(1, 10)) / 10
        else:
            face_angles[5] -= float(randint(1, 10)) / 10
        # move to the asking face position in using joint values
        group.set_joint_value_target(face_angles)
        group.go(wait=True)
        demoFunctions.stop()

# state machine

class DemoMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class CheckForPeople(State):
    def transitionRun(self):
        rospy.loginfo("CHECK FOR PEOPLE")
        # reset score variables and turn number after a playing sequence
        demoVariables.bot_score = 0
        demoVariables.bot_total_score = 0
        demoVariables.player_score = 0
        demoVariables.player_total_score = 0
        demoVariables.turn_number = 0
    def mainRun(self):
        demoFunctions.execute_goal(0, 'check_for_people_joint', 1)
        # tell vision to look for a face
        demoVisionChecks(1)
        rospy.sleep(1)
        if demoVariables.face_d > 0:
            demoVariables.face_position = 1
        else:
            demoFunctions.execute_goal(0, 'check_for_people_joint', 2)
            if demoVariables.face_d > 0:
                demoVariables.face_position = 2
            else:
                demoFunctions.execute_goal(0, 'check_for_people_joint', 3)
                rospy.sleep(0.3)
                if demoVariables.face_d > 0:
                    demoVariables.face_position = 3
        # tell vision to stop looking for a face
        demoVisionChecks(0)
    def next(self):
        if demoVariables.face_d == 2:
            return DemoMachine.invite
        elif demoVariables.face_d == 1:
            return DemoMachine.setUp
        else:
            return DemoMachine.idle

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("IDLE")
    def mainRun(self):
        # do some idle movement
        random_result = randint(0, 7)
        if random_result == 0:
            rospy.loginfo("Demo: curious_for_playing_field")
            demoFunctions.execute_goals(2, 'curious_for_playing_field_pose', 6) # note the final movement is skipped
        elif random_result == 1:
            rospy.loginfo("Demo: curious_for_playing_field_1")
            demoFunctions.execute_goals(2, 'curious_for_playing_field_1_pose', 3)
        elif random_result == 2:
            rospy.loginfo("Demo: curious_for_playing_field_2")
            demoFunctions.execute_goals(2, 'curious_for_playing_field_2_pose', 4) # note the final few movements are skipped
        elif random_result == 3:
            rospy.loginfo("Demo: curious_for_cup")
            demoFunctions.execute_goals(2, 'curious_for_cup_pose', 7)
        elif random_result == 4:
            rospy.loginfo("Demo: curious_for_cup_1")
            demoFunctions.execute_goals(2, 'curious_for_cup_1_pose', 4)
        elif random_result == 5:
            rospy.loginfo("Demo: curious_for_cup_2")
            demoFunctions.execute_goals(2, 'curious_for_cup_2_pose', 4) # note the final few movements are skipped
        elif random_result == 6:
            rospy.loginfo("Demo: bored_3")
            demoFunctions.execute_goals(2, 'bored_3_pose', 6)
        else:
            rospy.loginfo("Demo: flip_cup")
            demoFunctions.execute_goals(2, 'flip_cup_pose', 6)
    def next(self):
        return DemoMachine.checkForPeople

class Invite(State):
    def transitionRun(self):
        rospy.loginfo("INVITE")
    def mainRun(self):
        # to do: add more inviting movements
        rospy.loginfo("Demo: invite_3")
        demoFunctions.execute_goals(2, 'invite_3_pose', 5) # note the final movement is skipped and asking is added
        demoFunctions.ask_face_angles(demoVariables.face_position)
    def next(self):
        return DemoMachine.checkForPeople

class SetUp(State):
    def transitionRun(self):
        rospy.loginfo("SET UP")
        self.check = True
    def mainRun(self):
        self.check = demoFunctions.check_number_of_dice() # True=dice_in_tray False=no_dice_in_tray
        if self.check:
            # ask for dice in the cup if there are dice in the tray
            random_result = randint(0, 2)
            if random_result == 0:
                rospy.loginfo("Demo: ask_for_dice")
                demoFunctions.execute_goals(2, 'ask_for_dice_pose', 6)
            elif random_result == 1:
                rospy.loginfo("Demo: ask_for_dice_1")
                demoFunctions.execute_goals(2, 'ask_for_dice_1_pose', 6)
            else:
                rospy.loginfo("Demo: ask_for_dice_2")
                demoFunctions.execute_goals(2, 'ask_for_dice_2_pose', 8)
    def next(self):
        if not self.check:
            if demoVariables.turn_number == 1:
                return DemoMachine.checkForCup
            else:
                return DemoMachine.waitForTurn
        else:
            return DemoMachine.setUp

class WaitForTurn(State):
    def transitionRun(self):
        rospy.loginfo("WAIT FOR TURN")
    def mainRun(self):
        # encourage the person to take the turn by looking at the person, turning the 'head' as if asking and looking at the tray
        rospy.loginfo("Demo: Asking.")
        demoFunctions.ask_face_angles(demoVariables.face_position)
    def next(self):
        self.check = demoFunctions.check_number_of_dice() # True=dice_in_tray False=no_dice_in_tray
        #check = raw_input('Is the player done with his/her turn? (y/n) \n') ###
        #if demoFunctions.check_for_yes(check): ###
        if self.check:
            demoVisionChecks(0)
            return DemoMachine.checkScore
        else:
            demoVisionChecks(0)
            return DemoMachine.waitForTurn

class CheckForCup(State):
    def transitionRun(self):
        rospy.loginfo("CHECK FOR CUP")
    def mainRun(self):
        # go to cup checking position
        demoFunctions.execute_goal(0, 'check_for_cup_joint', 1)
    def next(self):
        # tell vision to look for the cup
        demoVisionChecks(3)
        # give the vision some time to check for the cup
        rospy.sleep(1)
        #check = raw_input('Is the cup placed correctly on the stand? (y/n) \n') ###
        #if demoFunctions.check_for_yes(check): ###
        if demoVariables.cup_detected == True:
            demoVisionChecks(0)
            rospy.loginfo("Demo: Cup found.")
            return DemoMachine.takeTurn
        else:
            demoVisionChecks(0)
            rospy.loginfo("Demo: No cup found.")
            # encourage the person to put the cup on the stand
            demoFunctions.ask_face_angles(demoVariables.face_position)
            return DemoMachine.checkForCup

class TakeTurn(State):
    def transitionRun(self):
        rospy.loginfo("TAKE TURN")
    def mainRun(self):
        # go to pick position
        rospy.loginfo("Demo: pick_cup_1")
        demoFunctions.execute_goal(0, 'pick_cup_1_joint', 2)
        # tell the gripper to close
        demoGripperCommand.rPR = 200
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(2)
        # roll and go back to place position
        random_result = randint(0, 1)
        if random_result == 0:
            rospy.loginfo("Demo: roll_1")
            demoFunctions.execute_goal(2, 'roll_1_pose', 9)
        else:
            rospy.loginfo("Demo: roll_3")
            demoFunctions.execute_goal(2, 'roll_3_pose', 14)
        # tell the gripper to open
        demoGripperCommand.rPR = 0
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(2)
        # get clear from the cup  
        demoFunctions.execute_goal(0, 'get_clear_joint', 1)
        demoFunctions.execute_goal(0, 'default_joint', 1)
    def next(self):
        return DemoMachine.checkScore

class CheckScore(State):
    def transitionRun(self):
        rospy.loginfo("CHECK SCORE")
    def mainRun(self):
        # do a check score sequence
        random_result = randint(0, 1)
        if random_result == 0:
            demoFunctions.execute_goal(2, 'check_score_3_pose', 2)
        else:
            demoFunctions.execute_goal(2, 'check_score_4_pose', 2)
        demoFunctions.execute_goal(0, 'check_score_end_position_joint', 1)
    def next(self):
        try:
            #score = int(raw_input('Please enter the score: \n')) ###
            score = demoVariables.amount_of_pips
            rospy.loginfo("Demo: Amount of pips: " + str(score))
            if score < 2 or score > 12:
                rospy.logwarn("Demo: Score isn't possible with two dice, but I guess... Let's just continue.")
        except:
            rospy.logerr("Demo: Invalid score was set.")
        if demoVariables.turn_number == 1:
            demoVariables.turn_number = 0
            demoVariables.bot_score = score
            # check who won this round
            if demoVariables.bot_score > demoVariables.player_score:
                demoVariables.bot_total_score += 1
            elif demoVariables.player_score > demoVariables.bot_score:
                demoVariables.player_total_score += 1
            # check if someone has already won
            if demoVariables.bot_total_score >= demoVariables.winning_amount_of_wins:
                return DemoMachine.reactOnWin
            elif demoVariables.player_total_score >= demoVariables.winning_amount_of_wins:
                return DemoMachine.reactOnLoss
            else:
                return DemoMachine.setUp
        else:
            demoVariables.turn_number = 1
            demoVariables.player_score = score
            return DemoMachine.setUp

class ReactOnWin(State):
    def transitionRun(self):
        rospy.loginfo("REACT ON WIN")
    def mainRun(self):
        random_result = randint(0, 2)
        if random_result == 0:
            rospy.loginfo("Demo: excited")
            demoFunctions.execute_goal(2, 'excited_pose', 8)
        elif random_result == 1:
            rospy.loginfo("Demo: excited_1")
            demoFunctions.execute_goal(2, 'excited_1_pose', 6)
        else:
            rospy.loginfo("Demo: excited_2")
            demoFunctions.execute_goal(2, 'excited_2_pose', 5)
        demoFunctions.execute_goal(0, 'default_joint', 1)
    def next(self):
        return DemoMachine.checkForPeople

class ReactOnLoss(State):
    def transitionRun(self):
        rospy.loginfo("REACT ON LOSS")
    def mainRun(self):
        random_result = randint(0, 2)
        if random_result == 0:
            rospy.loginfo("Demo: sad")
            demoFunctions.execute_goal(2, 'sad_pose', 8)
        elif random_result == 1:
            rospy.loginfo("Demo: sad_1")
            demoFunctions.execute_goal(2, 'sad_1_pose', 12)
        elif random_result == 2:
            rospy.loginfo("Demo: sad_2")
            demoFunctions.execute_goal(2, 'sad_2_pose', 9)
        else:
            rospy.loginfo("Demo: pouting_1")
            demoFunctions.execute_goal(2, 'pouting_1_pose', 12)
        demoFunctions.execute_goal(0, 'default_joint', 1)
    def next(self):
        return DemoMachine.checkForPeople

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('demo_moves_node', anonymous=True)
        rospy.loginfo("demo node starting")

        # prepare the ini parsing
        demoIniHandler = ConfigParser()

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander("manipulator")

        # set reference, allow replanning, give the move group enough time and tries
        group.set_pose_reference_frame = "/base_link"
        group.allow_replanning(True)
        group.set_planning_time(10) # 10 seconds is probably far too much
        group.set_num_planning_attempts(5)

        # init classes
        demoVariables = Variables()
        demoFunctions = Functions()
        demoCallbacks = Callbacks()

        # start the publisher for the gripper command
        demoGripperPublisher = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        demoGripperCommand = outputMsg.Robotiq2FGripper_robot_output()

        # activate gripper
        demoGripperCommand.rACT = 1
        demoGripperCommand.rGTO = 1
        demoGripperCommand.rSP  = 50
        demoGripperCommand.rFR  = 0
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(0.2)
        demoGripperCommand.rACT = 0
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(0.2)
        demoGripperCommand.rACT = 1
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(0.2)

        # open the gripper
        demoGripperCommand.rPR = 0
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(0.2)
        rospy.loginfo("Demo: Gripper activated.")

        # define what files to use
        ini_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/demo.ini'

        # parse the init file to be able to use the movements from it
        demoIniHandler.read(ini_file)

        # subscribe to the vision topics
        demoVision_face_coordinates = rospy.Subscriber('/vision_face_coordinates', FaceCoordinates, demoCallbacks.vision_face)
        demoVision_amount_of_dice = rospy.Subscriber('/vision_amount', Int8, demoCallbacks.vision_amount_of_dice)
        demoVision_amount_of_pips = rospy.Subscriber('/vision_score', Int8, demoCallbacks.vision_amount_of_pips)
        demoVision_cup_detected = rospy.Subscriber('/vision_cup_detected', Int8, demoCallbacks.vision_cup)

        # wait and init the by vision provided service
        rospy.wait_for_service('/vision_checks')
        demoVisionChecks = rospy.ServiceProxy('/vision_checks', SetVisionMode)

        # instantiate state machine
        DemoMachine.checkForPeople = CheckForPeople()
        DemoMachine.idle = Idle()
        DemoMachine.invite = Invite()
        DemoMachine.setUp = SetUp()
        DemoMachine.checkForCup = CheckForCup()
        DemoMachine.takeTurn = TakeTurn()
        DemoMachine.waitForTurn = WaitForTurn()
        DemoMachine.checkScore = CheckScore()
        DemoMachine.reactOnWin = ReactOnWin()
        DemoMachine.reactOnLoss = ReactOnLoss()
        DemoMachine().runAll()

    except rospy.ROSInterruptException:
        pass
