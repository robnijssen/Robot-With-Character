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
from ConfigParser import ConfigParser # ini file reading/writing

"""
This program takes movements from the ini files to be able to give a demo.
"""

class Variables:
    tolerance = 0.005
    waypoints = []
    current_file_to_read = ' '
    turn_number = 0 # 0=player_turn, 1=bot_turn
    bot_score = 0
    bot_total_score = 0
    player_score = 0
    player_total_score = 0

class Functions:
    def check_for_yes(self, check):
        if check == 'y' or check == 'Y' or check == 'yes' or check == 'affirmative':
            return True
        else:
            return False
    def read_from_ini(self, file_to_read, section_to_read, key_to_read):
        if file_to_read != demoVariables.current_file_to_read:
            demoIniHandler.read(file_to_read)
        goal_string = demoIniHandler.get(str(section_to_read), str(key_to_read))
        goal_list = map(float, goal_string.split())
        return goal_list
    def execute_goal(self, type_to_read, file_to_read, section_to_read, keys_to_read):
        if type_to_read == 0:
            # run as joint goals
            rospy.loginfo("Demo: Executing joint movement.")
            for i in range(0, keys_to_read):
                # read from the file
                goal = demoFunctions.read_from_ini(file_to_read, section_to_read, i + 1)
                # compute a plan
                group.set_joint_value_target(goal)
                # go to the planned position
                group.go(wait=True)
                demoFunctions.stop()
        elif type_to_read == 1:
            # run as pose goals
            rospy.loginfo("Demo: Executing pose movement.")
            for i in range(0, keys_to_read):
                # read from the file
                goal = demoFunctions.read_from_ini(file_to_read, section_to_read, i + 1)
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
            for i in range(0, int(keys_to_read)):
                goal = demoFunctions.read_from_ini(file_to_read, section_to_read, i + 1)
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
    def stop(self):
        # make sure to stop the residual movements
        group.stop()
        group.clear_pose_targets()
        # clear the waypoints
        del demoVariables.waypoints[:]
        demoVariables.waypoints_cleared = True

# state machine

class DemoMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("Demo: IDLE")
    def mainRun(self):
        if rospy.is_shutdown():
            return
        # do some idle movement
        random_result = randint(0, 8)
        if random_result == 0:
            rospy.loginfo("Demo: curious_for_playing_field")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_playing_field_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_playing_field_pose', 7)
        elif random_result == 1:
            rospy.loginfo("Demo: curious_for_playing_field_1")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_playing_field_1_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_playing_field_1_pose', 3)
        elif random_result == 2:
            rospy.loginfo("Demo: curious_for_playing_field_2")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_playing_field_2_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_playing_field_2_pose', 8)
        elif random_result == 3:
            rospy.loginfo("Demo: curious_for_cup")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_cup_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_cup_pose', 7)
        elif random_result == 4:
            rospy.loginfo("Demo: curious_for_cup_1")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_cup_1_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_cup_1_pose', 4)
        elif random_result == 5:
            rospy.loginfo("Demo: curious_for_cup_2")
            demoFunctions.execute_goal(0, idle_file, 'curious_for_cup_2_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'curious_for_cup_2_pose', 6)
        elif random_result == 6:
            rospy.loginfo("Demo: bored_1")
            demoFunctions.execute_goal(0, idle_file, 'bored_1_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'bored_1_pose', 5)
        elif random_result == 7:
            rospy.loginfo("Demo: bored_3")
            demoFunctions.execute_goal(0, idle_file, 'bored_3_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'bored_3_pose', 6)
        else:
            rospy.loginfo("Demo: flip_cup")
            demoFunctions.execute_goal(0, idle_file, 'flip_cup_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'flip_cup_pose', 6)
        # check for people
        random_result = randint(0, 2)
        if random_result == 0:
            rospy.loginfo("Demo: check_for_people")
            demoFunctions.execute_goal(0, idle_file, 'check_for_people_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'check_for_people_pose', 5)
        elif random_result == 1:
            rospy.loginfo("Demo: check_for_people_1")
            demoFunctions.execute_goal(0, idle_file, 'check_for_people_1_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'check_for_people_1_pose', 5)
        else:
            rospy.loginfo("Demo: check_for_people_2")
            demoFunctions.execute_goal(0, idle_file, 'check_for_people_2_joint', 1)
            demoFunctions.execute_goal(2, idle_file, 'check_for_people_2_pose', 6)
        demoFunctions.execute_goal(0, idle_file, 'default_joint', 1)
    def next(self):
        check = raw_input('Is there a person within 2 meters? (y/n) \n')
        if demoFunctions.check_for_yes(check):
            return DemoMachine.invite
        else:
            return DemoMachine.idle

class Invite(State):
    def transitionRun(self):
        rospy.loginfo("Demo: INVITE")
        demoVariables.bot_score = 0
        demoVariables.bot_total_score = 0
        demoVariables.player_score = 0
        demoVariables.player_total_score = 0
        demoVariables.turn_number = 0
    def mainRun(self):
        if rospy.is_shutdown():
            return
        rospy.loginfo("Demo: invite_3")
        demoFunctions.execute_goal(0, invite_file, 'invite_3_joint', 1)
        demoFunctions.execute_goal(2, invite_file, 'invite_3_pose', 6)
        demoFunctions.execute_goal(0, idle_file, 'default_joint', 1)
    def next(self):
        check = raw_input('Is there a person within 2 meters? (y/n) \n')
        if demoFunctions.check_for_yes(check):
            check = raw_input('Is there a person within 1 meter? (y/n) \n')
            if demoFunctions.check_for_yes(check):
                return DemoMachine.setUp
            else:
                return DemoMachine.invite
        else:
            return DemoMachine.idle

class SetUp(State):
    def transitionRun(self):
        rospy.loginfo("Demo: SET UP")
        self.check = raw_input('Is the board already set up? (y/n) \n')
    def mainRun(self):
        if rospy.is_shutdown():
            return
        if not demoFunctions.check_for_yes(self.check):
            # ask for dice in the cup
            random_result = randint(0, 2)
            if random_result == 0:
                rospy.loginfo("Demo: ask_for_dice")
                demoFunctions.execute_goal(2, set_up_file, 'ask_for_dice_pose', 6)
            elif random_result == 1:
                rospy.loginfo("Demo: ask_for_dice_1")
                demoFunctions.execute_goal(2, set_up_file, 'ask_for_dice_1_pose', 6)
            else:
                rospy.loginfo("Demo: ask_for_dice_2")
                demoFunctions.execute_goal(2, set_up_file, 'ask_for_dice_2_pose', 8)
            demoFunctions.execute_goal(0, set_up_file, 'default_joint', 1)
            self.check = raw_input('Is the board set up now? (y/n) \n')
    def next(self):
        if demoFunctions.check_for_yes(self.check):
            if demoVariables.turn_number == 1:
                return DemoMachine.takeTurn
            else:
                return DemoMachine.waitForTurn
        else:
            return DemoMachine.setUp

class TakeTurn(State):
    def transitionRun(self):
        rospy.loginfo("Demo: TAKE TURN")
    def mainRun(self):
        cup_in_position = False
        while not cup_in_position:
            check = raw_input('Is the cup placed correctly on the stand? (y/n) \n')
            if rospy.is_shutdown():
                return
            if demoFunctions.check_for_yes(check):
                cup_in_position = True
            else:
                # ask for the cup
                random_result = randint(0, 1)
                if random_result == 0:
                    rospy.loginfo("Demo: ask_for_cup")
                    demoFunctions.execute_goal(2, take_turn_file, 'ask_for_cup_pose', 3)
                else:
                    rospy.loginfo("Demo: ask_for_cup_2")
                    demoFunctions.execute_goal(2, take_turn_file, 'ask_for_cup_2_pose', 7)
        # choose a sequence
        random_result = randint(0, 1)
        # go to pick position
        rospy.loginfo("Demo: pick_cup_1")
        demoFunctions.execute_goal(0, take_turn_file, 'pick_cup_1_joint', 2)
        # tell the gripper to close
        demoGripperCommand.rPR = 200
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(2)
        # roll and go back to place position
        random_result = randint(0, 1)
        if random_result == 0:
            rospy.loginfo("Demo: roll_1")
            demoFunctions.execute_goal(2, take_turn_file, 'roll_1_pose', 9)
        else:
            rospy.loginfo("Demo: roll_3")
            demoFunctions.execute_goal(2, take_turn_file, 'roll_3_pose', 14)
        # tell the gripper to open
        demoGripperCommand.rPR = 0
        demoGripperPublisher.publish(demoGripperCommand)
        rospy.sleep(2)
        # get clear from the cup  
        demoFunctions.execute_goal(0, take_turn_file, 'get_clear_joint', 1)
        demoFunctions.execute_goal(0, take_turn_file, 'default_joint', 1)

    def next(self):
        return DemoMachine.checkScore

class WaitForTurn(State):
    def transitionRun(self):
        rospy.loginfo("Demo: WAIT FOR TURN")
    def mainRun(self):
        # encourage the person to take the turn
        random_result = randint(0, 2)
        if random_result == 0:
            rospy.loginfo("Demo: ask_for_throw")
            demoFunctions.execute_goal(2, wait_for_turn_file, 'ask_for_throw_pose', 2)
        elif random_result == 1:
            rospy.loginfo("Demo: ask_for_throw_1")
            demoFunctions.execute_goal(2, wait_for_turn_file, 'ask_for_throw_1_pose', 4)
        else:
            rospy.loginfo("Demo: ask_for_throw_2")
            demoFunctions.execute_goal(2, wait_for_turn_file, 'ask_for_throw_2_pose', 7)
        demoFunctions.execute_goal(0, wait_for_turn_file, 'default_joint', 1)
        rospy.sleep(1)
    def next(self):
        check = raw_input('Is the player done with his/her turn? (y/n) \n')
        if demoFunctions.check_for_yes(check):
            return DemoMachine.checkScore
        else:
            return DemoMachine.waitForTurn

class CheckScore(State):
    def transitionRun(self):
        rospy.loginfo("Demo: CHECK SCORE")
    def mainRun(self):
        if rospy.is_shutdown():
            return
        # do a check score sequence
        random_result = randint(0, 1)
        if random_result == 0:
            demoFunctions.execute_goal(2, check_score_file, 'check_score_pose', 5)
        else:
            demoFunctions.execute_goal(2, check_score_file, 'check_score_2_pose', 5)
        demoFunctions.execute_goal(0, check_score_file, 'default_joint', 1)
    def next(self):
        try:
            score = int(raw_input('Please enter the score: \n'))
            if score < 2 or score > 12:
                rospy.logwarn("Demo: Score isn't possible with two dice, but I guess... Let's just continue.")
        except:
            rospy.logerr("Demo: Invalid score was set.")
        if demoVariables.turn_number == 1:
            demoVariables.turn_number = 0
            demoVariables.bot_score = score
            if demoVariables.bot_score > demoVariables.player_score:
                demoVariables.bot_total_score += 1
            elif demoVariables.player_score > demoVariables.bot_score:
                demoVariables.player_total_score += 1
            if (demoVariables.bot_total_score == 2) or (demoVariables.player_total_score == 2):
                return DemoMachine.react
            else:
                return DemoMachine.setUp
        else:
            demoVariables.turn_number = 1
            demoVariables.player_score = score
            return DemoMachine.setUp

class React(State):
    def transitionRun(self):
        rospy.loginfo("Demo: REACT")
    def mainRun(self):
        if rospy.is_shutdown():
            return
        if (demoVariables.bot_total_score == 2):
            rospy.loginfo("Demo: The bot has won.")
            # do a happy move (or so)
            random_result = randint(0, 2)
            if random_result == 0:
                rospy.loginfo("Demo: excited")
                demoFunctions.execute_goal(2, react_file, 'excited_pose', 8)
            elif random_result == 1:
                rospy.loginfo("Demo: excited_1")
                demoFunctions.execute_goal(2, react_file, 'excited_1_pose', 6)
            else:
                rospy.loginfo("Demo: excited_2")
                demoFunctions.execute_goal(2, react_file, 'excited_2_pose', 5)
        else:
            rospy.loginfo("Demo: The player has won.")
            # do a sad move (or so)
            random_result = randint(0, 2)
            if random_result == 0:
                rospy.loginfo("Demo: sad")
                demoFunctions.execute_goal(2, react_file, 'sad_pose', 8)
            elif random_result == 1:
                rospy.loginfo("Demo: sad_1")
                demoFunctions.execute_goal(2, react_file, 'sad_1_pose', 12)
            elif random_result == 2:
                rospy.loginfo("Demo: sad_2")
                demoFunctions.execute_goal(2, react_file, 'sad_2_pose', 9)
            else:
                rospy.loginfo("Demo: pouting_1")
                demoFunctions.execute_goal(2, react_file, 'pouting_1_pose', 12)
        demoFunctions.execute_goal(0, react_file, 'default_joint', 1)
    def next(self):
        check = raw_input('Is there still a person within 2 meters? (y/n) \n')
        if demoFunctions.check_for_yes(check):
            return DemoMachine.invite
        else:
            return DemoMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('demo_moves_node', anonymous=True)
        rospy.loginfo("demo node starting")

        # prepare the ini parsing
        demoIniHandler = ConfigParser()

        # define what files to use
        idle_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/idle_movements.ini'
        invite_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/invite_movements.ini'
        set_up_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/set_up_movements.ini'
        take_turn_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/take_turn_movements.ini'
        wait_for_turn_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/wait_for_turn_movements.ini'
        check_score_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/check_score_movements.ini'
        react_file = '/home/ubuntu/catkin_ws/src/close_encounters_ur5/ini/react_movements.ini'

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

        # instantiate state machine
        DemoMachine.idle = Idle()
        DemoMachine.invite = Invite()
        DemoMachine.setUp = SetUp()
        DemoMachine.takeTurn = TakeTurn()
        DemoMachine.waitForTurn = WaitForTurn()
        DemoMachine.checkScore = CheckScore()
        DemoMachine.react = React()
        DemoMachine().runAll()

    except rospy.ROSInterruptException:
        pass
