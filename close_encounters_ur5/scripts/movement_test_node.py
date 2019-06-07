#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int8
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *
from ConfigParser import ConfigParser # ini file reading/writing

"""
This program sends a few values from an ini file to the move queue for testing that movement.
"""

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('statemachine_set_up_node', anonymous=True)
        rospy.loginfo("set up actions node starting")

        # init ini reading/writing
        movementTesterIniHandler = ConfigParser()

        # speed, acceleration, tolerance, and sleep_time settings for the requests for the move queue
        general_max_speed = 1.0
        general_max_acceleration = 1.0
        tolerance = 0.0001
        sleeptime = 0.001

        # prepare requests
        #   a joint angle request ready with settings
        joint_request = SendGoalRequest()
        joint_request.speed, joint_request.acceleration, joint_request.tolerance, joint_request.delay = general_max_speed, general_max_acceleration, tolerance, sleeptime
        #   a pose request ready with settings
        pose_request = SendGoalRequest()
        pose_request.speed, pose_request.acceleration, pose_request.tolerance, pose_request.delay = general_max_speed, general_max_acceleration, tolerance, sleeptime

        # init services
        rospy.wait_for_service('/add_goal')
        rospy.wait_for_service('/add_pose_goal')
        movementTesterAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)
        movementTesterAddPoseGoal = rospy.ServiceProxy('/add_pose_goal', SendGoal)

        # wait 5 seconds before starting, so the prompt is visible in the terminal
        rospy.sleep(5)

        # ask for file name
        file_to_read = raw_input('Movement tester: Please enter a file name to execute: ')

        # ask what type to read for
        joint_angles = raw_input('Movement tester: Would you like to read as joint values? (y/n): ')
        if joint_angles == 'y' or joint_angles == 'Y' or joint_angles == 'yes':
            type_to_read = 1
        else:
            pose = raw_input('Movement tester: Would you like to read as joint values? (y/n): ')
            if pose == 'y' or pose == 'Y' or pose == 'yes':
                type_to_read = 2
            else:
                type_to_read = 0

        if type_to_read != 0:
            # wait for enter to be pressed
            while not rospy.is_shutdown():
                # ask for section name
                section_to_read = raw_input('Movement tester: Please enter a section name to execute: ')
                section_valid = False # False till tried and proven otherwise
                
                # ask for amount of keys
                amount_of_keys = raw_input('Movement tester: Please enter a number of goals to execute from this section: ')

                # check if exit isn't requested
                if rospy.is_shutdown():
                    break

                rospy.loginfo("Movement tester: Sending values.")

                # parse ini file
                try:
                    movementTesterIniHandler.read(file_to_read)
                except:
                    rospy.logerr("Movement tester: No file named " + str(file_to_read) + " was found.")
                    break

                for j in range(0, int(amount_of_keys)):
                    if str(type_to_read) == '1':
                        try:
                            # read as joint goal
                            goal_string = movementTesterIniHandler.get(str(section_to_read), str(j + 1))
                            goal_string = goal_string.replace('[', '')
                            goal_string = goal_string.replace(']', '')
                            goal_string = goal_string.replace(',', '')
                            joint_request.goal = map(float, goal_string.split())
                            section_valid = True
                        except:
                            if section_valid == False:
                                rospy.logerr("Movement tester: No section named " + str(section_to_read) + " was found.")
                            else:
                                rospy.logerr("Movement tester: Amount of keys is too high.")
                            break
                        # send as joint goal
                        try:
                            movementTesterAddGoal(joint_request)
                        except:
                            rospy.logerr("Movement tester: Invalid values in file " + str(file_to_read) + " in section [" + str(section_to_read) + "] reading as joint angles.")
                            break
                    elif str(type_to_read) == '2':
                        try:
                            # read as pose goal
                            goal_string = movementTesterIniHandler.get(str(section_to_read), str(j + 1))
                            goal_string = goal_string.replace('[', '')
                            goal_string = goal_string.replace(']', '')
                            goal_string = goal_string.replace(',', '')
                            pose_request.goal = map(float, goal_string.split())
                            section_valid = True
                        except:
                            if section_valid == False:
                                rospy.logerr("Movement tester: No section named " + str(section_to_read) + " was found.")
                            else:
                                rospy.logerr("Movement tester: Amount of keys is too high.")
                            break
                        # send as pose goal
                        try:
                            movementTesterAddPoseGoal(pose_request)
                        except:
                            rospy.logerr("Movement tester: Invalid values in file " + str(file_to_read) + " in section [" + str(section_to_read) + "] reading as pose.")
                            break
                    else:
                        rospy.logerr("Movement tester: Type to read was set incorrectly.")
                        break

                # exit file
                #movementTesterIniHandler.close()

                print("Movement tester: Values sent.")
        
        else:
            rospy.logerr("Movement tester: Type to read was set incorrectly.")

        print("Movement tester: Shutting down.")

    except rospy.ROSInterruptException:
        pass
