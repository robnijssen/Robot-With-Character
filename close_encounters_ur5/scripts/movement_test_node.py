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

def set_settings(new):
    if new == True:
        file_to_read = set_file()
        type_to_read = set_type()
        section_to_read = set_section()
        keys_to_read = set_keys()
        return file_to_read, type_to_read, section_to_read, keys_to_read
    else:
        check = raw_input('Movement tester: Would you like to change the settings? (y/n) ')
        if check == 'y' or check == 'Y' or check == 'yes':
            check = raw_input('Movement tester: Would you like to change the file to read? (y/n) ')
            if check == 'y' or check == 'Y' or check == 'yes':
                file_to_read = set_file()
            else: 
                file_to_read = 'unchanged'
            check = raw_input('Movement tester: Would you like to change the type to read? (y/n) ')
            if check == 'y' or check == 'Y' or check == 'yes':
                type_to_read = set_type()
            else: 
                type_to_read = 'unchanged'
            check = raw_input('Movement tester: Would you like to change the section name to read? (y/n) ')
            if check == 'y' or check == 'Y' or check == 'yes':
                section_to_read = set_section()
            else: 
                section_to_read = 'unchanged'
            check = raw_input('Movement tester: Would you like to change the amount of keys to read? (y/n) ')
            if check == 'y' or check == 'Y' or check == 'yes':
                keys_to_read = set_section()
            else: 
                keys_to_read = 'unchanged'
            return file_to_read, type_to_read, section_to_read, keys_to_read
        else:
            return 'unchanged', 'unchanged', 'unchanged', 'unchanged'
def set_file():
    return raw_input('Movement tester: Please enter a file path and name to execute: ')
def set_type():
    # ask what type to read for
    check = raw_input('Movement tester: Would you like to read as joint values? (y/n): ')
    if check == 'y' or check == 'Y' or check == 'yes':
        type_to_read = 0
    else:
        check = raw_input('Movement tester: Would you like to read as pose values? (y/n): ')
        if check == 'y' or check == 'Y' or check == 'yes':
            type_to_read = 1
        else:
            check = raw_input('Movement tester: Would you like to read as cartesian path? (y/n): ')
            if check == 'y' or check == 'Y' or check == 'yes':
                type_to_read = 2
            else:
                type_to_read = -1
    return type_to_read
def set_section():
    return raw_input('Movement tester: Please enter a section name to execute: ')
def set_keys():
    return raw_input('Movement tester: Please enter a number of goals to execute from this section: ')

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
        tolerance = 0.01
        sleeptime = 0.001

        # time between sending to the move queue
        queue_time = 0.1

        # prepare requests
        #   a joint angle request ready with settings
        request = SendGoalRequest()
        request.speed, request.acceleration, request.tolerance, request.delay = general_max_speed, general_max_acceleration, tolerance, sleeptime

        # init services
        rospy.wait_for_service('/add_goal')
        movementTesterAddGoal = rospy.ServiceProxy('/add_goal', SendGoal)

        # wait 5 seconds before starting, so the prompt is visible in the terminal
        rospy.sleep(5)

        # ask for the initial settings
        file_to_read, type_to_read, section_to_read, keys_to_read = set_settings(True)
        section_valid = False # invalid till proven otherwise

        while not rospy.is_shutdown():
            rospy.loginfo("Movement tester: Sending values.")

            # check for valid type to read
            if (type_to_read < 0) or (type_to_read > 2):
                rospy.logerr("Movement tester: Type to read was set incorrectly.")
                break
            
            # set the right type to read
            request.type = type_to_read

            # parse ini file
            try:
                movementTesterIniHandler.read(file_to_read)
            except:
                rospy.logerr("Movement tester: No file named " + str(file_to_read) + " was found.")
                break
            
            for j in range(0, int(keys_to_read)):
                # put string into request goal
                try:
                    goal_string = movementTesterIniHandler.get(str(section_to_read), str(j + 1))
                    request.goal = map(float, goal_string.split())
                    section_valid = True
                except:
                    if section_valid == False:
                        rospy.logerr("Movement tester: No section named " + str(section_to_read) + " was found.")
                    else:
                        rospy.logerr("Movement tester: Amount of keys is too high.")
                    break
                # send goal
                try:
                    movementTesterAddGoal(request)
                    rospy.sleep(queue_time)
                except:
                    rospy.logerr("Movement tester: Invalid values in file " + str(file_to_read) + " in section [" + str(section_to_read) + "] reading as joint angles.")
                    break

            if type_to_read == 2:
                # give start command for cartesian path
                request.goal = []
                movementTesterAddGoal(request)
                rospy.sleep(queue_time)

            rospy.loginfo("Movement tester: Values sent.")

            # ask if settings need to be changed
            new_file_to_read, new_type_to_read, new_section_to_read, new_keys_to_read = set_settings(False)

            if new_file_to_read != 'unchanged':
                file_to_read = new_file_to_read
            if new_type_to_read != 'unchanged':
                type_to_read = new_type_to_read
            if new_section_to_read != 'unchanged':
                section_to_read = new_section_to_read
                section_valid = False # invalid till proven otherwise
            if new_keys_to_read != 'unchanged':
                keys_to_read = new_keys_to_read

            # exit file ?
            #movementTesterIniHandler.close()

        rospy.loginfo("Movement tester: Shutting down.")

    except rospy.ROSInterruptException:
        pass
