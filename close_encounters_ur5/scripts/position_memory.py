#!/usr/bin/env python
#position memory
import sys
import rospy
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg


"""
This server will just wait and respond to requests from the other nodes.
"""

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    default_memory_list = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -3.170588795338766]
    
class Variables:
    face_position = memoryConstants.default_memory_list
    cup_position = memoryConstants.default_memory_list

class Callbacks:
    def set_face_position(position):
        memoryVariables.face_position = position.data
        return [0]
    def get_face_position(_):
        return memoryVariables.face_position
    def set_cup_position(position):
        memoryVariables.cup_position = position.data
        return [0]
    def get_cup_position(_):
        return memoryVariables.cup_position
    def get_default_position(_):
        return memoryConstants.default_memory_list

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('position_server', anonymous=True)
        rospy.loginfo("position server starting")

        memoryConstants = Constants()
        memoryVariables = Variables()
        memoryCallbacks = Callbacks()

        # init services
        rospy.Service('set_face_position', Float32MultiArray, memoryCallbacks.set_face_position)
        rospy.Service('get_face_position', Float32MultiArray, memoryCallbacks.get_face_position)
        rospy.Service('set_cup_position', Float32MultiArray, memoryCallbacks.set_cup_position)
        rospy.Service('get_cup_position', Float32MultiArray, memoryCallbacks.get_cup_position)
        rospy.Service('get_default_position', Float32MultiArray, memoryCallbacks.get_default_position)
        while not rospy.is_shutdown:
            rospy.sleep(memoryConstants.sleeptime)

    except rospy.ROSInterruptException:
pass
'''
if face_memory_blinker_set == True:
    face_memory_joint_0= joint_values_variable_name[0]
    face_memory_joint_1= joint_values_variable_name[1]
    face_memory_joint_2= joint_values_variable_name[2]
    face_memory_joint_3= joint_values_variable_name[3]
    face_memory_joint_4= joint_values_variable_name[4]
    face_memory_joint_5= joint_values_variable_name[5]
elif cup_memory_blinker_set == True:
    cup_memory_joint_0= joint_values_variable_name[0]
    cup_memory_joint_1= joint_values_variable_name[1]
    cup_memory_joint_2= joint_values_variable_name[2]
    cup_memory_joint_3= joint_values_variable_name[3]
    cup_memory_joint_4= joint_values_variable_name[4]
    cup_memory_joint_5= joint_values_variable_name[5]

if face_memory_blinker_get == True:
    joint_values_variable_name[0]= face_memory_joint_0
    joint_values_variable_name[1]= face_memory_joint_1
    joint_values_variable_name[2]= face_memory_joint_2
    joint_values_variable_name[3]= face_memory_joint_3
    joint_values_variable_name[4]= face_memory_joint_4
    joint_values_variable_name[5]= face_memory_joint_5
elif cup_memory_blinker_get == True:
    joint_values_variable_name[0]= cup_memory_joint_0
    joint_values_variable_name[1]= cup_memory_joint_1
    joint_values_variable_name[2]= cup_memory_joint_2
    joint_values_variable_name[3]= cup_memory_joint_3
    joint_values_variable_name[4]= cup_memory_joint_4
    joint_values_variable_name[5]= cup_memory_joint_5
elif default_memory_blinker_get == True:
    joint_values_variable_name = default_memory_list
'''