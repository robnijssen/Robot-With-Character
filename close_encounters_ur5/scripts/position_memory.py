#!/usr/bin/env python
import sys
import rospy
# import service types
from close_encounters_ur5.srv import SetJointValues, SetJointValuesResponse, GetJointValues, GetJointValuesResponse

"""
This server will just wait and respond to requests from the other nodes.
"""

class Constants:
    # default position's joint values
    default_joint_values_input = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -3.170588795338766]
    # prepare default position for the service
    default_joint_values = GetJointValuesResponse()
    default_joint_values.angle_0 = default_joint_values_input[0]
    default_joint_values.angle_1 = default_joint_values_input[1]
    default_joint_values.angle_2 = default_joint_values_input[2]
    default_joint_values.angle_3 = default_joint_values_input[3]
    default_joint_values.angle_4 = default_joint_values_input[4]
    default_joint_values.angle_5 = default_joint_values_input[5]
    
class Variables:
    # prepare face and cup positions for the service
    face_position = GetJointValuesResponse()
    face_position.angle_0 = Constants().default_joint_values_input[0]
    face_position.angle_1 = Constants().default_joint_values_input[1]
    face_position.angle_2 = Constants().default_joint_values_input[2]
    face_position.angle_3 = Constants().default_joint_values_input[3]
    face_position.angle_4 = Constants().default_joint_values_input[4]
    face_position.angle_5 = Constants().default_joint_values_input[5]
    cup_position = GetJointValuesResponse()
    cup_position.angle_0 = Constants().default_joint_values_input[0]
    cup_position.angle_1 = Constants().default_joint_values_input[1]
    cup_position.angle_2 = Constants().default_joint_values_input[2]
    cup_position.angle_3 = Constants().default_joint_values_input[3]
    cup_position.angle_4 = Constants().default_joint_values_input[4]
    cup_position.angle_5 = Constants().default_joint_values_input[5]

class Callbacks:
    def set_face_position(self, position):
        memoryVariables.face_position.angle_0 = position.angle_0[0]
        memoryVariables.face_position.angle_1 = position.angle_1[1]
        memoryVariables.face_position.angle_2 = position.angle_2[2]
        memoryVariables.face_position.angle_3 = position.angle_3[3]
        memoryVariables.face_position.angle_4 = position.angle_4[4]
        memoryVariables.face_position.angle_5 = position.angle_5[5]
        res = SetJointValuesResponse()
        res.feedback = True
        return res
    def get_face_position(self, _):
        return memoryVariables.face_position
    def set_cup_position(self, position):
        memoryVariables.cup_position.angle_0 = position.angle_0[0]
        memoryVariables.cup_position.angle_1 = position.angle_1[1]
        memoryVariables.cup_position.angle_2 = position.angle_2[2]
        memoryVariables.cup_position.angle_3 = position.angle_3[3]
        memoryVariables.cup_position.angle_4 = position.angle_4[4]
        memoryVariables.cup_position.angle_5 = position.angle_5[5]
        res = SetJointValuesResponse()
        res.feedback = True
        return res
    def get_cup_position(self, _):
        return memoryVariables.cup_position
    def get_default_position(self, _):
        return memoryConstants.default_joint_values

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('position_server', anonymous=True)
        rospy.loginfo("position server starting")

        memoryConstants = Constants()
        memoryVariables = Variables()
        memoryCallbacks = Callbacks()

        # init services
        rospy.Service('/set_face_position', SetJointValues, memoryCallbacks.set_face_position)
        rospy.Service('/get_face_position', GetJointValues, memoryCallbacks.get_face_position)
        rospy.Service('/set_cup_position', SetJointValues, memoryCallbacks.set_cup_position)
        rospy.Service('/get_cup_position', GetJointValues, memoryCallbacks.get_cup_position)
        rospy.Service('/get_default_position', GetJointValues, memoryCallbacks.get_default_position)
        
        # sleep till shutdown (wake up to answer, then go back to sleep)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass