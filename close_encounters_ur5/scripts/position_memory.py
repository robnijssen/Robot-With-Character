#!/usr/bin/env python
import sys
import rospy
# import service types
from close_encounters_ur5.srv import SetJointValues, SetJointValuesResponse, GetJointValues, GetJointValuesRequest, GetJointValuesResponse
from close_encounters_ur5.msg import AnglesList
"""
This server will just wait and respond to requests from the other nodes.
"""

class Constants:
    # default position's joint angles
    default_angles = AnglesList()
    default_angles.angles = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -1.5051539579974573]
    # response ready for SetJointValues services
    res = SetJointValuesResponse()
    res.feedback = True
    
class Variables:
    # face and cup positions (default)
    face_angles = AnglesList()
    face_angles.angles = list(Constants().default_angles.angles)
    cup_angles = AnglesList()
    cup_angles.angles = list(Constants().default_angles.angles)

class Callbacks:
    def set_face_position(self, angles):
        memoryVariables.face_angles.angles = angles.angles
        face_joint_angles_publisher.publish(memoryVariables.face_angles)
        return memoryConstants.res
    def set_cup_position(self, angles):
        memoryVariables.cup_angles.angles = angles.angles
        cup_joint_angles_publisher.publish(memoryVariables.cup_angles)
        return memoryConstants.res

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('position_server', anonymous=True)
        rospy.loginfo("position server starting")

        memoryConstants = Constants()
        memoryVariables = Variables()
        memoryCallbacks = Callbacks()

        # init services
        rospy.Service('/set_face_joint_angles', SetJointValues, memoryCallbacks.set_face_position)
        rospy.Service('/set_cup_joint_angles', SetJointValues, memoryCallbacks.set_cup_position)

        # init publishers
        default_joint_angles_publisher = rospy.Publisher('/default_joint_angles', AnglesList, queue_size=1)
        face_joint_angles_publisher = rospy.Publisher('/face_joint_angles', AnglesList, queue_size=1)
        cup_joint_angles_publisher = rospy.Publisher('/cup_joint_angles', AnglesList, queue_size=1)
        
        # publish the initial values
        default_joint_angles_publisher.publish(memoryConstants.default_angles)
        face_joint_angles_publisher.publish(memoryConstants.default_angles)
        cup_joint_angles_publisher.publish(memoryConstants.default_angles)

        # sleep till shutdown (wake up to answer, then go back to sleep)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass