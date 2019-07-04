#!/usr/bin/env python
import sys
import rospy
# import service types
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *

"""
This server will just wait and respond to requests from the other nodes.
"""

class Constants:
    # default position's joint angles
    default_position = PositionList()
    default_position.angles = [-2.22257644335, -0.57710868517, -2.30035955111, -0.333354775106, 1.80458164215, -1.49498016039]
    default_position.pose = [0.137672240123, 0.0319267662058, 0.512328840913, -0.578146457675, -0.346376922049, 0.421628642553, 0.606629202338]
    # response ready for SetJointValues services
    res = UpdateMemoryResponse()
    res.response = True
    
class Variables:
    # face position (default)
    face_position = PositionList()
    face_position.angles = list(Constants().default_position.angles)
    face_position.pose = list(Constants().default_position.pose)

class Callbacks:
    def set_face_position(self, position):
        memoryVariables.face_position.angles = position.angles
        memoryVariables.face_position.pose = position.pose
        face_position_publisher.publish(memoryVariables.face_position)
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
        rospy.Service('/set_face_position', UpdateMemory, memoryCallbacks.set_face_position)

        # init publishers
        default_position_publisher = rospy.Publisher('/default_position', PositionList, queue_size=1)
        face_position_publisher = rospy.Publisher('/face_position', PositionList, queue_size=1)
        
        # publish the initial values
        default_position_publisher.publish(memoryConstants.default_position)
        face_position_publisher.publish(memoryConstants.default_position)

        # sleep till shutdown (wake up to answer, then go back to sleep)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass