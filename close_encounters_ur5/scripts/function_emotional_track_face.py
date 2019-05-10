#!/usr/bin/env python
import sys
import rospy
import time
from random import randint
from std_msgs.msg import Int8, Int16
# moveit stuff
import moveit_commander
import moveit_msgs
import geometry_msgs.msg
# gripper stuff
#import gripper_command
# state machine stuff
from state_machine import StateMachineBlueprint as StateMachine
from state_machine import StateBlueprint as State

"""
This stays in idle, till it's commanded to do something by the /cmd_idle
It will look for and then track a face, and look at it.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.3
    # resolution of the camera to decide where the center is
    res_x = 640
    res_y = 480
    # acceptable variation in pixels from the center
    center_variation_x = 400
    center_variation_y = 200
    # multipliers for the correction joint values
    x_multiplier_0 = 0.003
    x_multiplier_4 = 0.007
    y_multiplier_1 = 0.003
    y_multiplier_3 = 0.007
    # max speed&acceleration (1=100% 0.5=50%)
    max_speed = 0.3
    max_acceleration = 0.3
    # starting joint values
    start_joint_values = [-2.2938314119922083, -1.116389576588766, -2.527221981679098, 0.5780388116836548, -4.732337776814596, -3.170588795338766]
    
class Variables:
    # a variable to keep track of what state the idle is in
    cmd_idle = 0
    # expected joint values for 'looking' directly at a person in front
    face_joint_values = [-2.2938314119922083, -1.116389576588766, -2.527221981679098, 0.5780388116836548, -4.732337776814596, -3.170588795338766]
    # coordinates where the center of the face is detected
    face_x = 0
    face_y = 0

# functions used in state machine

class Functions:
    def determine_face_joint_values(self, face_x, face_y):
            #rospy.loginfo("TrackFace: Tracking face.")
            # update position
            trackfaceVariables.face_joint_values = trackfaceGroup.get_current_joint_values()
            # check if the face is centered enough
            distance_x = trackfaceConstants.res_x / 2 - int(face_x)
            distance_y = trackfaceConstants.res_y / 2 - int(face_y)
            if abs(distance_x) > trackfaceConstants.center_variation_x:
                # change value 0
                trackfaceVariables.face_joint_values[0] += trackfaceConstants.x_multiplier_0 * (distance_x)
                # change value 3 only when not at the limits
                if abs(trackfaceVariables.face_joint_values[4] - constants.start_joint_values[4]) > 1:
                    trackfaceVariables.face_joint_values[4] += trackfaceConstants.x_multiplier_4 * (trackfaceConstants.res_x / 2 - int(face_x))
            if abs(distance_y) > trackfaceConstants.center_variation_y:
                # change value 1
                trackfaceVariables.face_joint_values[1] += trackfaceConstants.y_multiplier_1 * (distance_y)
                # change value 4 only when not at the limits
                if abs(trackfaceVariables.face_joint_values[3] - trackfaceConstants.start_joint_values[3]) < 1:
                    trackfaceVariables.face_joint_values[3] += trackfaceConstants.y_multiplier_3 * (trackfaceConstants.res_y / 2 - int(face_y))
    def go_to_joint_values(self, goal_joint_values):
        # compute a plan to get these joint values
        trackfaceGroup.set_joint_value_target(goal_joint_values)
        # go to the planned position
        trackfaceGroup.go(wait=True)
        trackfaceGroup.stop()
        trackfaceGroup.clear_pose_targets()

class Callbacks:
    def idle(self, idle):
        trackfaceVariables.cmd_idle = idle.data
    def face_x(self, face_x):
        trackfaceVariables.face_x = face_x.data
    def face_y(self, face_y):
        trackfaceVariables.face_y = face_y.data

# state machine

class TrackMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("TrackFace: Not active.")
    def mainRun(self):
        rospy.sleep(trackfaceConstants.sleeptime)
    def next(self):
        if(trackfaceVariables.cmd_idle == 2):
            return TrackMachine.goToStartPosition
        else:
            return TrackMachine.idle

class GoToStartPosition(State):
    def transitionRun(self):
        rospy.loginfo("TrackFace: Trying to go to the start position.")
        # set max speed&acceleration
        trackfaceGroup.set_max_velocity_scaling_factor(trackfaceConstants.max_speed)
        trackfaceGroup.set_max_acceleration_scaling_factor(trackfaceConstants.max_acceleration)
    def mainRun(self):
        # go to start position
        trackfaceFunctions.go_to_joint_values(trackfaceConstants.start_joint_values)
    def next(self):
        if trackfaceVariables.cmd_idle == 2:
            return TrackMachine.track
        else:
            return TrackMachine.idle

class Track(State):
    def transitionRun(self):
        rospy.loginfo("TrackFace: Trying to track a face.")
    def mainRun(self):
        # determine where to go
        trackfaceFunctions.determine_face_joint_values(trackfaceVariables.face_x, trackfaceVariables.face_y)
        # go to the joint values where the face is expected to be
        trackfaceFunctions.go_to_joint_values(trackfaceVariables.face_joint_values)
    def next(self):
        if trackfaceVariables.cmd_idle == 2:
            return TrackMachine.track
        else:
            return TrackMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('function_emotional_track_face_node', anonymous=True)
        rospy.loginfo("track face function node starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        trackfaceGroup = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_track_face publisher to give feedback to the idle node
        #fb_track_face_publisher = rospy.Publisher('/fb_track_face', Int8, queue_size=1)

        trackfaceFunctions = Functions()
        trackfaceConstants = Constants()
        trackfaceVariables = Variables()
        trackfaceCallbacks = Callbacks()

        # init subscribers
        trackfaceCmd_idle = rospy.Subscriber("/cmd_idle", Int8, trackfaceCallbacks.idle)
        vision_face_x = rospy.Subscriber("/vision_face_x", Int16, trackfaceCallbacks.face_x)
        vision_face_y = rospy.Subscriber("/vision_face_y", Int16, trackfaceCallbacks.face_y)
        
        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        TrackMachine.idle = Idle()
        TrackMachine.goToStartPosition = GoToStartPosition()
        TrackMachine.track = Track()
        TrackMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass