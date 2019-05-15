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
# import service types
from close_encounters_ur5.srv import SetJointValues, SetJointValuesRequest, SetJointValuesResponse, GetJointValues, GetJointValuesRequest, GetJointValuesResponse

"""
This stays in idle, till it's commanded to do something by the /cmd_idle
It will look around to find a face.
"""

# constants and variables used in state machine

class Constants:
    # sleep between state checks
    sleeptime = 0.1
    # wait a certain time between planning the moves
    move_time = 5
    # resolution of the camera to decide where the center is
    res_x = 640
    res_y = 480
    # max speed&acceleration (1=100% 0.5=50%)
    max_speed = 0.1
    max_acceleration = 0.1
    # joint values
    middle_joint_values = [-2.315057341252462, -1.1454232374774378, -2.5245259443866175, 0.5526210069656372, -4.67750066915621, -3.170588795338766]
    left_joint_values = [-2.1855948607074183, -1.1591671148883265, -2.5244303385363978, 0.552585244178772, -4.098508659993307, -3.170588795338766]
    right_joint_values = [-2.411642853413717, -1.1589997450457972, -2.523783270512716, 0.552597165107727, -5.275455776845114, -3.170588795338766]
    # create a request so less lines are needed when calling a GetJointValues service
    get_request = GetJointValuesRequest()
    get_request = True
    
class Variables:
    # a variable to keep track of what state the idle is in
    cmd_idle = 0
    # distance value for distance to face
    face_d = 0
    # a variable to check if the move timer ended
    timer_ended = False

class Callbacks:
    def idle(self, idle):
        checkForFaceVariables.cmd_idle = idle.data
    def face_d(self, face_d):
        checkForFaceVariables.face_d = face_d.data
    def timer_end(self, random_data):
        checkForFaceVariables.timer_ended = True

class Functions:
    def check_for_face_angles(self):
        if checkForFaceVariables.face_d > 0:
            # save the variables
            req_list = checkForFaceGroup.get_current_joint_values()
            # go to the position where the person was last spotted
            checkForFaceGroup.set_joint_value_target(req_list)
            checkForFaceGroup.go(wait=False)
            # send the values to the position memory
            req_srv = SetJointValuesRequest()
            req_srv.angle_0, req_srv.angle_1, req_srv.angle_2, req_srv.angle_3, req_srv.angle_4, req_srv.angle_5 = req_list
            checkForFaceSetFaceJointAngles(req_srv)

# state machine

class CheckMachine(StateMachine):
    def __init__(self):
        # init state is Idle
        StateMachine.__init__(self, Idle())

# states

class Idle(State):
    def transitionRun(self):
        rospy.loginfo("CheckForFace: Not active.")
        # stop residual movement
        checkForFaceGroup.stop()
        checkForFaceGroup.clear_pose_targets()
        # publish done with moving
        fb_check_for_face_publisher.publish(1)
        rospy.sleep(checkForFaceConstants.sleeptime)
    def mainRun(self):
        # publish that the move is not done
        fb_check_for_face_publisher.publish(0)
        rospy.sleep(checkForFaceConstants.sleeptime)
    def next(self):
        if(checkForFaceVariables.cmd_idle == 1):
            return CheckMachine.goToStartPosition
        else:
            return CheckMachine.idle

class GoToStartPosition(State):
    def transitionRun(self):
        rospy.loginfo("CheckForFace: Trying to go to the start position.")
        # set max speed&acceleration
        checkForFaceGroup.set_max_velocity_scaling_factor(checkForFaceConstants.max_speed)
        checkForFaceGroup.set_max_acceleration_scaling_factor(checkForFaceConstants.max_acceleration)
        # check where the face was last spotted, to use that as a starting position
        res_srv = GetJointValuesResponse()
        res_srv = checkForFaceGetDefaultJointAngles(checkForFaceConstants.get_request)
        res_list = [res_srv.angle_0, res_srv.angle_1, res_srv.angle_2, res_srv.angle_3, res_srv.angle_4, res_srv.angle_5]
        # compute a plan to get these joint values
        checkForFaceGroup.set_joint_value_target(res_list)
        # go to the planned position, but don't wait for the end
        checkForFaceGroup.go(wait=False)
        # when the timer has passed, the callback will be called, triggering the next state
        #rospy.Time.now()
        rospy.Timer(rospy.Duration(checkForFaceConstants.move_time), 
            checkForFaceCallbacks.timer_end,
            oneshot=True)
        # reset the timer_ended variable
        checkForFaceVariables.timer_ended = False
    def mainRun(self):
        # check if there is a face, and if there is update the memory and go there
        checkForFaceFunctions.check_for_face_angles()
        rospy.sleep(checkForFaceConstants.sleeptime)
    def next(self):
        if checkForFaceVariables.cmd_idle == 1:
            if checkForFaceVariables.timer_ended == True:
                checkForFaceGroup.clear_pose_targets()
                return CheckMachine.left
            else:
                return CheckMachine.goToStartPosition
        else:
            checkForFaceGroup.stop()
            return CheckMachine.idle

class Left(State):
    def transitionRun(self):
        rospy.loginfo("CheckForFace: Trying to find a face. Looking left.")
        # set max speed&acceleration
        checkForFaceGroup.set_max_velocity_scaling_factor(checkForFaceConstants.max_speed)
        checkForFaceGroup.set_max_acceleration_scaling_factor(checkForFaceConstants.max_acceleration)
        # compute a plan to get these joint values
        checkForFaceGroup.set_joint_value_target(checkForFaceConstants.left_joint_values)
        # go to the planned position, but don't wait for the end
        checkForFaceGroup.go(wait=False)
        # when the timer has passed, the callback will be called, triggering the next state
        #rospy.Time.now()
        rospy.Timer(rospy.Duration(checkForFaceConstants.move_time), 
            checkForFaceCallbacks.timer_end,
            oneshot=True)
        # reset the timer_ended variable
        checkForFaceVariables.timer_ended = False
    def mainRun(self):
        # check if there is a face, and if there is update the memory and go there
        checkForFaceFunctions.check_for_face_angles()
        rospy.sleep(checkForFaceConstants.sleeptime)
    def next(self):
        if checkForFaceVariables.cmd_idle == 1:
            if checkForFaceVariables.timer_ended == True:
                checkForFaceGroup.clear_pose_targets()
                return CheckMachine.right
            else:
                return CheckMachine.left
        else:
            checkForFaceGroup.stop()
            return CheckMachine.idle

class Right(State):
    def transitionRun(self):
        rospy.loginfo("CheckForFace: Trying to find a face. Looking right.")
        # set max speed&acceleration
        checkForFaceGroup.set_max_velocity_scaling_factor(checkForFaceConstants.max_speed)
        checkForFaceGroup.set_max_acceleration_scaling_factor(checkForFaceConstants.max_acceleration)
        # compute a plan to get these joint values
        checkForFaceGroup.set_joint_value_target(checkForFaceConstants.right_joint_values)
        # go to the planned position, but don't wait for the end
        checkForFaceGroup.go(wait=False)
        # when the timer has passed, the callback will be called, triggering the next state
        rospy.Timer(rospy.Duration(checkForFaceConstants.move_time), 
            checkForFaceCallbacks.timer_end,
            oneshot=True)        
        # reset the timer_ended variable
        checkForFaceVariables.timer_ended = False
    def mainRun(self):
        # check if there is a face, and if there is update the memory and go there
        checkForFaceFunctions.check_for_face_angles()
        rospy.sleep(checkForFaceConstants.sleeptime)
    def next(self):
        if checkForFaceVariables.cmd_idle == 1:
            if checkForFaceVariables.timer_ended == True:
                checkForFaceGroup.clear_pose_targets()
                return CheckMachine.end
            else:
                return CheckMachine.right
        else:
            checkForFaceGroup.stop()
            return CheckMachine.idle

class End(State):
    def transitionRun(self):
        rospy.loginfo("CheckForFace: Trying to find a face. Centering.")
        # set max speed&acceleration
        checkForFaceGroup.set_max_velocity_scaling_factor(checkForFaceConstants.max_speed)
        checkForFaceGroup.set_max_acceleration_scaling_factor(checkForFaceConstants.max_acceleration)
        # compute a plan to get these joint values
        checkForFaceGroup.set_joint_value_target(checkForFaceConstants.middle_joint_values)
        # go to the planned position, but don't wait for the end
        checkForFaceGroup.go(wait=False)
        # when the timer has passed, the callback will be called, triggering the next state
        rospy.Timer(rospy.Duration(checkForFaceConstants.move_time), 
            checkForFaceCallbacks.timer_end,
            oneshot=True)
        # reset the timer_ended variable
        checkForFaceVariables.timer_ended = False
    def mainRun(self):
        # check if there is a face, and if there is update the memory and go there
        checkForFaceFunctions.check_for_face_angles()
    def next(self):
        if checkForFaceVariables.cmd_idle == 1 and checkForFaceVariables.timer_ended == False:
            return CheckMachine.end
        else:
            checkForFaceGroup.stop()
            return CheckMachine.idle

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('function_functional_check_for_face_node', anonymous=True)
        rospy.loginfo("check for face function node starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        checkForFaceGroup = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init /fb_check_for_face publisher to give feedback to the idle node
        fb_check_for_face_publisher = rospy.Publisher('/fb_check_for_people', Int8, queue_size=1)

        checkForFaceConstants = Constants()
        checkForFaceVariables = Variables()
        checkForFaceCallbacks = Callbacks()
        checkForFaceFunctions = Functions()

        # init subscribers
        cmd_idle = rospy.Subscriber("/cmd_idle", Int8, checkForFaceCallbacks.idle)
        vision_face_d = rospy.Subscriber("/vision_face_d", Int8, checkForFaceCallbacks.face_d)
        
        # init face position services from the position server
        rospy.wait_for_service('/set_face_position')
        checkForFaceSetFaceJointAngles = rospy.ServiceProxy('/set_face_position', SetJointValues)
        rospy.wait_for_service('/get_face_position')
        checkForFaceGetFaceJointAngles = rospy.ServiceProxy('/get_face_position', GetJointValues)
        rospy.wait_for_service('/get_default_position')
        checkForFaceGetDefaultJointAngles = rospy.ServiceProxy('/get_default_position', GetJointValues)

        # instantiate state machine
        #<statemachine_name>.<state_without_capital_letter> = <state_class_name>()
        CheckMachine.idle = Idle()
        CheckMachine.goToStartPosition = GoToStartPosition()
        CheckMachine.left = Left()
        CheckMachine.right = Right()
        CheckMachine.end = End()
        CheckMachine().runAll(0)

    except rospy.ROSInterruptException:
        pass
