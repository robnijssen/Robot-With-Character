#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
#import gripper_command # gripper stuff
from std_msgs.msg import Int8
# import service types
from close_encounters_ur5.srv import SendGoal, SendGoalRequest, SendGoalResponse
from close_encounters_ur5.srv import SetPosition, SetPositionResponse
from close_encounters_ur5.srv import SetJointValues, SetJointValuesRequest

"""
This program takes orders with goals, speeds, accelerations, and tolerances.
When it's got an order, it'll execute it.
Orders list can be either added or overwritten.
When done with all orders, it sleeps till a new order is given.
"""
    
class Variables:
    # prepare empty list for the goals supplied later
    goals = []
    speeds = []
    accelerations = []
    tolerances = []
    delays = []
    orders = [goals, speeds, accelerations, tolerances, delays]
    # variables for keeping track of speed, acceleration, and tolerance differences
    # calling set speed ect is not needed if it didn't change
    current_speed = 1
    current_acceleration = 1
    # a variable for checking if the plan was changed during the execution of the movements
    plan_overwritten = False
    # a variable to keep the node giving the move orders informed
    move_number = 0
    # standard responses for the callbacks ready to go
    goal_response = SendGoalResponse()
    goal_response.response = True
    position_response = SetPositionResponse()
    position_response.response = True
    # variables to keep track of the change
    current_face_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    current_cup_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
   
class Functions:
    def execute_movement(self):
        # reset plan_overwritten
        #queueVariables.plan_overwritten = False
        # set speed/acceleration if it changed
        try:
            if queueVariables.current_speed != queueVariables.orders[1][0]:
                group.set_max_velocity_scaling_factor(queueVariables.orders[1][0])
            if queueVariables.current_acceleration != queueVariables.orders[2][0]:
                group.set_max_acceleration_scaling_factor(queueVariables.orders[2][0])
            # compute a plan to get to the next goal
            group.set_joint_value_target(queueVariables.orders[0][0])
            # go to the planned position
            group.go(wait=True)
            '''
            # wait till the goal is reached, or till the plan was changed
            while not rospy.is_shutdown():
                # small sleeping time
                rospy.sleep(queueVariables.orders[4][0])
                # check if the plan changed
                if queueVariables.plan_overwritten == True:
                    break
                # get the pose to compare
                current_values = list(group.get_current_joint_values())
                # check if the values are within the tolerance
                if abs(current_values[0] - float(queueVariables.orders[0][0][0])) <= queueVariables.orders[3][0]:
                    if abs(current_values[1] - float(queueVariables.orders[0][0][1])) <= queueVariables.orders[3][0]:
                        if abs(current_values[2] - float(queueVariables.orders[0][0][2])) <= queueVariables.orders[3][0]:
                            if abs(current_values[3] - float(queueVariables.orders[0][0][3])) <= queueVariables.orders[3][0]:
                                if abs(current_values[4] - float(queueVariables.orders[0][0][4])) <= queueVariables.orders[3][0]:
                                    if abs(current_values[5] - float(queueVariables.orders[0][0][5])) <= queueVariables.orders[3][0]:
                                        # delete the completed item from the list
                                        del queueVariables.orders[0][0]
                                        del queueVariables.orders[1][0]
                                        del queueVariables.orders[2][0]
                                        del queueVariables.orders[3][0]
                                        del queueVariables.orders[4][0]
                                        # add one to the move number and publish it
                                        info = "Move_queue: Goal number " + str(queueVariables.move_number) + " reached."
                                        rospy.loginfo(info)
                                        queueVariables.move_number += 1
                                        fb_queue.publish(queueVariables.move_number)
                                        break
            '''
            # delete the completed item from the list
            del queueVariables.orders[0][0]
            del queueVariables.orders[1][0]
            del queueVariables.orders[2][0]
            del queueVariables.orders[3][0]
            del queueVariables.orders[4][0]
            # add one to the move number and publish it
            info = "Move_queue: Goal number " + str(queueVariables.move_number) + " reached."
            rospy.loginfo(info)
            queueVariables.move_number += 1
            fb_queue.publish(queueVariables.move_number)
        except:
            rospy.logwarn("Move queue: execute_movement passed.")
        # make sure to stop the residual movements
        group.stop()
        group.clear_pose_targets()
    def determine_angles(self, x, y):
        # to do: add corrections using the x and y supplied by the vision node
        return group.get_current_joint_values()

class Callbacks:
    def overwrite_orders(self, overwrite_orders):
        # reset the counter for movement number
        queueVariables.move_number = 0
        rospy.loginfo("Move_queue: Goal number reset")
        # delete items from the list
        del queueVariables.orders[0][:]
        del queueVariables.orders[1][:]
        del queueVariables.orders[2][:]
        del queueVariables.orders[3][:]
        del queueVariables.orders[4][:]
        # populate the list with the new goals and settings
        queueCallbacks.add_order(overwrite_orders)
        # tell execute_movement() that the plan was overwritten
        #queueVariables.plan_overwritten = True
        # plan an go to current position, immediately replan to the next goal in the execution
        group.set_joint_value_target(group.get_current_joint_values())
        group.go(wait=True)
        return queueVariables.goal_response
    def add_order(self, add_orders):
        queueVariables.orders[0].append(list(add_orders.goal))
        queueVariables.orders[1].append(add_orders.speed)
        queueVariables.orders[2].append(add_orders.acceleration)
        queueVariables.orders[3].append(add_orders.tolerance)
        queueVariables.orders[4].append(add_orders.delay)
        return queueVariables.goal_response
    def set_face_position(self, coordinates):
        # determine angles from position in frame and current angles
        determined_joint_angles = SetJointValuesRequest()
        determined_joint_angles.angles = queueFunctions.determine_angles(coordinates.x, coordinates.y)
        # pass determined angles on to the memory, but only if the change is big enough
        if abs(determined_joint_angles.angles[0] - queueVariables.current_face_angles[0]) + abs(determined_joint_angles.angles[1] - queueVariables.current_face_angles[1]) + abs(determined_joint_angles.angles[2] - queueVariables.current_face_angles[2]) + abs(determined_joint_angles.angles[3] - queueVariables.current_face_angles[3]) + abs(determined_joint_angles.angles[4] - queueVariables.current_face_angles[4]) > 0.01:
            setFaceAngles(determined_joint_angles)
            queueVariables.current_cup_angles = determined_joint_angles.angles
        return queueVariables.position_response
    def set_cup_position(self, coordinates):
        # determine angles from position in frame and current angles
        determined_joint_angles = SetJointValuesRequest()
        determined_joint_angles.angles = queueFunctions.determine_angles(coordinates.x, coordinates.y)
        # pass determined angles on to the memory, but only if the change is big enough
        if abs(determined_joint_angles.angles[0] - queueVariables.current_cup_angles[0]) + abs(determined_joint_angles.angles[1] - queueVariables.current_cup_angles[1]) + abs(determined_joint_angles.angles[2] - queueVariables.current_cup_angles[2]) + abs(determined_joint_angles.angles[3] - queueVariables.current_cup_angles[3]) + abs(determined_joint_angles.angles[4] - queueVariables.current_cup_angles[4]) > 0.01:
            setCupAngles(determined_joint_angles)
            queueVariables.current_cup_angles = determined_joint_angles.angles
        return queueVariables.position_response

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('move_queue_node', anonymous=True)
        rospy.loginfo("move queue starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher for the gripper
        #gripper_command = GripperCommand()

        # init classes
        queueVariables = Variables()
        queueCallbacks = Callbacks()
        queueFunctions = Functions()

        # init publisher for feedback to commanding nodes
        fb_queue = rospy.Publisher('/fb_move_queue', Int8, queue_size=1)

        # init services
        rospy.Service('/overwrite_goals', SendGoal, queueCallbacks.overwrite_orders)
        rospy.Service('/add_goal', SendGoal, queueCallbacks.add_order)
        rospy.Service('/set_face_position', SetPosition, queueCallbacks.set_face_position)
        rospy.Service('/set_cup_position', SetPosition, queueCallbacks.set_cup_position)

        # wait for other node's services
        rospy.wait_for_service('/set_face_joint_angles')
        rospy.wait_for_service('/set_cup_joint_angles')
        setFaceAngles = rospy.ServiceProxy('/set_face_joint_angles', SetJointValues)
        setCupAngles = rospy.ServiceProxy('/set_cup_joint_angles', SetJointValues)

        rospy.loginfo("Move_queue: Ready to take orders.")

        # sleep till called or shutdown
        while not rospy.is_shutdown():
            if not queueVariables.orders[1] == []:
                # execute the next movement when there's something in the queue
                rospy.loginfo("Move_queue: Executing new order.")
                queueFunctions.execute_movement()
            else:
                # sleep when there's nothing in the queue
                rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
