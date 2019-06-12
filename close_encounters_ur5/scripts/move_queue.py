#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
#import gripper_command # gripper stuff
from std_msgs.msg import Int8
# import message and service types
from close_encounters_ur5.msg import *
from close_encounters_ur5.srv import *
# import tf stuff for roll pitch yaw for pose goals
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
This program takes orders with goals, speeds, accelerations, and tolerances.
When it's got an order, it'll tell move_executor to execute it.
Orders list can be either added to or overwritten.
"""
    
class Variables:
    # prepare empty list for the goals supplied later
    goals = []
    types = []
    speeds = []
    accelerations = []
    tolerances = []
    delays = []
    orders = [goals, types, speeds, accelerations, tolerances, delays]
    # prepare a variable for sending to the move executor
    current_order = ExecuteGoal()
    # type declaration
    pose_goal = geometry_msgs.msg.Pose()
    # standard responses for the callbacks ready to go
    goal_response = SendGoalResponse()
    goal_response.response = True
    position_response = SetPositionResponse()
    position_response.response = True
    # variables to keep track of the change
    current_face_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    current_cup_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # time to sleep after publishing
    sleeptime = 0.05
    # a variable to keep track of if the bot is moving at the moment
    moving = False
    
class Functions:
    def prepare_overwriting(self):
        # reset the counters for order number
        queueVariables.current_order.number = 0
        # delete all items from the order list
        queueFunctions.delete_orders()
        # stop the current movements
        group.stop()
        group.clear_pose_targets()
    def delete_orders(self):
        # clear the full order list
        del queueVariables.orders[0][:]
        del queueVariables.orders[1][:]
        del queueVariables.orders[2][:]
        del queueVariables.orders[3][:]
        del queueVariables.orders[4][:]
        del queueVariables.orders[5][:]
    def execute_upon_add_order_check(self):
        if queueVariables.moving == False:
            if len(queueVariables.orders[1]) == 1:
                # only the newly added order is in the list
                if queueVariables.orders[1][0] != 2:
                    # joint/pose goal should be executed
                    queueVariables.moving = True
                    queueFunctions.update_current_order()
                # else: one cartesian waypoint is in the list, but no go command
            else: # length must be greater than 1 --> must be cartesian path
                if (list(queueVariables.orders[0][len(queueVariables.orders[0]) - 1]) == []) or (queueVariables.orders[1][len(queueVariables.orders[1]) - 1] != 2):
                    # path is complete --> add waypoints and then run as a cartesian path
                    queueVariables.moving = True
                    queueFunctions.start_cartesian_path()
                # else: path is incomplete --> don't do anything
    def start_cartesian_path(self):
        #   send the cartesian path goals
        start_length = len(queueVariables.orders[1])
        queueVariables.current_order.number += 1
        for i in range(0, start_length):
            if queueVariables.orders[1][0] != 2: # end of the (first) cartesian part of the queue
                break
            elif list(queueVariables.orders[0][0]) == []: # explicit start command (and end of the (first) cartesian part of the queue)
                # delete the empty waypoint, shifting the next waypoint/order into position 0
                del queueVariables.orders[0][0]
                del queueVariables.orders[1][0]
                del queueVariables.orders[2][0]
                del queueVariables.orders[3][0]
                del queueVariables.orders[4][0]
                del queueVariables.orders[5][0]
                break
            else:
                # prepare waypoint to send
                queueVariables.current_order.goal = queueVariables.orders[0][0]
                queueVariables.current_order.type = queueVariables.orders[1][0]
                queueVariables.current_order.speed = queueVariables.orders[2][0]
                queueVariables.current_order.acceleration = queueVariables.orders[3][0]
                queueVariables.current_order.tolerance = queueVariables.orders[4][0]
                # send the waypoint
                execute_movement.publish(queueVariables.current_order)
                rospy.sleep(queueVariables.sleeptime)
                # delete order number 0 from the list, shifting the next waypoint into position 0
                del queueVariables.orders[0][0]
                del queueVariables.orders[1][0]
                del queueVariables.orders[2][0]
                del queueVariables.orders[3][0]
                del queueVariables.orders[4][0]
                del queueVariables.orders[5][0]
        # send an empty list to make the executor start running a cartesian path
        queueVariables.current_order.goal = []
        execute_movement.publish(queueVariables.current_order)
        rospy.sleep(queueVariables.sleeptime)
    def update_current_order(self):
        queueVariables.moving = True
        # prepare values to run the order
        queueVariables.current_order.number += 1
        queueVariables.current_order.goal = queueVariables.orders[0][0]
        queueVariables.current_order.type = queueVariables.orders[1][0]
        queueVariables.current_order.speed = queueVariables.orders[2][0]
        queueVariables.current_order.acceleration = queueVariables.orders[3][0]
        queueVariables.current_order.tolerance = queueVariables.orders[4][0]
        # delete order number 0 from the list, shifting the next order into position 0, and making sure it isn't run again
        del queueVariables.orders[0][0]
        del queueVariables.orders[1][0]
        del queueVariables.orders[2][0]
        del queueVariables.orders[3][0]
        del queueVariables.orders[4][0]
        del queueVariables.orders[5][0]
        # run the order
        execute_movement.publish(queueVariables.current_order)
        rospy.sleep(queueVariables.sleeptime)
    def determine_angles(self, x, y):
        # to do: add corrections using the x and y supplied by the vision node
        return group.get_current_joint_values()

class Callbacks:
    def overwrite_orders(self, overwrite_orders):
        rospy.loginfo("Move_queue: Orders overwritten.")
        # reset move number, delete order list, execute upon add_order, and stop current movements
        queueFunctions.prepare_overwriting()
        # populate the list with the new goals and settings
        queueCallbacks.add_order(overwrite_orders)
        return queueVariables.goal_response
    def add_order(self, add_orders):
        # append to lists
        queueVariables.orders[0].append(add_orders.goal)
        queueVariables.orders[1].append(add_orders.type)
        queueVariables.orders[2].append(add_orders.speed)
        queueVariables.orders[3].append(add_orders.acceleration)
        queueVariables.orders[4].append(add_orders.tolerance)
        queueVariables.orders[5].append(add_orders.delay)
        # check if the added order needs to be executed now
        queueFunctions.execute_upon_add_order_check()
        return queueVariables.goal_response
    def order_completed(self, order_number):
        # check if the order isn't just being reset
        if order_number.data == queueVariables.current_order.number:
            # publish a new order if available
            if len(queueVariables.orders[1]) == 0:
                rospy.loginfo("Move_queue: Waiting for a new order to be added.")
                queueVariables.moving = False
            elif queueVariables.orders[1][0] != 2:
                # joint/pose goal should be executed
                queueFunctions.update_current_order()
            else:
                # cartesian --> check the orders for a go command
                complete = False
                for i in range(0, len(queueVariables.orders[1])):
                    if queueVariables.orders[0][i] == []:
                        complete = True
                    elif queueVariables.orders[1][i] != 2:
                        complete = True
                if complete == True:
                    # path is complete --> add waypoints and then run as a cartesian path
                    queueFunctions.start_cartesian_path()
                else:
                    rospy.loginfo("Move_queue: Waiting for a new order to be added.")
                    queueVariables.moving = False
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

        # init classes
        queueVariables = Variables()
        queueCallbacks = Callbacks()
        queueFunctions = Functions()

        # init publisher to command the move executor
        execute_movement = rospy.Publisher('/execute_movement', ExecuteGoal, queue_size=1)

        # init subscriber to check the feedback from the move_executor
        rospy.Subscriber('/fb_move_executor', Int8, queueCallbacks.order_completed)

        # init services
        rospy.Service('/overwrite_goal', SendGoal, queueCallbacks.overwrite_orders)
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
        rospy.spin()

        # stop the movements and clear goals
        rospy.loginfo("Move_queue: Stopping movements.")
        group.stop()
        group.clear_pose_targets()

    except rospy.ROSInterruptException:
        pass
