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

class Queue:
    def __init__(self):
        # prepare the list of orders from the created type
        self._orders = []
        # keep track of the order_number
        self._next_order_number = 1
        self._current_order_number = 0
    def clear(self):
        # delete all items from the queue
        del self._orders[:]
        # reset the order number
        self._next_order_number = 1
    def len(self):
        return len(self._orders)
    def take(self):
        # get the first item from the queue and delete it afterwards
        try:
            item = self._orders[0]
            self._current_order_number = item.number
            del self._orders[0]
            return (item)
        except IndexError:
            rospy.logwarn("Move_queue: order was overwritten before sending an order was completed.")
    def append(self, val):
        # prepare the new order to add to the order list
        new_order = ExecuteGoal()
        new_order.number = self._next_order_number
        new_order.goal = list(val.goal)
        new_order.type = int(val.type)
        new_order.tolerance = float(val.tolerance)
        # only increment the order number if it's a seperate order
        # a cartesian path is counted as a single order
        if val.type != 2 or list(val.goal) == []:
            self._next_order_number += 1
        # add the order that was just converted
        self._orders.append(new_order)
    def next_type(self):
        return self._orders[0].type
    def current_number(self):
        return self._current_order_number
    def check_for_go_command(self):
        length = len(self._orders)
        if length < 2:
            # no go command can be given with less than 2 items in the queue
            return 0
        for i in range(0, length):
            if list(self._orders[i].goal) == []:
                # go command was found at position i (must be greater than 0)
                return int(i)
        # if a return wasn't triggered here, there was no go command
        return 0
    
class Variables:
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
    sleeptime = 0.08
    # a variable to keep track of if the bot is moving at the moment
    moving = 0
    
class Functions:
    def stop(self):
        # make the move_executor and move_group both stop to be able to go on with the next movement without much lag
        queue_group.stop()
        queue_group.clear_pose_targets()
    def start_cartesian_path(self, go_index):
        # publish the waypoints and start command in sequence
        for i in range(0, go_index + 1):
            execute_movement.publish(queue.take())
            rospy.sleep(queueVariables.sleeptime)
    def start_single_goal(self):
        # publish a single order
        execute_movement.publish(queue.take())
        rospy.sleep(queueVariables.sleeptime)
    def determine_angles(self, x, y):
        # to do: add corrections using the x and y supplied by the vision node
        return queue_group.get_current_joint_values()

class Callbacks:
    def overwrite_orders(self, new_order):
        # clear the orders before adding new ones
        queue.clear()
        # append an order to the queue
        queue.append(new_order)
        # stop the move group
        queueFunctions.stop()
        # start the new order if it's a joint/pose goal
        if queue.next_type() == 0 or queue.next_type() == 1:
            rospy.loginfo("Move_queue: Starting a joint or pose goal from overwrite orders.")
            queueFunctions.start_single_goal()
        else:
            rospy.loginfo("Move_queue: Waiting for a go command for this cartesian path in overwrite orders.")
        return True
    def add_order(self, new_order):
        # append an order to the queue
        queue.append(new_order)
        # start if not moving
        if not queueVariables.moving:
            # start the new order if it's a joint/pose goal
            if queue.next_type() == 0 or queue.next_type() == 1:
                rospy.loginfo("Move_queue: Starting a joint or pose goal from add_order.")
                queueFunctions.start_single_goal()
                return True
            # start a cartesian path if a go command was added just now
            if list(new_order.goal) == []:
                rospy.loginfo("Move_queue: Starting a cartesian path from add_order.")
                queueFunctions.start_cartesian_path(queue.check_for_go_command())
            else:
                rospy.loginfo("Move_queue: Cartesian waypoint added. Still waiting for a go command.")
        return True
    def order_completed(self, order_number):
        # check if there's a new order available
        if order_number.data == queue.current_number():
            if queue.len() != 0:
                # start the new order if it's a joint/pose goal
                if queue.next_type() == 0 or queue.next_type() == 1:
                    rospy.loginfo("Move_queue: Starting a joint or pose goal from order_completed.")
                    queueFunctions.start_single_goal()
                    return
                # start a cartesian path if a go command is in the queue
                go_index = queue.check_for_go_command()
                if go_index != 0:
                    rospy.loginfo("Move_queue: Starting a cartesian path from order_completed.")
                    queueFunctions.start_cartesian_path(go_index)
                else:
                    rospy.loginfo("Move_queue: Waiting for a cartesian go command to be added in order_completed.")
            else:
                rospy.loginfo("Move_queue: Waiting for a new order to be added.")
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
    def executor_moving(self, data):
        queueVariables.moving = data.data

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('move_queue_node', anonymous=True)
        rospy.loginfo("move queue starting")

        # init classes
        queue = Queue()
        queueVariables = Variables()
        queueFunctions = Functions()
        queueCallbacks = Callbacks()

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        queue_group = moveit_commander.MoveGroupCommander("manipulator")

        # init publisher to command the move executor
        execute_movement = rospy.Publisher('/execute_movement', ExecuteGoal, queue_size=1)

        # init subscriber to check the feedback from the move_executor
        rospy.Subscriber('/fb_move_executor', Int8, queueCallbacks.order_completed)
        rospy.Subscriber('/executor_moving', Int8, queueCallbacks.executor_moving)

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

        # stop the movements and clear goals (automatically stops the move_executor this way)
        rospy.loginfo("Move_queue: Stopping movements.")
        queue_group.stop()
        queue_group.clear_pose_targets()

    except rospy.ROSInterruptException:
        pass
