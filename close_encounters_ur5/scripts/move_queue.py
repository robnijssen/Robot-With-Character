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
    joint_goals = []
    pose_goals = []
    speeds = []
    accelerations = []
    tolerances = []
    delays = []
    orders = [joint_goals, pose_goals, speeds, accelerations, tolerances, delays]
    # prepare a variable for sending to the move executor
    current_order = ExecuteGoal()
    # type declaration
    pose_goal = geometry_msgs.msg.Pose()
    # a variable to check if the order added must be executed immediately
    execute_upon_add_order = True
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
   
class Functions:
    """
    # compute a plan to get to the next goal
    if queueVariables.orders[0][0] != []:
        #   use joint angles
        group.set_joint_value_target(queueVariables.orders[0][0])
    elif queueVariables.orders[1][0] != []:
        #   use pose goal
        # put the values in the goal
        queueVariables.pose_goal.position.x, queueVariables.pose_goal.position.y, queueVariables.pose_goal.position.z, queueVariables.pose_goal.orientation.x, queueVariables.pose_goal.orientation.y, queueVariables.pose_goal.orientation.z, queueVariables.pose_goal.orientation.w = queueVariables.orders[1][0]
        # set tolerances
        group.set_goal_orientation_tolerance(queueVariables.orders[4][0])
        group.set_goal_position_tolerance(queueVariables.orders[4][0])
        # send the completed goal
        group.set_pose_target(queueVariables.pose_goal)
    else:
        # both joint angles and pose lists are empty, so no goal could be planned
        rospy.logerr("Move queue: No order could be given. Joint angles and pose lists are both empty.")
    # go to the planned position
    group.go(wait=True)
    # delete the completed item from the list
    del queueVariables.orders[0][0]
    del queueVariables.orders[1][0]
    del queueVariables.orders[2][0]
    del queueVariables.orders[3][0]
    del queueVariables.orders[4][0]
    del queueVariables.orders[5][0]
    """
    def prepare_overwriting(self):
        # reset the counters for order number
        queueVariables.current_order.number = 0
        # delete all items from the order list
        queueFunctions.delete_orders()
        # execute when add_order is executed
        queueVariables.execute_upon_add_order = True
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
    def delete_order_0(self):
        try:
            # delete order number 0 from the list, shifting the next order into position 0
            del queueVariables.orders[0][0]
            del queueVariables.orders[1][0]
            del queueVariables.orders[2][0]
            del queueVariables.orders[3][0]
            del queueVariables.orders[4][0]
            del queueVariables.orders[5][0]
        except:
            rospy.logwarn("Move_queue: Order 0 couldn't be deleted.")
    def update_current_order_joint_angles(self):
        # overwrite current order with the next 
        queueVariables.current_order.number += 1
        queueVariables.current_order.goal = queueVariables.orders[0][0]
        queueVariables.current_order.speed = queueVariables.orders[2][0]
        queueVariables.current_order.acceleration = queueVariables.orders[3][0]
        queueVariables.current_order.tolerance = queueVariables.orders[4][0]
        execute_joint_movement.publish(queueVariables.current_order)
        rospy.sleep(queueVariables.sleeptime)
    def update_current_order_pose(self):
        queueVariables.current_order.number += 1
        queueVariables.current_order.goal = queueVariables.orders[1][0]
        queueVariables.current_order.speed = queueVariables.orders[2][0]
        queueVariables.current_order.acceleration = queueVariables.orders[3][0]
        queueVariables.current_order.tolerance = queueVariables.orders[4][0]
        execute_pose_movement.publish(queueVariables.current_order)
        rospy.sleep(queueVariables.sleeptime)
    def determine_angles(self, x, y):
        # to do: add corrections using the x and y supplied by the vision node
        return group.get_current_joint_values()

class Callbacks:
    def order_completed(self, order_number):
        #   when an order is completed, a new order will be published if available
        # delete the completed order
        queueFunctions.delete_order_0()
        # check for joint or pose movement
        try:
            if not queueVariables.orders[0][0] == []:
                #rospy.loginfo("Move_queue: Sending joint angle order.")
                # joint goal isn't empty --> execute joint movement
                queueFunctions.update_current_order_joint_angles()
            elif not queueVariables.orders[1][0] == []:
                #rospy.loginfo("Move_queue: Sending pose order.")
                # pose goal isn't empty --> execute pose movement
                queueFunctions.update_current_order_pose()
        except:
            rospy.loginfo("Move_queue: Waiting for a new order to be added.")
            # no next goal yet --> execute in add_goal
            queueVariables.execute_upon_add_order = True
    def overwrite_orders(self, overwrite_orders):
        rospy.loginfo("Move_queue: Orders overwritten with a joint angle order.")
        # reset move number, delete order list, execute upon add_order, and stop current movements
        queueFunctions.prepare_overwriting()
        # populate the list with the new goals and settings
        queueCallbacks.add_order(overwrite_orders)
        rospy.loginfo("Move_queue: Done with overwriting orders.")
        return queueVariables.goal_response
    def add_order(self, add_orders):
        queueVariables.orders[0].append(list(add_orders.goal))
        queueVariables.orders[1].append(list([]))
        queueVariables.orders[2].append(add_orders.speed)
        queueVariables.orders[3].append(add_orders.acceleration)
        queueVariables.orders[4].append(add_orders.tolerance)
        queueVariables.orders[5].append(add_orders.delay)
        if queueVariables.execute_upon_add_order == True:
            rospy.loginfo("Move_queue: Starting movement from add_order.")
            # reset the variable
            queueVariables.execute_upon_add_order = False
            # execute the added order
            queueFunctions.update_current_order_joint_angles()
        rospy.loginfo("Move_queue: Done with adding orders.")
        return queueVariables.goal_response
    def overwrite_pose_order(self, overwrite_orders):
        rospy.loginfo("Move_queue: Orders overwritten with a pose order.")
        # reset move number, delete order list, execute upon add_order, and stop current movements
        queueFunctions.prepare_overwriting()
        # populate the list with the new goals and settings
        queueCallbacks.add_pose_order(overwrite_orders)
        rospy.loginfo("Move_queue: Done with overwriting orders.")
        return queueVariables.goal_response
    def add_pose_order(self, add_orders):
        queueVariables.orders[0].append(list([]))
        queueVariables.orders[1].append(list(add_orders.goal))
        queueVariables.orders[2].append(add_orders.speed)
        queueVariables.orders[3].append(add_orders.acceleration)
        queueVariables.orders[4].append(add_orders.tolerance)
        queueVariables.orders[5].append(add_orders.delay)
        if queueVariables.execute_upon_add_order == True:
            rospy.loginfo("Move_queue: Starting movement from add_pose_order.")
            # reset the variable
            queueVariables.execute_upon_add_order = False
            # execute the added order
            queueFunctions.update_current_order_pose()
        rospy.loginfo("Move_queue: Done with adding orders.")
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

        # give the move group enough time and tries
        group.set_planning_time(10) # 10 seconds is probably far too much
        group.set_num_planning_attempts(5)

        # init classes
        queueVariables = Variables()
        queueCallbacks = Callbacks()
        queueFunctions = Functions()

        # init publisher for feedback to commanding nodes
        #fb_queue = rospy.Publisher('/fb_move_queue', Int8, queue_size=1)

        # init publisher to command the move executor
        execute_joint_movement = rospy.Publisher('/execute_joint_movement', ExecuteGoal, queue_size=1)
        execute_pose_movement = rospy.Publisher('/execute_pose_movement', ExecuteGoal, queue_size=1)

        # init subscriber to check the feedback from the move_executor
        rospy.Subscriber('/fb_move_executor', Int8, queueCallbacks.order_completed)

        # init services
        rospy.Service('/overwrite_goals', SendGoal, queueCallbacks.overwrite_orders)
        rospy.Service('/add_goal', SendGoal, queueCallbacks.add_order)
        rospy.Service('/overwrite_pose_goal', SendGoal, queueCallbacks.overwrite_pose_order)
        rospy.Service('/add_pose_goal', SendGoal, queueCallbacks.add_pose_order)
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
        rospy.loginfo("Stopping movements.")
        group.stop()
        group.clear_pose_targets()

    except rospy.ROSInterruptException:
        pass
