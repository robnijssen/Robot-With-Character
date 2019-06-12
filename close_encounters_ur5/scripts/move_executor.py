#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
from copy import deepcopy # makes sure a copy is made without making a lasting reference
from std_msgs.msg import Int8
# import message types so the node understands what the move queue publications mean
from close_encounters_ur5.msg import *
# import tf stuff for roll pitch yaw for pose goals
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
This program takes one order with a goal, speed, acceleration, and tolerance.
When it's got an order, it'll execute it.

In the case of a cartesian path, it'll add the item to a list of waypoints and wait for the command to run them.
"""

class Variables:
    # variables for keeping track of speed, acceleration, and tolerance differences
    current_speed = 1
    current_acceleration = 1
    current_tolerance = 0.01
    # a list of waypoints for a cartesian path (added to with execute_movement on type=2, started with run_cartesian_path)
    waypoints = []
    waypoints_cleared = True

class Functions:
    def execute_joint_movement(self, order):
        rospy.loginfo("Move_executor: Executing joint movement.")
        # compute a plan
        execute_group.set_joint_value_target(order.goal)
        # go to the planned position
        execute_group.go(wait=True)
        # stop movements and publish done
        executorFunctions.stop(order.number)
    def execute_pose_movement(self, order):
        rospy.loginfo("Move_executor: Executing pose movement.")
        # put the values in the goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x, pose_goal.position.y, pose_goal.position.z = order.goal[0], order.goal[1], order.goal[2]
        pose_goal.orientation.x, pose_goal.orientation.y = order.goal[3], order.goal[4]
        pose_goal.orientation.z, pose_goal.orientation.w = order.goal[5], order.goal[6]
        # change the tolerance in two places if it is different
        executorFunctions.check_tolerances(order.tolerance)
        # send the completed goal
        execute_group.set_pose_target(pose_goal)
        # go to the planned position
        execute_group.go(wait=True)
        # stop movements and publish done
        executorFunctions.stop(order.number)
    def add_to_cartesian_path(self, order):
        # add to list of waypoints
        if executorVariables.waypoints_cleared == True:
            executorVariables.waypoints_cleared = False
            executorVariables.waypoints.append(deepcopy(execute_group.get_current_pose().pose))
        waypoint = geometry_msgs.msg.Pose()
        waypoint.position.x, waypoint.position.y, waypoint.position.z = order.goal[0], order.goal[1], order.goal[2]
        waypoint.orientation.x, waypoint.orientation.y = order.goal[3], order.goal[4]
        waypoint.orientation.z, waypoint.orientation.w = order.goal[5], order.goal[6]
        executorVariables.waypoints.append(waypoint)
    def run_carthesian_path(self, order_number):
        rospy.loginfo("Move_executor: Executing cartesian path.")
        #   start the path in the waypoints
        # plan path with the waypoints, interpolated at every 10 cm, and the jump threshold disabled
        cartesian_plan, fraction = execute_group.compute_cartesian_path(executorVariables.waypoints, 0.01, 0.0)
        # execute the cartesian path that was planned
        execute_group.execute(cartesian_plan, wait=True)
        # clear waypoints
        executorVariables.waypoints = []
        executorVariables.waypoints_cleared = True
        # stop movements and publish done
        executorFunctions.stop(order_number)
    def check_goal_number(self, number):
        # check if the move number has to be reset
        if number == 1:
            rospy.loginfo("Move_executor: Move number reset.")
        fb_move_executor.publish(number - 1)
    def check_speed_and_acceleration(self, speed, acceleration):
        # change the max speed and max acceleration if they are different
        if abs(executorVariables.current_speed - speed) > 0.001:
            if (speed > 0.01) and (speed < 1):
                execute_group.set_max_velocity_scaling_factor(speed)
                executorVariables.current_speed = speed
                rospy.loginfo("Move_executor: Speed changed to " + str(speed))
            else:
                rospy.logwarn("Move_executor: Speed couldn't be changed to " + str(speed))
        if abs(executorVariables.current_acceleration - acceleration) > 0.001:
            if (acceleration > 0.01) and (acceleration < 1):
                execute_group.set_max_acceleration_scaling_factor(acceleration)
                executorVariables.current_acceleration = acceleration
                rospy.loginfo("Move_executor: Acceleration changed to " + str(acceleration))
            else:
                rospy.logwarn("Move_executor: Acceleration couldn't be changed to " + str(acceleration))
    def check_tolerances(self, tolerance):
        # change the tolerance in two places if it is different
        if abs(executorVariables.current_tolerance - tolerance) > 0.001:
            execute_group.set_goal_orientation_tolerance(tolerance)
            execute_group.set_goal_position_tolerance(tolerance)
            executorVariables.current_tolerance = tolerance
            rospy.loginfo("Move_executor: Tolerance changed to " + str(tolerance))
    def stop(self, number):
        # make sure to stop the residual movements
        execute_group.stop()
        execute_group.clear_pose_targets()
        # add one to the move number and publish it
        rospy.loginfo("Move_executor: Goal number " + str(number) + " reached.")
        fb_move_executor.publish(number)

class Callbacks:
    def execute_movement(self, order):
        # check if the move_number needs to be reset
        executorFunctions.check_goal_number(order.number)
        # change the max speed and max acceleration if they are different
        executorFunctions.check_speed_and_acceleration(order.speed, order.acceleration)
        # try to execute
        if list(order.goal) == []:
            executorFunctions.run_carthesian_path(order.number)
        elif order.type == 0:
            executorFunctions.execute_joint_movement(order)
        elif order.type == 1:
            executorFunctions.execute_pose_movement(order)
        elif order.type == 2:
            executorFunctions.add_to_cartesian_path(order)
        else:
            rospy.logfatal("Move_executor: invalid order type.")

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('move_executor_node', anonymous=True)
        rospy.loginfo("move executor starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        execute_group = moveit_commander.MoveGroupCommander("manipulator")

        # set reference, allow replanning, give the move group enough time and tries
        execute_group.set_pose_reference_frame = "/base_link"
        execute_group.allow_replanning(True)
        execute_group.set_planning_time(10) # 10 seconds is probably far too much
        execute_group.set_num_planning_attempts(5)

        # init classes
        executorVariables = Variables()
        executorFunctions = Functions()
        executorCallbacks = Callbacks()

        # init publisher for feedback to higher level nodes
        fb_move_executor = rospy.Publisher('/fb_move_executor', Int8, queue_size=1)

        # init subscribers to receive commands
        rospy.Subscriber('/execute_movement', ExecuteGoal, executorCallbacks.execute_movement, queue_size=1)

        rospy.loginfo("Move_executor: Ready to take orders.")

        # sleep till called or shutdown
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
