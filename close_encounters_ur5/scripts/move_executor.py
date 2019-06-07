#!/usr/bin/env python
import sys
import rospy
import moveit_commander # moveit stuff
import moveit_msgs # moveit stuff
import geometry_msgs.msg
from std_msgs.msg import Int8
# import message types so the node understands what the move queue publications mean
from close_encounters_ur5.msg import *
# import tf stuff for roll pitch yaw for pose goals
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
This program takes one order with a goal, speed, acceleration, and tolerance.
When it's got an order, it'll execute it.
"""

class Variables:
    # type declaration for a pose goal to be used later
    pose_goal = geometry_msgs.msg.Pose()
    # variables for keeping track of speed, acceleration, and tolerance differences
    current_speed = 1
    current_acceleration = 1
    current_tolerance = 0.01

class Functions:
    def check_goal_number(self, number):
        # check if the move number has to be reset
        if number == 1:
            fb_move_executor.publish(0)
            rospy.loginfo("Move_executor: Move number reset.")
    def check_speed_and_acceleration(self, speed, acceleration):
        # change the max speed and max acceleration if they are different
        if abs(executorVariables.current_speed - speed) > 0.001:
            execute_group.set_max_velocity_scaling_factor(speed)
            executorVariables.current_speed = speed
            rospy.loginfo("Move_executor: Speed changed to " + str(speed))
        if abs(executorVariables.current_acceleration - acceleration) > 0.001:
            execute_group.set_max_acceleration_scaling_factor(acceleration)
            executorVariables.current_acceleration = acceleration
            rospy.loginfo("Move_executor: Acceleration changed to " + str(acceleration))
    def check_tolerances(self, tolerance):
        # change the tolerance in two places if it is different
        if abs(executorVariables.current_tolerance - tolerance) > 0.001:
            execute_group.set_goal_orientation_tolerance(order.tolerance)
            execute_group.set_goal_position_tolerance(order.tolerance)
            executorVariables.current_tolerance = tolerance
            rospy.loginfo("Move_executor: Tolerance changed to " + str(tolerance))
    def go(self, number):
        # go to the planned position
        execute_group.go(wait=True)
        # make sure to stop the residual movements
        execute_group.stop()
        execute_group.clear_pose_targets()
        # add one to the move number and publish it
        rospy.loginfo("Move_executor: Goal number " + str(number) + " reached.")
        fb_move_executor.publish(number)

class Callbacks:
    def execute_joint_movement(self, order):
        rospy.loginfo("Move_executor: Executing joint movement.")
        # check if the move_number needs to be reset
        executorFunctions.check_goal_number(order.number)
        # change the max speed and max acceleration if they are different
        executorFunctions.check_speed_and_acceleration(order.speed, order.acceleration)
        # compute a plan
        execute_group.set_joint_value_target(order.goal)
        # go to the planned position
        executorFunctions.go(order.number)
    def execute_pose_movement(self, order):
        rospy.loginfo("Move_executor: Executing pose movement.")
        # check if the move_number needs to be reset
        executorFunctions.check_goal_number(order.number)
        # change the max speed and max acceleration if they are different
        executorFunctions.check_speed_and_acceleration(order.speed, order.acceleration)
        # put the values in the goal
        executorVariables.pose_goal.position.x, executorVariables.pose_goal.position.y, executorVariables.pose_goal.position.z, executorVariables.pose_goal.orientation.x, executorVariables.pose_goal.orientation.y, executorVariables.pose_goal.orientation.z, executorVariables.pose_goal.orientation.w = order.goal
        # change the tolerance in two places if it is different
        executorFunctions.check_tolerances(order.tolerance)
        # send the completed goal
        execute_group.set_pose_target(executorVariables.pose_goal)
        # go to the planned position
        executorFunctions.go(order.number)

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('move_executor_node', anonymous=True)
        rospy.loginfo("move executor starting")

        # start moveit
        moveit_commander.roscpp_initialize(sys.argv)
        execute_group = moveit_commander.MoveGroupCommander("manipulator")

        # give the move group enough time and tries
        execute_group.set_planning_time(10) # 10 seconds is probably far too much
        execute_group.set_num_planning_attempts(5)

        # init classes
        executorVariables = Variables()
        executorFunctions = Functions()
        executorCallbacks = Callbacks()

        # init publisher for feedback to higher level nodes
        fb_move_executor = rospy.Publisher('/fb_move_executor', Int8, queue_size=1)

        # init subscribers to receive commands
        rospy.Subscriber('/execute_joint_movement', ExecuteGoal, executorCallbacks.execute_joint_movement, queue_size=1)
        rospy.Subscriber('/execute_pose_movement', ExecuteGoal, executorCallbacks.execute_pose_movement, queue_size=1)

        rospy.loginfo("Move_executor: Ready to take orders.")

        # sleep till called or shutdown
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
