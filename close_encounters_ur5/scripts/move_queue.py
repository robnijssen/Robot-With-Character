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
    orders = [goals, speeds, accelerations, tolerances]
    # variables for keeping track of speed, acceleration, and tolerance differences
    # calling set speed ect is not needed if it didn't change
    current_speed = 1
    current_acceleration = 1
    # a variable for checking if the plan was changed during the execution of the movements
    plan_overwritten = False
    # a variable to keep the node giving the move orders informed
    move_number = 0
    # a standard response for the callbacks ready to go
    response = SendGoalResponse()
    response.response = True
   
class Functions:
    def execute_movement(self):
        # reset plan_overwritten
        queueVariables.plan_overwritten = False
        # set speed/acceleration if it changed
        if queueVariables.current_speed != queueVariables.orders[1][0]:
            group.set_max_velocity_scaling_factor(queueVariables.orders[1][0])
        if queueVariables.current_acceleration != queueVariables.orders[2][0]:
            group.set_max_acceleration_scaling_factor(queueVariables.orders[2][0])
        # compute a plan to get to the next goal
        group.set_joint_value_target(queueVariables.orders[0][0])
        # go to the planned position
        group.go(wait=False)
        # wait till the goal is reached, or till the plan was changed
        while True:
            # check if the plan changed
            if queueVariables.plan_overwritten == True:
                break
            # get the pose to compare
            current_values = group.get_current_joint_values()
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
                                    # add one to the move number and publish it
                                    info = "Move_queue: Goal number " + str(queueVariables.move_number) + " reached."
                                    rospy.loginfo(info)
                                    queueVariables.move_number += 1
                                    fb_queue.publish(queueVariables.move_number)
                                    break
        # make sure to stop the residual movements
        group.stop()
        group.clear_pose_targets()

class Callbacks:
    def overwrite_orders(self, overwrite_orders):
        # reset the counter for movement number
        queueVariables.move_number = 0
        rospy.loginfo("Move_queue: Goal number reset")
        # delete items from the list
        del queueVariables.orders[0:4][:]
        # populate the list with the new goals and settings
        queueCallbacks.add_order(overwrite_orders)
        # tell execute_movement() that the plan was overwritten
        queueVariables.plan_overwritten = True
        return queueVariables.response
    def add_order(self, add_orders):
        queueVariables.orders[0].append(add_orders.goal)
        queueVariables.orders[1].append(add_orders.speed)
        queueVariables.orders[2].append(add_orders.acceleration)
        queueVariables.orders[3].append(add_orders.tolerance)
        return queueVariables.response

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
        fb_queue = rospy.Publisher('/fb_queue', Int8, queue_size=1)

        # init services
        rospy.Service('/overwrite_goals', SendGoal, queueCallbacks.overwrite_orders)
        rospy.Service('/add_goal', SendGoal, queueCallbacks.add_order)

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
