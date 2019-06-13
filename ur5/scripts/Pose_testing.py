#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import geometry_msgs.msg
from geometry_msgs import Pose
from math import pi as pi 
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""


  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    
    odom_sub = rospy.Subscriber('/dice', pose_to_list, callback)
    
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the MARA's
    ## arm so we set ``group_name = manipulator``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the MARA:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:

    #group.set_pose_reference_frame("camera_vision_position")
    planning_frame = group.get_planning_frame()


    
    group.set_pose_reference_frame("base_link")
    reference_frame = group.get_pose_reference_frame()
    print "============ planning frame: %s" % planning_frame
   
    print "============ re ference frame: %s" % reference_frame
    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #group_frames = get_group_names()

    print "============ Robot links:",robot.get_link_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## MARA's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -2.36
    joint_goal[1] = -2.10
    joint_goal[2] = -1.90
    joint_goal[3] = -0.62
    joint_goal[4] = 1.78
    joint_goal[5] = 0

    

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  
  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    group = self.group
    
    group.set_planner_id("RRTConnectConfigDefault")
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)

    frame = pose_to_list()
    print frame
    listener = tf.TransformListener()

    (trans,rot) = listener.lookupTransform('/base_link', '/ee_link', rospy.Time(0))
    print (trans)


    roll  = pi + (90 * (pi/180))
    pitch = pi/2
    yaw  =  0 
 

    quaternion = quaternion_from_euler(roll, pitch, yaw)


    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    tpose = group.get_current_pose()
    print type(tpose)
    print tpose

    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )
    group.set_goal_orientation_tolerance(0.005)
    group.set_goal_position_tolerance(0.005)


    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )

    pose_goal = geometry_msgs.msg.Pose()
    """
    pose_goal.orientation.w = 0.245506
    pose_goal.orientation.x = 0.656344
    pose_goal.orientation.y = 0.31796
    pose_goal.orientation.z = -0.638624
    """
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    print(quaternion[0])
    print(quaternion[1])
    print(quaternion[2])
    print(quaternion[3])

    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.3
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_goal_2(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #euler = euler_from_quaternion(roll, pitch, yaw)
    
    roll = pi + (45 * (pi/180))
    pitch = pi/2
    yaw = 0


    quaternion2 = quaternion_from_euler(roll, pitch, yaw)


    group = self.group
    group.set_pose_reference_frame("base_link")
    group.set_planner_id("RRTConnectConfigDefault")
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    tpose = group.get_current_pose()

    dice_pose = group.get
    print type(tpose)
    print tpose

    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )
    group.set_goal_orientation_tolerance(0.05)
    group.set_goal_position_tolerance(0.05)

   
   
    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )

    pose_goal = geometry_msgs.msg.Pose()
    
    #pose_goal.orientation.x = -0.5
    #pose_goal.orientation.y = 0.5
    #pose_goal.orientation.z = 0.5
    #pose_goal.orientation.w = 0.5

    pose_goal.orientation.x = quaternion2[0]
    pose_goal.orientation.y = quaternion2[1]
    pose_goal.orientation.z = quaternion2[2]
    pose_goal.orientation.w = quaternion2[3]

    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.470
    pose_goal.position.z = 0.3
    

    group.set_pose_target(pose_goal)
    
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

    

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose 

    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal_3(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    group = self.group
    group.set_planner_id("RRTConnectConfigDefault")
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)
    
  
    

    
    roll3  = pi + (45 * (pi/180))
    pitch3 = pi/2
    yaw3  =  0 
 

    quaternion3 = quaternion_from_euler(roll3, pitch3, yaw3)


    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    tpose = group.get_current_pose()
    print type(tpose)
    print tpose

    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )
    group.set_goal_orientation_tolerance(0.005)
    group.set_goal_position_tolerance(0.005)


    print("Default Goal Orientation Tolerance: %f" % group.get_goal_orientation_tolerance() )
    print("Default Goal posiition Tolerance: %f" % group.get_goal_position_tolerance() )

    pose_goal = geometry_msgs.msg.Pose()
    """
    pose_goal.orientation.w = 0.245506
    pose_goal.orientation.x = 0.656344
    pose_goal.orientation.y = 0.31796
    pose_goal.orientation.z = -0.638624
    """
    pose_goal.orientation.x = quaternion3[0]
    pose_goal.orientation.y = quaternion3[1]
    pose_goal.orientation.z = quaternion3[2]
    pose_goal.orientation.w = quaternion3[3]

    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.47
    pose_goal.position.z = 0.15
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input() 
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to execute a movement using a pose goal 2 ..."
    raw_input()
    tutorial.go_to_pose_goal_2()

    print "============ Press `Enter` to execute a movement using a pose goal 3 ..."
    raw_input()
    tutorial.go_to_pose_goal_3()


    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()