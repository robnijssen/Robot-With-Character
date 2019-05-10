#! /usr/bin/env python
'''
    Standard pose:  position:       X: 0.81725
    extended straight               Y: 0.19145
                                    Z: -0.00549

                    Orientation:    X: 0.7071
                                    Y: 0.7071
                                    Z: 0
                                    W: 0 --> standard on 1.0

    test pose:      position:       X: 0
    up                              Y: 0.19
                                    Z: 1.0
                    
                    Orientation:    X: 7.265e-05
                                    Y: 7.264e-05
                                    Z: 0.707
                                    W: 0 -->standard on 1.0
    ****************************************************************************************************************
    joint values: 
    state 1 (configured home): 0, -1.61, 1.63, -1.59, 4.80, 1.54
    state 2 (inbetween): -0.29, -1.82, 1.50, -2.86, 5.68, 1.71
    state 3 (up): 0, -1.57, 0, -1.57, 0, 0
    ****************************************************************************************
    for real joint goals:
    state 1 (above the object): -2.2618, -1.8381, -1.3082, -1.5310, 1.5861, 0.0775
    state 2 (over object): -2.2548, -2.0859, -1.7383, -0.8532, 1.5858, 0.0847 --> close gripper
    state 3 (above drop place): -3.5905, -1.7816, 1.3546, -1.1309, 4.7290, -0.3904
    state 4 (drop place): -3.5744, -1.7631, 2.2500, -2.0451, 4.7292, 0.3529 --> open gripper
    *****************************************************************************************
    poses needed, read it from /world:
    state 0 (above the object):     Position:       X: 0.4668
                                                    Y: 0.3951
                                                    Z: 0.4183

                                    Orientation:    X: -0.2656
                                                    Y: 0.6423
                                                    Z = 0.2866
                                                    W = 0.6593  --> standard on 1.0

    state 1 (over the object):      Position:       X: 0.4699
                                                    Y: 0.4055
                                                    Z: 0.1327

                                    Orientation:    X: -0.2656
                                                    Y: 0.6424
                                                    Z = 0.2866
                                                    W = 0.6593  --> standard on 1.0

    state 2 (above the dropzone):   Position:       X: -0.3738
                                                    Y: 0.0574
                                                    Z: 0.5837

                                    Orientation:    X: -0.7086
                                                    Y: 0.0280
                                                    Z = 0.7050
                                                    W = 0.0135 --> standard on 1.0

    state 3 (drop zone):            Position:       X: -0.3723
                                                    Y: 0.0502
                                                    Z: 0.2393

                                    Orientation:    X: -0.6574
                                                    Y: 0.2768
                                                    Z = 0.6490
                                                    W = 0.2646 --> standard on 1.0
    
'''
import rospy
import sys 
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#import gripper_service_client
presentCase = 0

def statuscb (data):
    global status
    status = data.status_list.status
    print data

class executeState():
    def __init__(self):
        global presentCase
        self.case = presentCase

    def moveArm(self):
        '''
        poseMsg.header.frame_id = "/base_link"
        #poseMsg.pose.orientation.w = 1.0
        moveitGroup.set_pose_target(poseMsg)
        #print(poseMsg)
        plan = moveitGroup.go(wait=True)
        moveitGroup.stop
        moveitGroup.clear_pose_targets()
        '''
        planned_path = moveitGroup.plan(joint_goal)
        moveitGroup.execute(planned_path, wait=True)
        
        rospy.sleep(1)
        

    def case0(self): #abovePickUp
        '''
        poseMsg.pose.position.x = 0.4668
        poseMsg.pose.position.y = 0.3951
        poseMsg.pose.position.z = 0.4183
        poseMsg.pose.orientation.x = -0.2656
        poseMsg.pose.orientation.y = 0.6423
        poseMsg.pose.orientation.z = 0.2866
        poseMsg.pose.orientation.w = 0.6593
        '''
        joint_goal[0] = -2.2618
        joint_goal[1] = -1.8381
        joint_goal[2] = -1.3082
        joint_goal[3] = -1.5310
        joint_goal[4] = 1.5861
        joint_goal[5] = 0.0775
        
        self.moveArm()

    def case1(self): #overObject
        '''
        poseMsg.pose.position.x = 0.4699
        poseMsg.pose.position.y = 0.4055
        poseMsg.pose.position.z = 0.1327
        poseMsg.pose.orientation.x = -0.2656
        poseMsg.pose.orientation.y = 0.6424
        poseMsg.pose.orientation.z = 0.2866
        '''
        joint_goal[0] = -2.2548
        joint_goal[1] = -2.0859
        joint_goal[2] = -1.7383
        joint_goal[3] = -0.8532
        joint_goal[4] = 1.5858
        joint_goal[5] = 0.0847
        
        self.moveArm() 
        #gripper.gripper_client(0)


    def case2(self): #above drop
        '''
        poseMsg.pose.position.x = -0.3738
        poseMsg.pose.position.y = 0.0574
        poseMsg.pose.position.z = 0.5837
        poseMsg.pose.orientation.x = -0.7086
        poseMsg.pose.orientation.y = 0.0280
        poseMsg.pose.orientation.z = 0.7050
        '''
        joint_goal[0] = -3.5905
        joint_goal[1] = -1.7816
        joint_goal[2] = 1.3546
        joint_goal[3] = -1.1309
        joint_goal[4] = 4.7290
        joint_goal[5] = -0.3904
        
        self.moveArm()   

    def case3(self): #dropPlace
        '''
        poseMsg.pose.position.x = -0.3723
        poseMsg.pose.position.y = 0.0502
        poseMsg.pose.position.z = 0.2393
        poseMsg.pose.orientation.x = -0.6574
        poseMsg.pose.orientation.y = 0.2768
        poseMsg.pose.orientation.z = 0.6490
        '''
        joint_goal[0] = -3.5744
        joint_goal[1] = -1.7631
        joint_goal[2] = 2.2500
        joint_goal[3] = -2.0451
        joint_goal[4] = 4.7292
        joint_goal[5] = 0.3529
        
        self.moveArm()  
        #gripper.gripper_client(185)


class stateChange():
    def __init__(self):
        global presentCase
        self.case = presentCase
        #initialize

    def next(self):
        test = True
        if test == True:
            self.case += 1
            presentCase = self.case
            print(self.case)
        else:
            rospy.logerr("Failed to move")
        test = False


if __name__ == '__main__':
    try:
        execute_state = executeState()
        change_state = stateChange()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5_arm', anonymous=True)
        robotcommander = moveit_commander.RobotCommander()

        #planningScene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        moveitGroup = moveit_commander.MoveGroupCommander(group_name)
        #display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        poseMsg = geometry_msgs.msg.PoseStamped()
        #gripper = gripper_service_client
        while not rospy.is_shutdown():
            pose = moveitGroup.get_current_pose()
            jointNames = moveitGroup.get_joints()
            joint_goal = moveitGroup.get_current_joint_values()
            #print ('pose: %s' %pose)
            #print jointValues
            #print(pose)
            if presentCase == 0:
                execute_state.case0()
                #print ('pose case0: %s' %pose)
                presentCase = 1
                #change_state.next()
            elif presentCase == 1:
                execute_state.case1()
                #print ('pose case1: %s' %pose)
                presentCase = 2
            elif presentCase == 2:
                #print ('pose case2: %s' %pose)
                execute_state.case2()
                presentCase = 3
            elif presentCase == 3:
                #print ('pose case3: %s' %pose)
                execute_state.case3()
                presentCase = 4
            elif presentCase == 4:
                presentCase = 0
        
    except rospy.ROSInterruptException: pass