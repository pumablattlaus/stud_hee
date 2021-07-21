#!/usr/bin/env python
# coding=latin-1

import rospy
import moveit_commander
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
import moveit_msgs.msg
import sys
import time
import numpy as np
import math
from my_functions import myPoint, myPose

# class myPose(Pose):
#     def __init__(self, pos=(0,0,0), quatern = (0, 0, 0, 1)):
#         point = Point(*pos)
#         orient = Quaternion(*quatern)
#         super(myPose, self).__init__(point, orient)

class MirandaNav2Goal(object):

    def __init__(self, mir_mir_prefix=""):
        self.x = None
        self.y = None
        self.status = None    # if robot has no status/goal := 10
        self.id = 0
        self.ready = True

        # Relative stable Poses to work from (use joint angles and relative Pose) 
        self.posesPandaRel = []
        self.posesPandaRel.append(myPose((0.5, 0, 0)))
        self.posesPandaRel.append(myPose((0.2, 0.6, 0)))
        self.posesPandaRel.append(myPose((0.2, -0.6, 0)))
        
        self.axisGoalsPosesPanda = []
        self.axisGoalsPosesPanda.append([0, 0, 0, 0, 0, 0, 0])

        rospy.init_node('my_moveGoal', anonymous=False)
        self.pub = rospy.Publisher(mir_mir_prefix + '/move_base_simple/goal', PoseStamped, queue_size=1) 
        # sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame
        sub_odom = rospy.Subscriber(mir_mir_prefix + '/robot_pose', Pose, self.odom_callback)
        sub_status = rospy.Subscriber(mir_mir_prefix+'/move_base/status', GoalStatusArray, self.status_callback)
    
    def status_callback(self, msg=GoalStatusArray()):
        if len(msg.status_list):
            id = msg.status_list[0].goal_id.id
            self.status=msg.status_list[0].status
            if id != self.id:    
                self.id = id
                self.ready = True
        else:
            self.status=10

        ############ -- get the current pose of the robot -- #################
    def odom_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y

        # rospy.loginfo("------------------------------------------------")
        # rospy.loginfo("pose x = " + str(self.x))
        # rospy.loginfo("pose y = " + str(self.y))

    def sendGoalPos(self, pose):
        self.ready = False
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "map"
        poseMsg.pose = pose

        self.pub.publish(poseMsg)

    def getSendGoal(self):
        pose = self.getGoalCommandLine()
        self.sendGoalPos(pose)
        
    def getGoalCommandLine(self):
        print("Position: ")
        x = float(input("X= "))
        y = float(input("Y= "))

        print("Orientation: 0-360")
        z = float(input("Z= "))

        z=(z%360)/360*2-1
        w = math.sqrt(1-z**2)

        pose = myPose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        
        return pose

    def goalReached():
        #  * /mobile_base_controller/cmd_vel [geometry_msgs/Twist]
        # * /move_base/feedback [move_base_msgs/MoveBaseActionFeedback]
        # * /move_base/goal [move_base_msgs/MoveBaseActionGoal]
        # * /move_base/result [move_base_msgs/MoveBaseActionResult]
        # * /move_base/status [actionlib_msgs/GoalStatusArray] == status_list --> letzter Status
            # GoalStatusArray.status_list.text=="Goal reached" oder .status == 3?
                # "Failed to find a valid plan. Even after executing recovery behaviors." .status==4
                # while .status < 3: sleep
            # sonst: dist(soll-ist < accuracy) via /robot_pose
        pass


    def calculateMirGrippingPose(self, gripPose=Pose()):
        mirPose = Pose()
        mirPose.position = gripPose.position - self.posesPandaRel[0].position
        mirPose.orientation.z = 1
        mirPose.orientation.w = 0
        
        return mirPose


class pandaMove(object):
    def __init__(self, group_name="panda_arm", ns='', robot_description="robot_description"):
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander(ns=ns, robot_description=robot_description)
        # interface to a planning group (group of joints). used to plan and execute motions
        try:
            self.move_group = moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description=robot_description)
        except RuntimeError:
            rospy.logerr("group name " + group_name + " is not available. Available Planning Groups: ")
            rospy.logerr(self.robot.get_group_names())
            raise RuntimeError
        
        # remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        # Displays trajectory in RVIZ
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
    def velocity_scale(self, plan, scale):
        for i in range(len(plan.joint_trajectory.points)):
            plan.joint_trajectory.points[i].time_from_start /= scale
            vel = list(plan.joint_trajectory.points[i].velocities)
            acc = list(plan.joint_trajectory.points[i].accelerations)
            for j in range(len(plan.joint_trajectory.points[i].velocities)):
                vel[j] *= scale
                acc[j] *= scale*scale
                plan.joint_trajectory.points[i].velocities = vel
                plan.joint_trajectory.points[i].accelerations = acc

        return plan



if __name__ == '__main__':
    
    # if len(sys.argv) > 1:
        # mir_prefix = sys.argv[1]    # /miranda/mir
    if len(sys.argv) > 1 and sys.argv[1] == "miranda":
        mir_prefix = "/miranda/mir"
        panda_prefix = "/miranda/panda"
        panda_description="/miranda/panda/robot_description"
    else:
        mir_prefix = ""
        panda_prefix = ""
        panda_description="robot_description"
        rospy.loginfo("No mir_prefix is set. pass arggument <miranda> for use with miranda!")
        
    panda_robot = pandaMove("panda_arm", ns=panda_prefix, robot_description=panda_description)
    # target = [-2.3975483592199653, 0.005740540426159113, -0.8940702379807232, -1.889888072013855, -0.06544553327560425, 1.8750191038037525, -2.5427091833628674]
    # panda_robot.move_group.set_joint_value_target(target)
    target = [-2.3975483592199653, 0.005740540426159113, -0.8940702379807232, -1.889888072013855, -0.06544553327560425, 1.8750191038037525, -1.5427091833628674]
    panda_robot.move_group.set_joint_value_target(target)
    
    plan = panda_robot.move_group.plan()
    plan = panda_robot.velocity_scale(plan, 0.9)
    panda_robot.move_group.execute(plan, wait=True)
    
        
    my_nav = MirandaNav2Goal(mir_prefix)
    rate = rospy.Rate(0.5)

    pos=(0,0,0)
    point = Point(*pos)

    while not rospy.is_shutdown():
        if not my_nav.ready or my_nav.status < 3:
            continue
        if my_nav.status == 4:
            rospy.loginfo("Goal not reachable!")
        
        rospy.loginfo("Status is: " + str(my_nav.status))
        # my_nav.getSendGoal()
        mir_pose = my_nav.calculateMirGrippingPose(my_nav.getGoalCommandLine())
        rospy.loginfo("Mir_pose is: ")
        rospy.loginfo(mir_pose)
        my_nav.sendGoalPos(mir_pose)
        # rate.sleep()
        time.sleep(0.1)
        
        