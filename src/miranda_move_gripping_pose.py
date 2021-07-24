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
from my_functions import myPoint, myPose, pandaGoals, PandaMove, MirNav2Goal


class MirandaNav2Goal(MirNav2Goal):
    def __init__(self, mir_prefix="", panda_prefix="", panda_description="robot_description"):
        super(MirandaNav2Goal, self).__init__(mir_prefix=mir_prefix)
        self.panda = PandaMove("panda_arm", ns=panda_prefix, robot_description=panda_description)
        # Relative stable Poses to work from (use joint angles and relative Pose) 
        self.pandaRelative = []
        p1 = myPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                          -0.023065127793, 0.0352011934197))
        a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
              2.762937863667806, 0.815807519980951]
        self.pandaRelative.append(pandaGoals(p1, a1))

    def calculateMirGrippingPose(self, gripPose=myPose(), pose_num=0):
        mirPose = myPose()
        mirPose.position = gripPose.position - self.pandaRelative[pose_num].pose_relative.position
        mirPose.orientation.z = 1
        mirPose.orientation.w = 0

        return mirPose

    def movePanda(self, pose_num=0):
        target = self.pandaRelative[pose_num].axis_goal
        self.panda.move_group.set_joint_value_target(target)
        plan = self.panda.move_group.plan()
        # plan = panda_robot.velocity_scale(plan, 0.9)
        self.panda.move_group.execute(plan, wait=True)


if __name__ == '__main__':

    # if len(sys.argv) > 1:
    # mir_prefix = sys.argv[1]    # /miranda/mir
    if len(sys.argv) > 1 and sys.argv[1] == "miranda":
        mir_prefix = "/miranda/mir"
        panda_prefix = "/miranda/panda"
        panda_description = "/miranda/panda/robot_description"
    else:
        mir_prefix = ""
        panda_prefix = ""
        panda_description = "robot_description"
        rospy.loginfo("No prefix is set. pass arggument <miranda> for use with miranda!")

    miranda = MirandaNav2Goal(mir_prefix, panda_prefix, panda_description)

    rate = rospy.Rate(0.5)

    pos = (0, 0, 0)
    point = Point(*pos)

    while not rospy.is_shutdown():
        if not miranda.is_ready():
            continue
        if miranda.status == 4:
            rospy.loginfo("Goal not reachable!")

        rospy.loginfo("Status is: " + str(miranda.status))
        # my_nav.getSendGoal()
        goal_pos = miranda.getGoalCommandLine()
        mir_pose = miranda.calculateMirGrippingPose(goal_pos)
        rospy.loginfo("Mir_pose is: ")
        rospy.loginfo(mir_pose)
        miranda.sendGoalPos(mir_pose)
        time.sleep(0.5)
        while not miranda.is_ready():
            time.sleep(0.1)
        miranda.movePanda(0)
        # miranda.sendGoalPos(goal_pos)
        # TODO: use real mir pose (miranda.mirPose)
        miranda.panda.movePose(miranda.pandaRelative[0].calcRelGoal(miranda.mirPose))
        rospy.loginfo("GoalPose is: ")
        rospy.loginfo(goal_pos)
        # rate.sleep()
        time.sleep(0.1)
