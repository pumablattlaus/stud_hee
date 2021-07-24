#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
import nav_msgs.
import sys
import time
import numpy as np
import math

class moveMir(object):

    def __init__(self):
        self.start = PoseStamped()
        self.start.header.seq = 0
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', nav_msgs.GetPlan)
        self.req = nav_msgs.GetPlan()


    def pose_callback(self, msg=PoseWithCovarianceStamped()):
        self.start.header.seq += 1
        self.start.header.frame_id = "map"
        self.start.header.stamp = rospy.Time(0)
        self.start.pose = msg.pose.pose
        
    def send_pos(goal_pose=Pose()):
        while self.start.header.seq == 0:   # seq at least 1: pose_callback ran once
            rospy.loginfo("Waiting for start position")
            time.sleep(0.5) #ToDo: ros rate.sleep 

        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time(0)
        goal.pose = goal_pose

        self.req.start = self.start
        self.req.goal = goal
        self.req.tolerance = .1

        resp = self.get_plan(self.req.start, self.req.goal, self.req.tolerance)
        rospy.loginfo("Response: "+ resp)


if __name__ == '__main__':
    my_move = moveMir()

    goal_pose = Pose()
    while self.start.header.seq == 0:   # seq at least 1: pose_callback ran once
            rospy.loginfo("Main: Waiting for start position")
            time.sleep(0.1)
    
    goal_pose.position.x = my_move.start.pose.position.x + 0.2
    goal_pose.position.y = my_move.start.pose.position.y
    goal_pose.orientation = my_move.start.pose.orientation
    my_move.send_pos(goal_pose)










