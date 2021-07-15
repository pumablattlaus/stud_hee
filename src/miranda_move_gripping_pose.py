#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
import sys
import time
import numpy as np
import math

class myPose(Pose):
    def __init__(self, pos=(0,0,0), quatern = (0, 0, 0, 1)):
        point = Point(*pos)
        orient = Quaternion(*quatern)
        super(myPose, self).__init__(point, orient)

class Nav2Goal(object):

    def __init__(self, mir_prefix=""):
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

        rospy.init_node('my_moveGoal', anonymous=False)
        self.pub = rospy.Publisher(mir_prefix + '/move_base_simple/goal', PoseStamped, queue_size=1) 
        # sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame
        sub_odom = rospy.Subscriber(mir_prefix + '/robot_pose', Pose, self.odom_callback)
        sub_status = rospy.Subscriber(mir_prefix+'/move_base/status', GoalStatusArray, self.status_callback)
    
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
        print("Position: ")
        x = int(input("X= "))
        y = int(input("Y= "))

        print("Orientation: 0-360")
        z = float(input("Z= "))

        z=(z%360)/360*2-1
        w = math.sqrt(1-z**2)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        self.sendGoalPos(pose)

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


    def calculateMirGrippingPose(self, gripPose):
        mirPose = Pose()




if __name__ == '__main__':
    if len(sys.argv) > 1:
        prefix = sys.argv[1]    # /miranda/mir
    else:
        prefix = ''
    my_nav = Nav2Goal(prefix)
    rate = rospy.Rate(0.5)

    pos=(0,0,0)
    point = Point(*pos)

    while not rospy.is_shutdown():
        if not my_nav.ready or my_nav.status < 3:
            continue
        if my_nav.status == 4:
            rospy.loginfo("Goal not reachable!")
        
        rospy.loginfo("Status is: " + str(my_nav.status))
        my_nav.getSendGoal()
        # rate.sleep()
        time.sleep(0.5)