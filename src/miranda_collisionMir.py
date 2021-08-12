#!/usr/bin/env python
# coding=latin-1

# https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/planning_scene_ros_api/src/planning_scene_ros_api_tutorial.cpp
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg as moveit_msg
import geometry_msgs.msg as geom_msg
import shape_msgs.msg as shape_msg
try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

if len(sys.argv) > 1:
  tf_prefix = sys.argv[1]
else:
  tf_prefix = ""

robot_description = tf_prefix+"/robot_description"

rospy.init_node('add_collision_mir', anonymous=True)

safety_dists = np.array([0.0, 0.0, 0.0])
mir_dim = np.array([0.88, 0.6, 0.45])
box_dim = mir_dim+safety_dists

robot = moveit_commander.RobotCommander(robot_description=robot_description,ns=tf_prefix)

p = geom_msg.PoseStamped()
p.header.frame_id = "panda_link0" # robot.get_planning_frame()

print(p)

# ungefahere pose von mir:
p.pose.orientation.w = 1
p.pose.position.z = -mir_dim[2]/2
p.pose.position.x = -mir_dim[0]/2 + 0.25
p.pose.position.y = -mir_dim[1]/2+0.2


scene = moveit_commander.PlanningSceneInterface(ns=tf_prefix)
rospy.sleep(2)
scene.attach_box("panda_link0", 'mir', p, box_dim, touch_links=['panda_link0', 'panda_link1'])


###################
# Panda-Control:
###################
control_dim = np.array([0.4, 0.6, 0.1])

p2 = geom_msg.PoseStamped()
p2.header.frame_id = "panda_link0"
p2.pose.orientation.w = 1
p2.pose.position.z = +control_dim[2]/2
p2.pose.position.x = -mir_dim[0]/2 + 0.25 - control_dim[0]/2
p2.pose.position.y = -mir_dim[1]/2+0.2

scene.attach_box("panda_link0", 'control_box', p2, control_dim)