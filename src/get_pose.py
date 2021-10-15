#!/usr/bin/env python
# coding=latin-1

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

panda_description="/miranda/panda/robot_description"
panda_ns="/miranda/panda"


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Provides information such as the robot’s kinematic model and the robot’s current joint states
try:
  robot = moveit_commander.RobotCommander(robot_description=panda_description,ns=panda_ns)
except RuntimeError:
  panda_description="robot_description"
  panda_ns=""
  robot = moveit_commander.RobotCommander(robot_description=panda_description,ns=panda_ns)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# interface to a planning group (group of joints). used to plan and execute motions
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=panda_description, ns=panda_ns)

# remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface(ns=panda_ns)

# Displays trajectory in RVIZ
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

move_group.set_end_effector_link("panda_hand")
# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# move_group.get_jacobian_matrix()
print("============ ROBOT POSE")
p = move_group.get_current_pose()
print(p)