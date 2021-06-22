#!/usr/bin/env python
# coding=latin-1

# https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/planning_scene_ros_api/src/planning_scene_ros_api_tutorial.cpp
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
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

rospy.init_node('add_collision_mir', anonymous=True)


group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


mir_dim = (1.0, 0.5, 0.4)
robot = moveit_commander.RobotCommander()

p = geom_msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()

# ungefahere pose von mir:
# p.pose.orientation.w = 1
# p.pose.position.z = -mir_dim[2]/2
# p.pose.position.x = -mir_dim[0]/2 + 0.15
# p.pose.position.y = -0.1

p.pose.orientation.w = 1
p.pose.position.z = -mir_dim[2]/2
p.pose.position.x = mir_dim[0]/2


scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)
scene.attach_box("panda_link0", 'mir', p, mir_dim, touch_links=['panda_link0'])

rospy.sleep(2)


# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


# plan a motion for this group to a desired pose for the end-effector:
pose_goal = geom_msg.Pose()

pose_goal.position.x =  0.643969949165
pose_goal.position.y = -0.179094911571
pose_goal.position.z = 0.308246775616

pose_goal.orientation.x = 0.983417907524
pose_goal.orientation.y = 0.169264199062
pose_goal.orientation.z = 0.0424994926684
pose_goal.orientation.w = 0.0493218328994

move_group.set_pose_target(pose_goal)

# funktioniert nicht?:
# move_group.set_max_velocity_scaling_factor = 0.0002
# move_group.set_max_acceleration_scaling_factor = 0.02

print("moving:")
# call the planner to compute the plan and execute it.
plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()


# CARTESIAN PATH:
rospy.loginfo("Cartesian Path")
scale = 1
waypoints = []

# The robot’s current joint state must be within some tolerance of the first waypoint in the RobotTrajectory or execute() will fail
wpose = move_group.get_current_pose().pose  
wpose.position.z -= scale * 0.2  #
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

move_group.execute(plan, wait=True)