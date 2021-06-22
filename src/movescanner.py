#!/usr/bin/env python
# coding=latin-1

# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# start my_panda.launch before execution 
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

def velocity_scale(plan, scale):
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


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Provides information such as the robot’s kinematic model and the robot’s current joint states
robot = moveit_commander.RobotCommander()

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# interface to a planning group (group of joints). used to plan and execute motions
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface()

# Displays trajectory in RVIZ
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


# # plan a motion for this group to a desired pose for the end-effector:
move_group.set_pose_target(move_group.get_current_pose())

# # funktioniert nicht?:
# # move_group.set_max_velocity_scaling_factor = 0.0002
# # move_group.set_max_acceleration_scaling_factor = 0.02

print("moving:")
# call the planner to compute the plan and execute it.
# plan = move_group.go(wait=True)
# plan = move_group.plan()
# plan = velocity_scale(plan, 0.2)
# # move_group.retime_trajectory(move_group.get_current_state(), plan , 1)  # was ist ref_state_in in?
# move_group.execute(plan, wait=True)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()


# CARTESIAN PATH:
rospy.loginfo("Cartesian Path")
scale = 1
waypoints = []

# The robot’s current joint state must be within some tolerance of the first waypoint in the RobotTrajectory or execute() will fail
wpose = move_group.get_current_pose().pose  
wpose.position.y -= scale * 0.4  #
waypoints.append(copy.deepcopy(wpose))
wpose.position.y += scale * 0.4  #
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

plan = velocity_scale(plan, 0.2)
move_group.execute(plan, wait=True)