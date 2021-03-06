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

# ns="/miranda"
ns="/miranda/panda"
# Try with no namespace if error os thrown:
try:
  panda_description=ns+"/robot_description"

  # Provides information such as the robotâs kinematic model and the robotâs current joint states
  robot = moveit_commander.RobotCommander(robot_description=panda_description)

except RuntimeError:
  ns=""
  panda_description=ns+"/robot_description"

  # Provides information such as the robotâs kinematic model and the robotâs current joint states
  robot = moveit_commander.RobotCommander(robot_description=panda_description)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# interface to a planning group (group of joints). used to plan and execute motions
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description=panda_description)

# remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface(ns=ns)

# Displays trajectory in RVIZ
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


move_group.set_pose_reference_frame(ns+"panda_link0")
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

print("Reference Frame!!!:")
print(move_group.get_pose_reference_frame())
# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


# Kollisionsdetektion:
# scene.


# plan a motion for this group to a desired pose for the end-effector:
# move_group.set_pose_target(move_group.get_current_pose())
pose_goal = geometry_msgs.msg.Pose()

# pose_goal.position.x =  0.643969949165
# pose_goal.position.y = -0.179094911571
# pose_goal.position.z = 0.308246775616

# pose_goal.orientation.x = 0.983417907524
# pose_goal.orientation.y = 0.169264199062
# pose_goal.orientation.z = 0.0424994926684
# pose_goal.orientation.w = 0.0493218328994


pose_goal.position.x = -0.0322782574931
pose_goal.position.y = -0.362752014653
pose_goal.position.z = 0.289523425033

pose_goal.orientation.x = -0.354986669926
pose_goal.orientation.y = 0.933600890671
pose_goal.orientation.z = 0.00276999233915
pose_goal.orientation.w = 0.0486432755392

move_group.set_pose_target(pose_goal)

# funktioniert nicht?:
# move_group.set_max_velocity_scaling_factor = 0.0002
# move_group.set_max_acceleration_scaling_factor = 0.02


# call the planner to compute the plan and execute it.
# plan = move_group.go(wait=True)
# move_group.retime_trajectory(move_group.get_current_state(), plan , 1)  # was ist ref_state_in in?

plan = move_group.plan()
plan = velocity_scale(plan, 1)  # change velocity
print("moving:")
move_group.execute(plan, wait=True)
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
wpose.position.z -= scale * 0.4  #
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

if fraction != 1:
  print("Cartesian path not valid. Fraction: " + str(fraction))
else:
  plan = velocity_scale(plan, 0.2)
  res = move_group.execute(plan, wait=True)
  print("Result = ")
  print(res)

# moveit_commander.Grasp()
