#!/usr/bin/env python
# coding=latin-1

import sys
import rospy
import moveit_commander
import geometry_msgs.msg as geom_msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("grasp_demo", anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("panda_arm")
# arm_group.set_goal_tolerance(0.5)
print(arm_group.get_named_targets())
arm_group.set_named_target("ready")
plan1 = arm_group.go()

hand_group = moveit_commander.MoveGroupCommander("hand")
print(hand_group.get_named_targets())
hand_group.set_named_target("open")
plan2 = hand_group.go()


# arm close to object:
pose_target = geom_msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0.0
pose_target.position.z = 0.85
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()


pose_target.position.z = 0.825
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()