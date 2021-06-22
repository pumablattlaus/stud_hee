#!/usr/bin/env python
# coding=latin-1


# http://moveit2_tutorials.picknik.ai/doc/pick_place/pick_place_tutorial.html


import sys
import rospy
import moveit_commander
import moveit_msgs.msg as moveit_msg
import actionlib
import actionlib_msgs.msg
import franka_gripper.msg as grip_msg
import shape_msgs.msg as shape_msg
import geometry_msgs.msg as geom_msg
import trajectory_msgs.msg as traj_msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


def openGripper(posture=traj_msg.JointTrajectory()):
    # /* Add both finger joints of panda robot. */
    posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

    # /* Set them as open, wide enough for the object to fit. */
    posture.points = [traj_msg.JointTrajectoryPoint()]
    posture.points[0].positions = [ 0.04, 0.04]
    posture.points[0].time_from_start = rospy.Duration(0.5)

def closedGripper(posture=traj_msg.JointTrajectory()):
    # /* Add both finger joints of panda robot. */
    posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]

    # /* Set them as closed. */
    posture.points = [traj_msg.JointTrajectoryPoint()]
    # posture.points[0].positions.resize(2)
    posture.points[0].positions = [ 0.00, 0.00]
    posture.points[0].time_from_start = rospy.Duration(0.5)



def my_open_gripper():

    client = actionlib.SimpleActionClient('/franka_gripper/move', grip_msg.MoveAction)
    client.wait_for_server()
    goal = grip_msg.MoveGoal(width = 0.08, speed = 0.04)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    return True

def my_close_gripper():

    client = actionlib.SimpleActionClient('/franka_gripper/move', grip_msg.MoveAction)
    client.wait_for_server()
    goal = grip_msg.MoveGoal(width = 0.0, speed = 0.04)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    return True


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasping_test', anonymous=True)

# Provides information such as the robot’s kinematic model and the robot’s current joint states
robot = moveit_commander.RobotCommander()

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# interface to a planning group (group of joints). used to plan and execute motions
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


my_open_gripper()
# my_close_gripper()
# my_open_gripper()




# target_gripper = Float64MultiArray()  # gripper stuff
# target_gripper.layout.dim.append(MultiArrayDimension())  # gripper
# target_gripper.layout.dim[0].label = "gripper"
# target_gripper.layout.dim[0].size = 3  # width, speed, force

# object_width = 0.02
# speed = 0.04
# force = 1

# target_gripper.data = [float(object_width), float(speed), float(force)]
# pub_grasp = rospy.Publisher('franka_gripper_grasp', Float64MultiArray, queue_size=0)
# pub_grasp.publish(target_gripper)



collision_object = moveit_msg.CollisionObject()
collision_object.header.frame_id = "panda_link0"
collision_object.id = "object"

# /* Define the primitive and its dimensions. */
collision_object.primitives = [shape_msg.SolidPrimitive()]
collision_object.primitives[0].type = collision_object.primitives[0].BOX
collision_object.primitives[0].dimensions = [0.02, 0.02, 0.2]

# /* Define the pose of the object. */
collision_object.primitive_poses = [geom_msg.Pose()]
collision_object.primitive_poses[0].position.x = 0.5
collision_object.primitive_poses[0].position.y = 0
collision_object.primitive_poses[0].position.z = 0.5
collision_object.primitive_poses[0].orientation.w = 1.0


grasp_obj = moveit_msg.Grasp() # hier ggf. vector mit grasps

grasp_obj.grasp_pose.header.frame_id = "panda_link0"
grasp_obj.grasp_pose.pose.position.x = 0.583627837217
grasp_obj.grasp_pose.pose.position.y = -0.0538808649518
grasp_obj.grasp_pose.pose.position.z = 0.425980600707

grasp_obj.grasp_pose.pose.orientation.w = 0.017993068363
grasp_obj.grasp_pose.pose.orientation.x = 0.872090657478
grasp_obj.grasp_pose.pose.orientation.y = -0.486192407843
grasp_obj.grasp_pose.pose.orientation.z = 0.052450711972

grasp_obj.pre_grasp_approach.direction.header.frame_id = "panda_link0"
grasp_obj.pre_grasp_approach.direction.vector.x = 1.0
grasp_obj.pre_grasp_approach.min_distance = 0.095
grasp_obj.pre_grasp_approach.desired_distance = 0.115

grasp_obj.post_grasp_retreat.direction.header.frame_id = "panda_link0"
grasp_obj.post_grasp_retreat.direction.vector.x = -1.0
grasp_obj.post_grasp_retreat.min_distance = 0.1
grasp_obj.post_grasp_retreat.desired_distance = 0.15

grasp_obj.max_contact_force = 1

openGripper(grasp_obj.pre_grasp_posture)
closedGripper(grasp_obj.grasp_posture)

move_group.pick("object", grasp_obj)