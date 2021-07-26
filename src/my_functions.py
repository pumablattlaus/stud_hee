#!/usr/bin/env python
# # coding=latin-1
import copy
import math
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, Pose
import moveit_msgs.msg
from actionlib_msgs.msg import *
import numpy as np
import quaternion


class MyPoint(Point):
    def __init__(self, pos=(0, 0, 0)):
        if type(pos) == Point:
            # super(myPoint, self).__init__(pos)
            self = pos
        else:
            super(MyPoint, self).__init__(*pos)

    def __add__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x + p2.x
        p_out.y = self.y + p2.y
        p_out.z = self.z + p2.z

        return p_out

    def __sub__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x - p2.x
        p_out.y = self.y - p2.y
        p_out.z = self.z - p2.z

        return p_out


class MyOrient(Quaternion):
    def __init__(self, quatern=(0, 0, 0, 1)):
        if type(quatern) == Quaternion:
            self = quatern
        else:
            super(MyOrient, self).__init__(*quatern)

    def __add__(self, o2):
        q1 = quaternion.quaternion(*self.__reduce__()[2])
        q2 = quaternion.quaternion(*o2.__reduce__()[2])
        q3 = q1*q2
        o_out = MyOrient(q3.__reduce__()[1])
        return o_out

    def __sub__(self, o2):
        q1 = quaternion.quaternion(*self.__reduce__()[2])
        q2 = quaternion.quaternion(*o2.__reduce__()[2])
        # q3 = quaternion.quaternion(np.quaternion.conjugate(q1) * q2)
        q3 = q1.conjugate()*q2
        o_out = MyOrient(q3.__reduce__()[1])
        return o_out


class MyPose(Pose):
    def __init__(self, pos=(0, 0, 0), quatern=(0, 0, 0, 1)):
        point = MyPoint(pos)
        orient = MyOrient(quatern)
        super(MyPose, self).__init__(point, orient)

    def __add__(self, p2):
        p_out = MyPose()
        p_out.position = self.position + p2.position
        p_out.orientation = self.orientation + p2.orientation
        return p_out

    def __sub__(self, p2):
        p_out = MyPose()
        p_out.position = self.position - p2.position
        p_out.orientation = self.orientation - p2.orientation
        return p_out


class PandaGoals(object):
    def __init__(self, pose_relative=MyPose(), axis_goal=[]):
        if len(axis_goal) < 7:
            print("using standard for Axis_goal because len<7")
            axis_goal = [-0.09165325995045537, -0.1307664982896102, -0.08691672911214791, -1.2039535559629443,
                         -0.058938511593474276, 1.7850536203251945, -1.5727488613542584]
        self.pose_relative = pose_relative
        self.axis_goal = axis_goal
        self.movement = None

    def calcRelGoal(self, mir_pose=MyPose(), grab_pose=None):
        # vec_move = myPose()
        # pose_current = mir_pose+self.pose_relative
        # vec_move = grab_pose-pose_current
        if grab_pose is None:
            grab_pose = self.pose_relative
        vec_move = grab_pose - mir_pose

        return vec_move


class PandaMove(object):
    def __init__(self, group_name="panda_arm", ns='', robot_description="robot_description"):
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander(ns=ns, robot_description=robot_description)
        # interface to a planning group (group of joints). used to plan and execute motions
        try:
            self.move_group = moveit_commander.MoveGroupCommander(group_name, ns=ns,
                                                                  robot_description=robot_description)
        except RuntimeError:
            rospy.logerr("group name " + group_name + " is not available. Available Planning Groups: ")
            rospy.logerr(self.robot.get_group_names())
            raise RuntimeError

        # remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        # Displays trajectory in RVIZ
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

    def velocity_scale(self, plan, scale):
        for i in range(len(plan.joint_trajectory.points)):
            plan.joint_trajectory.points[i].time_from_start /= scale
            vel = list(plan.joint_trajectory.points[i].velocities)
            acc = list(plan.joint_trajectory.points[i].accelerations)
            for j in range(len(plan.joint_trajectory.points[i].velocities)):
                vel[j] *= scale
                acc[j] *= scale * scale
                plan.joint_trajectory.points[i].velocities = vel
                plan.joint_trajectory.points[i].accelerations = acc

        return plan

    def movePose(self, pose=MyPose(), vel=1):
        pose = Pose(pose.position, pose.orientation)
        self.move_group.set_pose_target(pose)
        # call the planner to compute the plan and execute it.
        # plan = move_group.go(wait=True)
        plan = self.move_group.plan()  # plan can get altered
        plan = self.velocity_scale(plan, vel)
        self.move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def movePoseLin(self, pose=MyPose(), vel=1):
        waypoints = []  # ggf in 90 deg zu Orientierung EEF und dann Rest nur in x-Richtung aus EEF heraus
        waypoints.append(copy.deepcopy(self.move_group.get_current_pose().pose))  # current pose
        wpose = Pose(pose.position, pose.orientation)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        plan = self.velocity_scale(plan, vel)
        self.move_group.execute(plan, wait=True)


class MirNav2Goal(object):

    def __init__(self, mir_prefix=""):
        self.mirPose = MyPose()
        self.status = None  # if robot has no status/goal := 10
        self.id = 0
        self.ready = True

        rospy.init_node('my_moveGoal', anonymous=False)
        self.pub = rospy.Publisher(mir_prefix + '/move_base_simple/goal', PoseStamped, queue_size=1)
        # sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame
        sub_odom = rospy.Subscriber(mir_prefix + '/robot_pose', Pose, self.odom_callback)
        sub_status = rospy.Subscriber(mir_prefix + '/move_base/status', GoalStatusArray, self.status_callback)

    def status_callback(self, msg=GoalStatusArray()):
        if len(msg.status_list):
            id = msg.status_list[0].goal_id.id
            self.status = msg.status_list[0].status
            if id != self.id:
                self.id = id
                self.ready = True
        else:
            self.status = 10

        ############ -- get the current pose of the robot -- #################

    def odom_callback(self, msg):
        # ToDO: test if adding subtracting is working if constructed this way
        self.mirPose = MyPose(msg.position, msg.orientation)
        # self.y = msg.position.y

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
        pose = self.getGoalCommandLine()
        self.sendGoalPos(pose)

    def getGoalCommandLine(self):
        print("Position: ")
        x = float(input("X= "))
        y = float(input("Y= "))

        print("Orientation: 0-360")
        z = float(input("Z= "))

        z = (z % 360) / 360 * 2 - 1
        w = math.sqrt(1 - z ** 2)

        pose = MyPose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        return pose

    def is_ready(self):
        if self.ready and self.status >= 3:
            return True
        return False

    def goalReached(self):
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


if __name__ == '__main__':
    p1 = MyPoint((1, 2, 3))
    p2 = MyPoint((1, 2, 3))

    p3 = p1 + p2

    print(p3)

    pose1 = MyPose((1, 2, 3))
    pose2 = MyPose((1, 2, 3))

    pose3 = MyPose()

    pose3.position = pose1.position + pose2.position

    print(pose3)
