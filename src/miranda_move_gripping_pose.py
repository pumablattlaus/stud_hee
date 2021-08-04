#!/usr/bin/env python
# coding=latin-1

import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
import sys
import time
from my_functions import MyPose, PandaGoals, PandaMove, MirNav2Goal


class MirandaNav2Goal(MirNav2Goal):
    def __init__(self, mir_prefix="", panda_prefix="", panda_description="robot_description"):
        super(MirandaNav2Goal, self).__init__(mir_prefix=mir_prefix)
        self.panda = PandaMove("panda_arm", ns=panda_prefix, robot_description=panda_description)
        # Relative stable Poses to work from (use joint angles and relative Pose) 
        self.pandaRelative = []
        
        # von oben
        p1 = MyPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                          -0.023065127793, 0.0352011934197))
        a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
              2.762937863667806, 0.815807519980951]       
        self.pandaRelative.append(PandaGoals(p1, a1))
        
        # von links
        p2 = MyPose((0.447684206665, 0.143218696903, -0.168640488851), (0.651378340986, 0.308604768232,
                                                                        -0.292953425794, 0.62820987276))
        a2 = [1.2237364689655472, 1.750162085129698, -0.6800040184991402, -1.4698347165208097, -0.14583960898717244,
              1.857775869356261, 0.05098501096834743]
        self.pandaRelative.append(PandaGoals(p2, a2))
        
        # von vorne
        p3 = MyPose((0.44152835008, -0.0731349874596, -0.274781670372), (0.294601267058, 0.655450407444, 
                                                                          0.237834588757, 0.653474992039))
        a3 = [-0.3430143268756699, 1.7191940036238282, 0.09120028918354135, -1.5264221894163197, 2.862566652389678,
              1.5280684216684766, 0.65590209259636892]
        self.pandaRelative.append(PandaGoals(p3, a3))
        
        
        
        pandaBasePose = self.listener.lookupTransform('/miranda/mir/base_link', '/miranda/panda/panda_link0',rospy.Time(0))
        self.pandaBaseRel = MyPose(pandaBasePose[0], pandaBasePose[1])

    def calculateMirGrippingPose(self, gripPose=MyPose(), pose_num=0):
        mirPose = MyPose()
        mirPose.position = gripPose.position - (self.pandaRelative[pose_num].pose_relative.position + self.pandaBaseRel.position)
        mirPose.position.z = 0
        # mirPose.orientation = gripPose.orientation - self.pandaRelative[pose_num].pose_relative.orientation # self.pandaBaseRel.orientation
        mirPose.orientation = gripPose.orientation - self.pandaRelative[pose_num].pose_relative.orientation - self.pandaBaseRel.orientation
        mirPose.orientation.x = 0
        mirPose.orientation.y = 0
        # mirPose.orientation.w = 0

        return mirPose

    def movePanda(self, pose_num=0):
        target = self.pandaRelative[pose_num].axis_goal
        self.movePandaAxis(target)
        
    def movePandaAxis(self, target=None):
        if target is None:
            # target = [-0.198922703533319, 0.5, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
            #   2.762937863667806, 0.815807519980951]
            target = [0.0740422346346211, -0.20232101289978627, 0.08934749049583862, -0.7155832671282583, 
                      -0.034721063966157525, 3.480999345766173, 0.9134598995556333]
        self.panda.move_group.set_joint_value_target(target)
        plan = self.panda.move_group.plan()
        # plan = panda_robot.velocity_scale(plan, 0.9)
        self.panda.move_group.execute(plan, wait=True)


if __name__ == '__main__':
    
    rospy.init_node("MirandaMoveGrip")

    # if len(sys.argv) > 1:
    # mir_prefix = sys.argv[1]    # /miranda/mir
    if len(sys.argv) > 1:
        ns = sys.argv[1].__str__()
        mir_prefix = ns+"/mir"
        panda_prefix = ns+"/panda"
        panda_description = ns+"/panda/robot_description"
        miranda = MirandaNav2Goal(mir_prefix, panda_prefix, panda_description)
    else:
        mir_prefix = "/miranda/mir"
        panda_prefix = "/miranda/panda"
        panda_description = "/miranda/panda/robot_description"
        try:
            miranda = MirandaNav2Goal(mir_prefix, panda_prefix, panda_description)
        except RuntimeError:
            mir_prefix = ""
            panda_prefix = ""
            panda_description = "robot_description"
            rospy.loginfo("No prefix is set.")
            miranda = MirandaNav2Goal(mir_prefix, panda_prefix, panda_description)

    rate = rospy.Rate(0.5)
    
    miranda.movePandaAxis()

    pose_num = 2
    while not rospy.is_shutdown():
        if not miranda.is_ready():
            continue
        if miranda.status == 4:
            rospy.loginfo("Goal not reachable!")

        rospy.loginfo("Status is: " + str(miranda.status))
        # my_nav.getSendGoal()
        
        miranda.movePandaAxis() 

        goal_pos = miranda.getGoalCommandLine(False)
        mir_pose = miranda.calculateMirGrippingPose(goal_pos. pose_num)
        rospy.loginfo("Mir gripping Pose is: ")
        rospy.loginfo(mir_pose)
        miranda.sendGoalPos(mir_pose)
        time.sleep(0.5)
        while not miranda.is_ready():
            time.sleep(0.1)
        miranda.movePanda(pose_num)
        # miranda.sendGoalPos(goal_pos)
        
        # posePanda_goal = miranda.pandaRelative[0].calcRelGoal(miranda.getMirPose()+miranda.pandaBaseRel, goal_pos)
        # print("Panda Goal is: ")
        # print(posePanda_goal)
        # miranda.panda.movePose(posePanda_goal)
        # rospy.loginfo("GoalPose is: ")
        # rospy.loginfo(goal_pos)
        
        print("Panda: Moving remaining dist ")
        time.sleep(0.5)
        
        miranda.panda.movePoseTotal(goal_pos)
        # rate.sleep()
        time.sleep(0.1)
