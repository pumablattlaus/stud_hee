#!/usr/bin/env python
# coding=latin-1

import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
import sys
import time
import math
import numpy as np
from my_functions import MyPose, MyPoint, MyOrient, PandaGoals, PandaMove, MirNav2Goal


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
        p3 = MyPose((0.51120407709, 0.38271167266, -0.0896339517655), (0.284668312985, 0.632243382911, 
                                                                          0.271560550401, 0.667448218077))
        a3 = [1.3034922551576071, 1.575979168256124, -0.7041951051637146, -1.3652462901567157, -1.7017831660051927, 
              2.2569356721625358, 0.7447485772350596]
        self.pandaRelative.append(PandaGoals(p3, a3))
        
        # von vorne, flach/unten
        p4 = MyPose((0.254601632, 0.387543637578, -0.238438655556), (0.328250580377, 0.650865539517, 0.236145165068, 0.642542657701))
        a4 = [1.2570957422876035, 1.6296624805718138, -0.05383580090079391, -1.436617380749945, -1.758743998289108, 1.7004530393013495, 0.755999629455308]
        self.pandaRelative.append(PandaGoals(p3, a3))
        
        
        try:
            pandaBasePose = self.listener.lookupTransform('/miranda/mir/base_link', '/miranda/panda/panda_link0',rospy.Time(0))
        except:
            pandaBasePose = self.listener.lookupTransform('/world', '/panda_link0',rospy.Time(0))
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
        return self.movePandaAxis(target)
        
    def movePandaAxis(self, target=None):
        if target is None:
            # target = [-0.198922703533319, 0.5, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
            #   2.762937863667806, 0.815807519980951]
            target = [-0.23087953991586707, -0.26350456948865925, 0.060693435398110174, -0.6763397448455262, 
                      0.1547569044896521, 2.8163922914174484, 1.33943788516301]
        self.panda.move_group.set_joint_value_target(target)
        plan = self.panda.move_group.plan()
        if len(plan.joint_trajectory.points):
            # plan = panda_robot.velocity_scale(plan, 0.9)
            return self.panda.move_group.execute(plan, wait=True)
        return False
    
    def movePandaReplan(self, pose_num=0):
        res = miranda.movePanda(pose_num)
        if not res:
            print("Panda: Moving to position not possible")
            old_num = pose_num
            while pose_num == old_num:
                pose_num=np.random.randint(0, len(miranda.pandaRelative))
                print("New Pose_Num = " + str(pose_num))
                # Add Stop condition:
                self.movePandaReplan(pose_num)


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

    pose_num = 0
    while not rospy.is_shutdown():
        if not miranda.is_ready():
            continue
        if miranda.status == 4:
            rospy.loginfo("Goal not reachable!")

        rospy.loginfo("Status is: " + str(miranda.status))
        # my_nav.getSendGoal()
        
        miranda.movePandaAxis() 

        goal_pos = miranda.getGoalCommandLine(False)
        mir_pose = miranda.calculateMirGrippingPose(goal_pos, pose_num)
        rospy.loginfo("Mir gripping Pose is: ")
        rospy.loginfo(mir_pose)
        miranda.sendGoalPos(mir_pose)
        time.sleep(0.5)
        while not miranda.is_ready():
            time.sleep(0.1)
        # res = miranda.movePanda(pose_num)
        miranda.movePandaReplan(pose_num)
                
        # miranda.sendGoalPos(goal_pos)
        
        # posePanda_goal = miranda.pandaRelative[0].calcRelGoal(miranda.getMirPose()+miranda.pandaBaseRel, goal_pos)
        # print("Panda Goal is: ")
        # print(posePanda_goal)
        # miranda.panda.movePose(posePanda_goal)
        # rospy.loginfo("GoalPose is: ")
        # rospy.loginfo(goal_pos)
        
        print("Panda: Moving remaining dist ")
        time.sleep(0.5)
        
        miranda.panda.movePoseTotal(goal_pos, linear=True)
        # rate.sleep()
        time.sleep(0.1)
        print("Start again? y/n:  ")
        again = raw_input()
        if not again=="y":
            rospy.signal_shutdown("Aborted by user")
