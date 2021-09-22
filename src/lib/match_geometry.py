#!/usr/bin/env python
# # coding=latin-1
import copy
import math
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, Pose
import moveit_msgs.msg
from actionlib_msgs.msg import *
import std_msgs.msg as std_msg
import numpy as np
# import quaternion
import tf
from tf import transformations
from tf import ExtrapolationException

class MyPoint(Point):
    def __init__(self, pos=(0.0, 0.0, 0.0)):
        if type(pos) == Point:
            self.asArray = np.array(pos.__reduce__()[2])
        else:
            self.asArray = np.array(pos)
        super(MyPoint, self).__init__(*(self.asArray))

        # 1 hinten anfuegen
        self.asArray = np.append(self.asArray, 0)

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
    def __init__(self, quatern=(0.0, 0.0, 0.0, 1.0)):
        if type(quatern) == Quaternion:
            self.asArray = np.array(quatern.__reduce__()[2])
        else:
            self.asArray = np.array(quatern)
        super(MyOrient, self).__init__(*(self.asArray))

    def __add__(self, o2):
        return MyOrient(transformations.quaternion_multiply(self.asArray, o2.asArray))

    def __sub__(self, o2):
        q_inv = transformations.quaternion_conjugate(o2.asArray)
        return MyOrient(transformations.quaternion_multiply(self.asArray, q_inv))


class MyPose(Pose):
    def __init__(self, pos=(0.0, 0.0, 0.0), quatern=(0.0, 0.0, 0.0, 1.0)):
        point = MyPoint(pos)
        orient = MyOrient(quatern)
        super(MyPose, self).__init__(point, orient)
        self.position = point
        self.orientation = orient

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
    
    def rotateVector(self, vec=None, rot=None):
        if vec == None:
            vec = self.position.asArray
        if rot == None:
            rot = self.orientation.asArray
        rot_conj = transformations.quaternion_conjugate(rot)
        trans = transformations.quaternion_multiply(transformations.quaternion_multiply(rot_conj, vec), rot) [:3]
        return MyPoint(trans)


def rotateToXAxis(points, axis=(0,1), transpose = False):
    """Rotate list of points to from X-Axis to new axis 

    Args:
        points ([[[x1, y1]], [[x2,y2]],...]): list of points to rotate
        axis (tuple, optional): new x-Axis. Defaults to (0,1).
        transpose (bool, optional): reverse action. Defaults to False.

    Returns:
        points: rotated
    """
    if points[0] is None: return None

    axis = axis/np.linalg.norm(axis)
    R = np.mat([[axis[0],axis[1]],
                [-axis[1],axis[0]]])
    if transpose: R = R.T
    contour_new = np.array(np.matmul(points,R))
    return contour_new

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
    
    
    p1 = MyPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                        -0.023065127793, 0.0352011934197))
    a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
            2.762937863667806, 0.815807519980951]
    
    panda_goal = PandaGoals(p1, a1)
    print(panda_goal)
