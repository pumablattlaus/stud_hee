#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

x, y = 0, 0
yaw = 0


def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


def rotate(velocity_publisher, speed_deg, angle_deg, counter_cw=True):
    velocity_message = Twist()

    speed = math.radians(speed_deg)
    angle = abs(math.radians(angle_deg))
    if counter_cw:
        velocity_message.angular.z = abs(speed)
    else:
        velocity_message.angular.z = -abs(speed)

    angle_rotated = 0.0
    t0 = rospy.rostime.get_time()
    loop_rate = rospy.Rate(10)

    while True:
        rospy.loginfo("Turtlesim rotates around z")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        t1 = rospy.rostime.get_time()
        angle_rotated = (t1 - t0) * speed
        print(f"angle_rotated: {angle_rotated}")
        if angle_rotated >= angle:
            rospy.loginfo("reached")
            break

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def move(velocity_publisher, speed, distance, is_forward=True):
    # declare Twist message to send velocity commands:
    velocity_message = Twist()

    global x, y
    x0 = x
    y0 = y

    if is_forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)

    while True:
        rospy.loginfo("Turtlesim moves in linear x")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = abs(math.sqrt((x - x0) ** 2 + (y - y0) ** 2))
        print(distance_moved)
        if distance_moved >= distance:
            rospy.loginfo("reached")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def move_goal(velocity_publisher, x_goal, y_goal):
    velocity_message = Twist()

    global x, y, yaw

    loop_rate = rospy.Rate(10)

    while True:
        K_lin = 0.5
        d = abs(math.sqrt((x - x_goal) ** 2 + (y - y_goal) ** 2))
        speed_lin = K_lin * d

        k_ang = 3
        ang = math.atan2(y_goal - y, x_goal - x)
        speed_ang = k_ang * (ang - yaw)

        velocity_message.linear.x = speed_lin
        velocity_message.angular.z = speed_ang

        velocity_publisher.publish(velocity_message)

        if d <= 0.01:
            break
    print("Reached goal pos")


def set_desired_orientation(vel_publisher, speed_deg, heading_deg):
    global yaw
    heading = math.radians(heading_deg)
    angle = (heading - yaw + math.pi) % (2*math.pi) - math.pi

    if angle > 0:
        clockwise = True
    else:
        clockwise = False

    rotate(vel_publisher, speed_deg, math.degrees(angle), clockwise)

if __name__ == "__main__":
    rospy.init_node("turtlesim_motion_pose", anonymous=False)

    cmd_vel_topic = 'mobile_base_controller/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    position_topic = "/robot_pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
    time.sleep(2)

    # move(velocity_publisher, 1.0, 2.0, True)
    # rotate(velocity_publisher, 90.0, 360.0, True)

    # move_goal(velocity_publisher, 5, 2)
    set_desired_orientation(velocity_publisher, 10, -179)

    while True:
        try:
            rotate(velocity_publisher, 10, 360)
            time.sleep(0.1)
        except:
            break
