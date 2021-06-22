#!/usr/bin/env python
import enum
import math
import time

from numpy.core.fromnumeric import shape, size
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import rospy
import numpy as np

class Obsticle(object):
    def __init__(self, x_start=0, y_start=0, map=[[0],[0]]):
        self.x_start = x_start
        self.y_start = y_start
        self.map = map
        self.x_max = self.x_start + len(map[0])
        self.y_max = self.y_start + len(map[1])

    def add_obsticleToUpdate(self, gridUpdate):
        w = gridUpdate.width
        h = gridUpdate.height
        data = list(gridUpdate.data)
        num_data = len(data)
        for i in range(num_data):
            column = i%w + gridUpdate.x
            row = int(i/w)+gridUpdate.y
            
            if (self.x_start <= column < self.x_max) and (self.y_start <= row < self.y_max) :
                data[i] = self.map[column-self.x_start][row-self.y_start]
                

        new_update = OccupancyGridUpdate(gridUpdate.header, gridUpdate.x, gridUpdate.y, w, h, data)

        return new_update

    def add_obsticleToMap(self, map):
        data = list(map.data)
        for i in range(len(data[0])):
            for j in range(len(data[1])):

                if (not self.x_start <= data[0][i] < self.x_max) or (not self.y_start <= data[1][j] <= self.y_max) :
                    continue
                else:
                    data[i][j] = 100
        
        new_map = OccupancyGrid(map.header, map.info, data)

        return new_map

    def move(x,y):
        pass
        

def subCallback(map, obsticle):
    global my_publisher
    rospy.loginfo("publishing")
    print(shape(map.data))
    # map = obsticle.add_obsticleToMap(map)
    my_publisher.publish(map)
    # print(map.data)
    rospy.loginfo("Laenge der Map ist %d", len(map.data))
    rospy.loginfo("published map")

# def subUpdateCallback(gridUpdate):
def subUpdateCallback(gridUpdate, obsticle):
    global update_publisher
    rospy.loginfo("UPdate")
    # print(gridUpdate.data)
    gridUpdate = obsticle.add_obsticleToUpdate(gridUpdate)

    # for i in range(len(gridUpdate.data)):
    #     gridUpdate.data[i] = 99

    update_publisher.publish(gridUpdate)
    print("Laenge ist: ")
    print(len(gridUpdate.data))
    rospy.loginfo("x= %d, y= %d", gridUpdate.x, gridUpdate.y)
    
obsticle_map = np.ones((50, 50), dtype=int)*99
my_obsticle = Obsticle(x_start=120, y_start=350, map=obsticle_map)

rospy.init_node('change_costmap', anonymous=False)

pub_topic = "mycostmap/costmap"
my_publisher = rospy.Publisher(pub_topic, OccupancyGrid, queue_size=10)

# sub_topic = "mycostmap/costmap"
sub_topic_map = "/move_base_node/global_costmap/costmap"
map_subscriber = rospy.Subscriber(sub_topic_map, OccupancyGrid, subCallback, callback_args=my_obsticle)

pub_topic_update = "/mycostmap_update"
sub_topic_update = "/move_base_node/global_costmap/costmap_updates"
update_publisher = rospy.Publisher(pub_topic_update, OccupancyGridUpdate, queue_size=10)
update_subscriber = rospy.Subscriber(sub_topic_update, OccupancyGridUpdate, subUpdateCallback, callback_args=my_obsticle)

loop_rate = rospy.Rate(10)

rospy.spin()
