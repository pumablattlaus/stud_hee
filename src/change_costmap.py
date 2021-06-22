#!/usr/bin/env python
import enum
import math
import time

from numpy.core.fromnumeric import shape, size
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import rospy
import numpy as np
from pynput import keyboard
import sys
import json

# NEW LAUNCH FILE IS NEEDED! (mir)

class Obsticle(object):
    """ Obsticle class:
    functions for adding the defined obsticle to a costmap."""
    def __init__(self, x_start=0, y_start=0, map=[[0],[0]]):
        """ x/y_start: position of map defined by 2-dim-matrix"""
        self.x_start = x_start
        self.y_start = y_start
        self.map = map
        self.x_max = self.x_start + len(map[0])
        self.y_max = self.y_start + len(map[1])

    def add_obsticleToUpdate(self, gridUpdate):
        """adds the obsticle, defined by map and position to the costmap updates.
        Has to be called by Topic-Subscription. Changed map has to be published to topic CostmapUpdates"""
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
        """not working properly and obsolete. Only add_obsticleToUpdate is used"""
        data = list(map.data)
        for i in range(size(data,0)):
            for j in range(size(data,1)):

                if (self.x_start <= size(data,0) < self.x_max) or (self.y_start <= size(data,1) < self.y_max) :
                     data[i][j] = self.map[i-self.x_start][j-self.y_start]
        
        new_map = OccupancyGrid(map.header, map.info, data)

        return new_map

    def move(self, down):
        """boolean arguments for direction"""
        if down:
            print("Down")
        

def subCallback(map, obsticle):
    global my_publisher
    rospy.loginfo("publishing")
    print(shape(map.data))
    #map = obsticle.add_obsticleToMap(map)
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

def on_press(key):
    global my_obsticle
    """register key presses"""
    try:
        #print(key.char)
        if key == keyboard.Key.down:
            my_obsticle.move(1,0)
    
    except AttributeError:
        rospy.loginfo('special key {0} pressed'.format(key))



if __name__ == '__main__':
    rospy.init_node('change_costmap', anonymous=False)

    if len(sys.argv) >= 2:
        obsticle_file = sys.argv[1]
        with open(obsticle_file) as f:
            data = json.load(f)
            obsticle_map = data['map']
            obsticle_x = data['x']
            obsticle_y = data['y']
            rospy.loginfo("read json. x=%d, y= %d", obsticle_x, obsticle_y)

        if len(sys.argv) >= 4:
            obsticle_x = int(sys.argv[2])
            obsticle_y = int(sys.argv[3])
            rospy.loginfo("changed position: x=%d, y= %d", obsticle_x, obsticle_y)
    else:
        obsticle_map = np.ones((10, 5), dtype=int)*100  # 20 damit sichtbarer Unterschied. eig 100
        obsticle_x = 150
        obsticle_y = 450
        with open('test.json', 'w+') as f:
            json_construct = {'map': obsticle_map.tolist(), 'x': obsticle_x, 'y': obsticle_y}
            json.dump(json_construct, f)

    my_obsticle = Obsticle(x_start=obsticle_x, y_start=obsticle_y, map=obsticle_map)

    # # Costmap
    # pub_topic = "mycostmap/costmap"
    # my_publisher = rospy.Publisher(pub_topic, OccupancyGrid, queue_size=10)
    # sub_topic_map = "/move_base_node/global_costmap/costmap"
    # map_subscriber = rospy.Subscriber(sub_topic_map, OccupancyGrid, subCallback, callback_args=my_obsticle)

    # Costmap Updates:
    pub_topic_update = "/move_base_node/global_costmap/costmap_updates"
    update_publisher = rospy.Publisher(pub_topic_update, OccupancyGridUpdate, queue_size=10)
    sub_topic_update = "/mycostmap_update"
    update_subscriber = rospy.Subscriber(sub_topic_update, OccupancyGridUpdate, subUpdateCallback, callback_args=my_obsticle)

    # move_down = False
    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()
    # keyboard.on_press_key('down', lambda move_down: True)
    # keyboard.on_release_key('down', lambda move_down: False)
    loop_rate = rospy.Rate(10)
    rospy.spin()
# while not rospy.is_shutdown():
#     test=input("Test: ")
#     print(test)
#     my_obsticle.move(move_down)
#     loop_rate.sleep()

