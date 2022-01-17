#! /usr/bin/env python3
# coding=utf-8
import os
import time
import shutil
import math

import rospy
import numpy as np
import cv2
import yaml

from ars408_msg.msg import RadarPoint, RadarPoints
from std_msgs.msg import Float32, String


# load config
with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

first_radar = config['topic_Radar1']
second_radar = config['topic_Radar2']
third_radar = config['topic_Radar3']

radar2_y = config['radar2_ytrans']
radar3_y = config['radar3_ytrans']
radar2_rad = config['radar2_rad']
radar3_rad = config['radar3_rad']

frameRate = config['frameRate']

# Custom Class
class MyRadars(): 
    def __init__(self, radarChannel):
        self.radarPoints = []
        self.radarChannel = radarChannel

    def rotate(self):
        """
        [Rotate Second and Third Radar]
        Rotate a point counterclockwise by a given angle around a given origin point.
        The angle should be given in radians.
        """

        if len(self.radarPoints) == 0:
            return
        
        if self.radarChannel == 2:
            rad = radar2_rad
            y = radar2_y
        elif self.radarChannel == 3:
            rad = radar3_rad
            y = radar3_y
        else:
            return

        for rPt in self.radarPoints:
            rPt.angle = rPt.angle + rad # * 180 / math.pi
            """
            Matrix
            [cos, -sin, tx]
            [sin, cos,  ty]
            [0,      0, 1]
            """
            
            mat_rotate_dist = np.array([[math.cos(rad), -1 * math.sin(rad)],
                                        [math.sin(rad), math.cos(rad)],])
            
            mat_trans_dist = np.array([[1, 0, 0],
                                       [0, 1, y],])
            
            src_dist = np.array([rPt.distX, rPt.distY])
            
            mat_vrel = np.array([[math.cos(rad), -1 * math.sin(rad)],
                                   [math.sin(rad), math.cos(rad)]])
            
            src_vrel = np.array([rPt.vrelX, rPt.vrelY])
            
            dist = np.dot(np.dot(src_dist, mat_rotate_dist), mat_trans_dist)
            
            vrel = np.dot(src_vrel, mat_vrel)
            
            rPt.distX = dist[0]
            rPt.distY = dist[1]
            rPt.vrelX = vrel[0]
            rPt.vrelY = vrel[1]


# Subscriber callback
def callbackPoint1(data):
    """
    [First Radar]
    """
    global points1
    points1.radarPoints = data.rps    

def callbackPoint2(data):
    """
    [Second Radar]
    """
    global points2, pub2_rotate
    points2.radarPoints = data.rps
    pub2_rotate = False
    points2.rotate()
    pub2_rotate = True
    

def callbackPoint3(data):
    """
    [Third Radar]
    """
    global points3, pub3_rotate
    points3.radarPoints = data.rps
    pub3_rotate = False
    points3.rotate()
    pub3_rotate = True


def listener():

    global points1, points2, points3, pub2_rotate, pub3_rotate
    points1 = MyRadars(1)
    points2 = MyRadars(2)
    points3 = MyRadars(3)
    pub2_rotate = False
    pub3_rotate = False
    
    rospy.init_node("threeRadar")
    rosrate = rospy.Rate(frameRate / 2)

    print("Subscribe1: {}".format(first_radar))
    sub1 = rospy.Subscriber(first_radar, RadarPoints,
                            callbackPoint1, queue_size=1)

    print("Subscribe2: {}".format(second_radar))
    sub2 = rospy.Subscriber(second_radar, RadarPoints,
                            callbackPoint2, queue_size=1)

    print("Subscribe3: {}".format(third_radar))
    sub3 = rospy.Subscriber(third_radar, RadarPoints,
                            callbackPoint3, queue_size=1)
    
    pub1 = rospy.Publisher(first_radar + "RT", RadarPoints, queue_size=1)
    pub2 = rospy.Publisher(second_radar + "RT", RadarPoints, queue_size=1)
    pub3 = rospy.Publisher(third_radar + "RT", RadarPoints, queue_size=1)
    
    while not rospy.is_shutdown():
        if not ("points1" in globals() and "points2" in globals() and "points3" in globals()):
        # if not ("points1" in globals() and "points2" in globals()):
            continue
        
        if len(points1.radarPoints) != 0:
            pub1.publish(points1.radarPoints)
            points1.radarPoints.clear()
            
        if pub2_rotate and len(points2.radarPoints) != 0:
            pub2.publish(points2.radarPoints)
            points2.radarPoints.clear()
        
        if pub3_rotate and len(points3.radarPoints) != 0:
            pub3.publish(points3.radarPoints)
            points3.radarPoints.clear()
        
        rosrate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
