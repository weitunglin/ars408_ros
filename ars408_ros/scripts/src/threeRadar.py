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
with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
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

# DEBUG FLAG
SHOW_FILTER = False
SHOW_LOG = False

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
            
            mat_rotate_dist = np.array([[math.cos(rad), -1 * math.sin(rad), 0],
                                        [math.sin(rad), math.cos(rad), 0],
                                        [0, 0, 1]])
            mat_trans_dist = np.array([[1, 0, 0],
                                       [0, 1, y],
                                       [0, 0, 1]])
            src_dist = np.array([rPt.distX, rPt.distY, 1])
            mat_vrel = np.array([[math.cos(rad), -1 * math.sin(rad)],
                                   [math.sin(rad), math.cos(rad)],])
            src_vrel = np.array([rPt.vrelX, rPt.vrelY])
            
            dist = np.dot(np.dot(src_dist, mat_rotate_dist), mat_trans_dist)
            # dist = np.dot(np.dot(src_dist, mat_trans_dist), mat_rotate_dist)
            
            vrel = np.dot(src_vrel, mat_vrel)
            
            rPt.distX = dist[0]
            rPt.distY = dist[1]
            rPt.vrelX = vrel[0]
            rPt.vrelY = vrel[1]


# Functions
def filterRadar():
    """
    Filter those close range and same direction.
    """
    global points1, points2, points3

    if len(points1.radarPoints) == 0:
        return
    
    closeRange = 5
    angleThresh = 3
    
    for rpt1 in points1.radarPoints:
        # radar1 and radar2
        for i2 in range(len(points2.radarPoints)-1, -1, -1):
            rpt2 = points2.radarPoints[i2]
            if math.sqrt((rpt1.distX - rpt2.distX)**2 + (rpt1.distY - rpt2.distY)**2) < closeRange and \
                abs(math.atan2(rpt1.vrelY, rpt1.vrelX) - math.atan2(rpt2.vrelY, rpt2.vrelX)) < angleThresh:
                if SHOW_LOG:
                    print("filter point pt1X:{} pt2X:{} pt1Y:{} pt2Y:{}".format(rpt1.distX, rpt2.distX, rpt1.distY, rpt2.distY))
                points2.radarPoints.pop(i2)
        
        # radar1 and radar3
        for i3 in range(len(points3.radarPoints)-1, -1, -1):
            rpt3 = points3.radarPoints[i3]
            if math.sqrt((rpt1.distX - rpt3.distX) **2 + (rpt1.distY - rpt3.distY)**2) < closeRange and \
                abs(math.atan2(rpt1.vrelY, rpt1.vrelX) - math.atan2(rpt3.vrelY, rpt3.vrelX)) < angleThresh:
                if SHOW_LOG:
                    print("filter point pt1X:{} pt3X:{} pt1Y:{} pt3Y:{}".format(rpt1.distX, rpt3.distX, rpt1.distY, rpt3.distY))
                points3.radarPoints.pop(i3)

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
    rosrate = rospy.Rate(frameRate)

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
        
        if SHOW_FILTER:
            print("PT2 Before Filter: {}".format(len(points2.radarPoints)))
            print("PT3 Before Filter: {}".format(len(points3.radarPoints)))
        filterRadar()
        
        if SHOW_FILTER:
            print("PT2 AFTER Filter: {}".format(len(points2.radarPoints)))
            print("PT3 AFTER Filter: {}".format(len(points3.radarPoints)))
            print("="*50)
        
        if pub3_rotate and len(points3.radarPoints) != 0 and pub2_rotate and len(points2.radarPoints) != 0 and len(points1.radarPoints) != 0:
            pub3.publish(points3.radarPoints)
            pub2.publish(points2.radarPoints)
            pub1.publish(points1.radarPoints)
            
            points3.radarPoints.clear()
            points2.radarPoints.clear()
            points1.radarPoints.clear()

        rosrate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
