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

# Custom Class
class MyRadars():
    def __init__(self, radarChannel):
        self.radarPoints = []
        self.radarChannel = radarChannel
    
    def __eq__(self, anotherRadar, thresh=1):
        """
        [Override Operator ==]

        Args:
            anotherRadar ([MyRadar]): [Customize Radar Class]
            thresh ([float]): [Threshold of value diff]

        Returns:
            [Boolean]: [two class is equal]
        """
        if isinstance(anotherRadar, MyRadars):
            for myPt in self.radarPoints:
                for anotherPt in anotherRadar.radarPoints:
                    if abs(anotherPt.distX - myPt.distX) < thresh  and abs(anotherPt.distY - myPt.distY) < thresh and\
                            abs(anotherPt.width - myPt.width) < thresh and abs(anotherPt.height - myPt.height) < thresh:
                            # abs(anotherPt.vrelX - myPt.vrelX) < thresh and abs(anotherPt.vrelY - myPt.vrelY) < thresh:
                            # print("="*50)
                            # print("channel:", self.radarChannel)
                            # print(myPt)
                            # print("-"*50)
                            # print("channel: ", anotherRadar.radarChannel)
                            # print(anotherPt)
                            # print("="*50)
                            pass
                            
        return False

    
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
            rad = 0
            y = 0

        # print("="*50)
        # print("Before rotate")
        # print("vrelX: ", self.radarPoints[0].vrelX)
        # print("vrelY: ", self.radarPoints[0].vrelY)
        # print("Angle: ", self.radarPoints[0].angle)
        
        
        for rPt in self.radarPoints:
            rPt.angle = rPt.angle + rad # * 180 / math.pi
            """
            Matrix
            [cos, -sin, tx]
            [sin, cos,  ty]
            [0,      0, 1]
            """
            
            mat_dist = np.array([[math.cos(rad), -1 * math.sin(rad), 0],
                                 [math.sin(rad), math.cos(rad), y], 
                                 [0.0, 0.0, 1.0]])
            
            src_dist = np.array([rPt.distX, rPt.distY, 1])
            
            mat_vrel = np.array([[math.cos(rad), -1 * math.sin(rad)],
                                   [math.sin(rad), math.cos(rad)]])
            
            src_vrel = np.array([rPt.vrelX, rPt.vrelY])
            
            
            dist = np.matmul(src_dist, mat_dist)
            vrel = np.matmul(mat_vrel, src_vrel)
            
            rPt.distX = dist[0]
            rPt.distY = dist[1]
            rPt.vrelX = vrel[0]
            rPt.vrelY = vrel[1]

        
        # print("After rotate")
        # print("vrelX: ", self.radarPoints[0].vrelX)
        # print("vrelY: ", self.radarPoints[0].vrelY)
        # print("Angle: ", self.radarPoints[0].angle)


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
    global points2
    points2.radarPoints = data.rps
    points2.rotate()

def callbackPoint3(data):
    """
    [Third Radar]
    """
    global points3
    points3.radarPoints = data.rps
    points3.rotate()

# functions
def filterCloseRange():
    global points1, points2, points3
    
    # merge all points
    all_points = []
    all_points = points1.radarPoints + points2.radarPoints + points3.radarPoints
    print("Before filter:", len(all_points))
    
    filter_points = []
    filter_index = []
    v_i = [0.1, 0.02, 0.002, 0.00004]
    N_i = [2, 3, 4, 10]
    thresh = 1.4
    
    # filter radar points with close range
    for i in range(len(all_points)):
        N_d = 0
        for j in range(i+1, len(all_points)):
            d = pow(pow(all_points[i].distX - all_points[j].distX, 2) +
                    pow(all_points[i].distY - all_points[j].distY, 2), 0.5)
            if d < thresh:
                N_d += 1
                filter_index.append(i)

        # if N_d < 1:
        #     filter_index.append(i)
        # else:
        #     for j in range(len(v_i)):
        #         v = pow(pow(all_points[i].vrelX, 2) +
        #                 pow(all_points[i].vrelY, 2), 0.5)
        #         if v < v_i[j] and N_d < N_i[j]:
        #             filter_index.append(i)
        #             break

    for i in range(len(all_points)):
        if not i in filter_index:
            filter_points.append(all_points[i])
    
    print("After:", len(filter_points))
    

def checkDuplicate():
    global points1, points2, points3

    if points1 == points2 or points1 == points3 or points2 == points3:
        return True

    return False


def listener():

    global points1, points2, points3
    points1 = MyRadars(1)
    points2 = MyRadars(2)
    points3 = MyRadars(3)

    rospy.init_node("threeRadar")
    rosrate = rospy.Rate(20)

    print("Subscribe1: {}".format(first_radar))
    sub1 = rospy.Subscriber(first_radar, RadarPoints,
                            callbackPoint1, queue_size=1)

    print("Subscribe2: {}".format(second_radar))
    sub2 = rospy.Subscriber(second_radar, RadarPoints,
                            callbackPoint2, queue_size=1)

    print("Subscribe3: {}".format(third_radar))
    sub3 = rospy.Subscriber(third_radar, RadarPoints,
                            callbackPoint3, queue_size=1)

    while not rospy.is_shutdown():
        if not ("points1" in globals() and "points2" in globals() and "points3" in globals()):
            continue

        checkDuplicate()
        # filterCloseRange()
        rosrate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
