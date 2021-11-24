#! /usr/bin/env python3
# coding=utf-8
import os
import time
import shutil

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
topic_Radar = config['topic_Radar']

dp_idx = 0

# Custom Class
class MyRadars():
    def __init__(self):
        self.radarPoints = []
    
    def __eq__(self, anotherRadar):
        if isinstance(anotherRadar, MyRadars):
            for myPt in self.radarPoints:
                for anotherPt in anotherRadar.radarPoints:
                    if anotherPt.distX == myPt.distX and anotherPt.vrelX == myPt.vrelX and\
                        anotherPt.distY == myPt.distY and anotherPt.vrelY == myPt.vrelY and\
                        anotherPt.width == myPt.width and anotherPt.height == myPt.height:
                            print("="*50)
                            print(myPt)
                            print("-"*40)
                            print(anotherPt)
                            return True
            
        return False



# Subscriber callback
def callbackPoint1(data):
    global points1
    points1.radarPoints = data.rps


def callbackPoint2(data):
    global points2
    points2.radarPoints = data.rps


def callbackPoint3(data):
    global points3
    points3.radarPoints = data.rps

# functions
def checkDuplicate():
    global points1, points2, points3

    if points1 == points2 or points1 == points3 or points2 == points3:
        return True

    return False


def listener():

    global points1, points2, points3
    points1 = MyRadars()
    points2 = MyRadars()
    points3 = MyRadars()
    
    rospy.init_node("threeRadar")
    rosrate = rospy.Rate(20)
    

    print("Subscribe1: {}".format(first_radar))
    sub1 = rospy.Subscriber(first_radar, RadarPoints,
                            callbackPoint1, queue_size=1)

    print("Subscribe2: {}".format(second_radar))
    sub1 = rospy.Subscriber(second_radar, RadarPoints,
                            callbackPoint2, queue_size=1)

    print("Subscribe3: {}".format(third_radar))
    sub1 = rospy.Subscriber(third_radar, RadarPoints,
                            callbackPoint3, queue_size=1)
    

    while not rospy.is_shutdown():
        if not ("points1" in globals() and "points2" in globals() and "points3" in globals()):
            continue

        checkDuplicate()

        rosrate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
