#! /usr/bin/env python3
# coding=utf-8
import os
import math

import rospy
import numpy as np
import yaml

from ars408_msg.msg import RadarPoint, RadarPoints
from std_msgs.msg import Float32, String


# load config
with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']


# Custom Class
class MyRadars(): 
    def __init__(self, radarChl):
        self.radarPoints = []
        self.radarChannel = radarChl


# Subscriber callback
def callbackPoint1(data):
    global radarPt
    
    radarPt.radarPoints = data.rps
    

def listener(radarChl):
    
    rospy.init_node("AEB")
    rosrate = rospy.Rate(frameRate)
    
    global radarPt
    radarPt = MyRadars(radarChl)
    
    # Subscribe RadarPubRT
    print("AEB Subscribe: {}".format(radarChl))
    sub1 = rospy.Subscriber(radarChl + "/radarPubRT", RadarPoints,
                            callbackPoint1, queue_size=1)
    
    # Publish RadarTraj
    # pub1 = rospy.Publisher(first_radar + "RT", RadarPoints, queue_size=1)
    
    while not rospy.is_shutdown():
        if not ("radarPt" in globals()):
            continue
        
        
        rosrate.sleep()


if __name__ == "__main__":
    
    try:
        # Get Param
        radarChannel = rospy.get_param('AEB/radarChannel')
        
        if radarChannel != None:
            listener(radarChannel)
        
    except rospy.ROSInternalException:
        pass