#! /usr/bin/env python3
# coding=utf-8
import os
import math

import rospy
import numpy as np
import yaml

from ars408_msg.msg import RadarPoint, RadarPoints, GPSinfo
from std_msgs.msg import Float32, String


# load config
with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']
topic_GPS = config['topic_GPS']

# Custom Class
class MyGPS(): 
    def __init__(self, radarChl):
        self.gps = []


# Subscriber callback
def callbackPoint1(data):
    global GPS_Info


def listener(radarChl):
    
    rospy.init_node("AEB")
    rosrate = rospy.Rate(frameRate)
    
    global GPS_Info
    GPS_Info = MyGPS(radarChl)
    
    # Subscribe RadarPubRT
    print("GPS Subscribe: {}".format(radarChl))
    sub1 = rospy.Subscriber(topic_GPS, GPSinfo,
                            callbackPoint1, queue_size=1)
    
    # Publish
    pub1 = rospy.Publisher("/FUNC/", RadarPoints, queue_size=1)
    
    while not rospy.is_shutdown():
        if not ("gpsInfo" in globals()):
            continue
        
        
        rosrate.sleep()


if __name__ == "__main__":
    
    try:
        # Get Param
        # radarChannel = rospy.get_param('AEB/radarChannel')

        # if radarChannel != None:
        #     listener(radarChannel)
        pass
        
    except rospy.ROSInternalException:
        pass