#! /usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
import cv2
import yaml, os

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

thrIndex = config['thrIndex']
topic_TRM = config['topic_TRM']
frameRate = config['frameRate']

thrcam = cv2.VideoCapture(thrIndex)
thrcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
bridge = CvBridge()

class thermalSensor:
    def readData(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        thrret, thr_image = thrcam.read()
        self.data = bridge.cv2_to_imgmsg(thr_image)

    def __init__(self):
        # Create a ROS publisher
        self.publisher = rospy.Publisher(topic_TRM, Image, queue_size=1)
        # Initialize data
        self.data = Image()

    def publishData(self, event=None):
        self.publisher.publish(self.data)


if __name__ == "__main__":
    rospy.init_node("thermalToRos")

    # Create an instance of sensor
    thr = thermalSensor()
    readDuration = 1.0 / frameRate
    pubDuration = 1.0 / frameRate

    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(readDuration), thr.readData)

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(pubDuration), thr.publishData)
    
    # Don't forget this or else the program will exit
    rospy.spin()
