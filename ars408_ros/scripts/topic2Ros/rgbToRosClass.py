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

rgbIndex = config['rgbIndex']
topic_RGB = config['topic_RGB']
frameRate = config['frameRate']
codec = cv2.VideoWriter_fourcc(*'MJPG')

w, h = 1280, 720
rgbcam = cv2.VideoCapture(rgbIndex)
# rgbcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
rgbcam.set(cv2.CAP_PROP_FRAME_WIDTH, w)
rgbcam.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
rgbcam.set(cv2.CAP_PROP_FOURCC, codec)
rgbcam.set(cv2.CAP_PROP_FPS, frameRate)
bridge = CvBridge()

class rgbSensor:
    def readData(self, event=None):
        # Here you read the data from your sensor
        # And you return the real value
        rgbret, rgb_image = rgbcam.read()
        self.data = bridge.cv2_to_imgmsg(rgb_image)

    def __init__(self):
        # Create a ROS publisher
        self.publisher = rospy.Publisher(topic_RGB, Image, queue_size=1)
        # Initialize data
        self.data = Image()

    def publishData(self, event=None):
        self.publisher.publish(self.data)


if __name__ == "__main__":
    rospy.init_node("rgbToRos")

    # Create an instance of sensor
    rgb = rgbSensor()
    readDuration = 1.0 / frameRate
    pubDuration = 1.0 / frameRate

    # Create a ROS Timer for reading data
    rospy.Timer(rospy.Duration(readDuration), rgb.readData)

    # Create another ROS Timer for publishing data
    rospy.Timer(rospy.Duration(pubDuration), rgb.publishData)
    
    # Don't forget this or else the program will exit
    rospy.spin()
