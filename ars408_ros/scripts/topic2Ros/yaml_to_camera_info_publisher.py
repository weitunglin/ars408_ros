#! /usr/bin/env python3
# coding=utf-8

import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["camera_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/scripts/topic2Ros/head_camera.yaml")

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("/camera_info", CameraInfo, queue_size=1)
    rate = rospy.Rate(20)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()