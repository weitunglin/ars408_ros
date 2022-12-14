#! /usr/bin/env python3
# coding=utf-8
import os
import sys

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from easydict import EasyDict as edict

sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/TSR_yolov5")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_package/TSR_yolov5")

from detect_c import TSR


class TrafficSignRecognition():
    def __init__(self):
        self.bridge = CvBridge()

        self.opt = edict(
            weights = ["weights/8_TrafficShape_221009.pt"],
            view_img = True,
            img_size = 640,
            auto_size = 32,
            conf_thres = 0.4,
            iou_thres = 0.5,
            agnostic_nms = False,
            device = "0",
            cfg = "cfg/8_TrafficSignShapes.cfg",
            names = "data/8_TrafficSignShapes.names",
            light_cnn = True,
            weights_light = ["weights/6_TrafficLight-tiny_221026N.pt"],
            img_size_light = 640,
            cfg_light = "cfg/6_TrafficLight-tiny.cfg",
            names_light = "data/6_TrafficLight.names",
            agnostic_nms_light = False,
            conf_thres_light = 0.4,
            iou_thres_light = 0.5,
            speedLimit_cnn = True,
            weights_SpeedLimit = ["weights/SpeedLimit221005.pt"],
            img_size_SpeedLimit = 640,
            cfg_SpeedLimit = "cfg/SpeedLimit.cfg",
            names_SpeedLimit = "data/SpeedLimit.names",
            agnostic_nms_SpeedLimit = False,
            conf_thres_SpeedLimit = 0.4,
            iou_thres_SpeedLimit = 0.5,
        )

        self.model = TSR(self.opt)

        #---sub---#
        self.rgb_name = "front_center"
        self.sub_rgb = rospy.Subscriber("/rgb/" + self.rgb_name + "/calib_image", Image, self.callback)

        #---pub---#
        self.pub = rospy.Publisher("/rgb/" + self.rgb_name + "/tsr_image", Image, queue_size=1)

    def callback(self, image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

        result = self.model(img)

        self.pub.publish(self.bridge.cv2_to_imgmsg(img))

def main():
    rospy.init_node("Traffic Sign Recognition")    

    tsr = TrafficSignRecognition()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
