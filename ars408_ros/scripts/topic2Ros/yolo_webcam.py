#! /usr/bin/env python3
# coding=utf-8

from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import Bboxes, Bbox
import rospy
import time
import os, sys

sys.path.append(os.path.expanduser("~") + "/Documents/YOLO3-4-Py")
os.chdir(os.path.expanduser("~") + "/Documents/YOLO3-4-Py")

import pydarknet
from pydarknet import Detector, Image
import cv2

frameRate = 30
topic_RGB = "/rgbImg"
size_RGB = (800, 600)
global nowImg_RGB

def callback_RGBImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global nowImg_RGB
    nowImg_RGB = img.copy()


def listener():
    # Optional statement to configure preferred GPU. Available only in GPU version.
    # pydarknet.set_cuda_device(0)

    rospy.init_node("yolo")
    rate = rospy.Rate(frameRate)
    sub_RGB = rospy.Subscriber(topic_RGB, Image, callback_RGBImg, queue_size=1)
    pub_yolo = rospy.Publisher("/yoloImg", Image, queue_size=1)
    pub_bbox = rospy.Publisher("/Bbox", Bboxes, queue_size=1)
    bridge = CvBridge()
    net = Detector(bytes("cfg/yolov3.cfg", encoding="utf-8"), bytes("weights/yolov3.weights", encoding="utf-8"), 0,
                   bytes("cfg/coco.data", encoding="utf-8"))

    while not rospy.is_shutdown():
        if not ("nowImg_RGB"  in globals()):
            continue

        RGBImg = cv2.resize(nowImg_RGB, size_RGB, cv2.INTER_CUBIC)
        start_time = time.time()

        # Only measure the time taken by YOLO and API Call overhead

        dark_frame = Image(RGBImg)
        results = net.detect(dark_frame)
        del dark_frame

        end_time = time.time()
        # Frames per second can be calculated as 1 frame divided by time required to process 1 frame
        fps = 1 / (end_time - start_time)
        
        print("FPS: ", fps)
        print("Elapsed Time:",end_time-start_time)

        BB = Bboxes()
        for cat, score, bounds in results:
            x, y, w, h = bounds
            tempBB = Bbox()
            tempBB.x_min = int(x - w/2)
            tempBB.y_min = int(y - h/2)
            tempBB.x_max = int(x + w/2)
            tempBB.y_max = int(y + h/2)
            tempBB.objClass = str(cat.decode("utf-8"))
            BB.bboxes.append(tempBB)
            cv2.rectangle(RGBImg, (int(x - w/2), int(y - h/2)), (int(x + w/2), int(y + h/2)), (255, 0, 0))
            cv2.putText(RGBImg, str(cat.decode("utf-8")), (int(x - w/2 - 5), int(y - h/2 - 5)), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 0), 1, cv2.LINE_AA)

        img_message = bridge.cv2_to_imgmsg(RGBImg)
        pub_yolo.publish(img_message)
        pub_bbox.publish(BB)
        
        rate.sleep()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass