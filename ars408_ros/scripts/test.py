#! /usr/bin/env python3

from ars408_msg.msg import Tests, Test

import rospy
import math
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

x = []
y = []
pub = None


def callbackData(data):
    global x, y
    x = []
    y = []
    for i in data.tests:
        x.append(i.x)
        y.append(i.y)

def callbackImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    for it in range(min(len(x),len(y))):
        angle = math.atan2(y[it],x[it])/math.pi*180
        camFOV = 30
        if abs(angle < camFOV) and x[it] < 50:
            plotX = int(320 / camFOV * angle)
            cv2.circle(img, (320 - plotX,int(240 - x[it]/2)), 3, (0, 255, 0), 4)
    
    img_message = bridge.cv2_to_imgmsg(img)
    pub.publish(img_message)
    # print(img.shape) # 480*640*3


def listener():
    global pub
    rospy.init_node("camListener", anonymous=False)
    sub1 = rospy.Subscriber("/testRects", Tests, callbackData)
    sub2 = rospy.Subscriber("/camImg", Image, callbackImg)
    pub = rospy.Publisher("/camDraw", Image, queue_size=100)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
