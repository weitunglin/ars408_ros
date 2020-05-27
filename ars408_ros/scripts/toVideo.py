#! /usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# topic = "/rgbImg"
topic = "/thermalImg"

# path = "/home/viplab/rgb_"
path = "/home/viplab/thermal_"

# size = (800,600) #rgb
size = (640, 512) #thermal

def callbackImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global out
    out.write(img)
    
def listener():
    rospy.init_node("toVideo", anonymous=False)
    global out
    out = cv2.VideoWriter(path+str(rospy.Time.now())+".avi", cv2.VideoWriter_fourcc(*'DIVX'), 20, size, True)
    sub2 = rospy.Subscriber(topic, Image, callbackImg)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
    