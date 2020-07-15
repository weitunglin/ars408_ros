#! /usr/bin/env python3

from subprocess import Popen, PIPE
import os
import rospy
from std_msgs.msg import Float64, Float32
import math
import numpy as np

pub1 = None

def callback(data):
    print("Zaxis: ", "{: .4f}".format(data.data), " degree/s")
    zaxis = min(327, data.data)
    zaxis = max(-327, data.data)
    sendcodeStr = "{0:04x}".format(int((zaxis + 327.68) / 0.01 + 0.5))
    sendText = "cansend can0 301#" + sendcodeStr
    os.popen(sendText)
    print(sendText)
    pub1.publish(zaxis)


def main():
    global pub1
    rospy.init_node("zaxisMpu6050")
    sub1 = rospy.Subscriber("/mpu6050", Float64, callback)
    pub1 = rospy.Publisher("/zaxis", Float32, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass