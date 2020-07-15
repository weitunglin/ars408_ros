#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from subprocess import Popen, PIPE
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import os, rospy, math, cv2
import numpy as np
import matplotlib.pyplot as plt

now_zaxis = 0
now_zaxis2 = 0

zaxisY = []
zaxisY2 = []
rate = 10

t = np.arange(0,120,float(1/rate))

def ft(l):
    o = []
    w = [1, 0.8, 0.5, 0.1]

    for idx, n in enumerate(l):
        last = max(idx - 3, 0)
        weList = l[last:idx+1][::-1]
        v = 0
        for w_idx, wn in enumerate(weList):
            v += (wn * w[w_idx])
        v /= sum(w)
        o.append(v)
    return o

def callbackZaxis(data):
    global now_zaxis
    now_zaxis = float(data.data)

def callbackZaxis2(data):
    global now_zaxis2
    now_zaxis2 = float(data.data)

def listener():
    rospy.init_node("plotMotion")
    rosrate = rospy.Rate(rate)
    sub1 = rospy.Subscriber("/zaxis", Float32, callbackZaxis)
    sub2 = rospy.Subscriber("/zaxis2", Float32, callbackZaxis2)


    plt.title('Zaxis Compare')
    plt.ylabel('value')
    plt.xlabel('time(s)')

    global now_zaxis, now_zaxis2, zaxisY, zaxisY2, t

    while not rospy.is_shutdown():
        zaxisY.append(now_zaxis)
        zaxisY2.append(now_zaxis2)

        # fs = ft(zaxisY)
        # fz = ft(zaxisY2)
        fs = zaxisY
        fz = zaxisY2


        plt.plot(t[:len(fs)], fs, label=r'zaxis(mpu6050)')
        plt.plot(t[:len(fz)], fz, label=r'zaxis(S10+)')
        plt.legend()
        plt.savefig('output.png')
        plt.cla()
        rosrate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
