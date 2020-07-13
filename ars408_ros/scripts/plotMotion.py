#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from subprocess import Popen, PIPE
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import os, rospy, math, cv2
import numpy as np
import matplotlib.pyplot as plt

now_speed = 0
now_zaxis = 0
now_NofObj = 0

speedY = []
zaxisY = []
NofObject = []
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

def callbackSpeed(data):
    global now_speed
    now_speed = float(data.data)

def callbackZaxis(data):
    global now_zaxis
    now_zaxis = float(data.data)

def callbackNofObject(data):
    # print(data.data)
    temp = str(data.data)
    index = temp.find('M',20,30)
    # print(index)
    temp = temp[20:index]
    # print(temp)
    global now_NofObj
    now_NofObj = int(temp)

def listener():
    rospy.init_node("plotMotion")
    rosrate = rospy.Rate(rate)
    sub1 = rospy.Subscriber("/speed", Float32, callbackSpeed)
    sub2 = rospy.Subscriber("/zaxis", Float32, callbackZaxis)
    sub3 = rospy.Subscriber("/info_obj_sta", String, callbackNofObject)


    plt.title('Speed && Zaxis')
    plt.ylabel('value')
    plt.xlabel('time(s)')

    global now_NofObj, now_zaxis, now_speed, zaxisY, speedY, NofObject, t

    while not rospy.is_shutdown():
        speedY.append(now_speed)
        zaxisY.append(now_zaxis)
        NofObject.append(now_NofObj)

        fs = ft(speedY)
        fz = ft(zaxisY)


        plt.plot(t[:len(fs)], fs, label=r'speed(m/s)')
        plt.plot(t[:len(fz)], fz, label=r'zaxis(degree/s)')
        plt.plot(t[:len(NofObject)], NofObject, label=r'NofObject')
        plt.legend()
        plt.savefig('output.png')
        plt.cla()
        rosrate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
