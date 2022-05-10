#! /usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import Float32, String

import numpy as np
import matplotlib.pyplot as plt


now_speed = 0
now_zaxis = 0
now_NofObj = 0

speedY = []
zaxisY = []
NofObject = []
rate = 10

t = np.arange(0, 120, float(1/rate))


def midFilter(zaxis):
    num = 5
    outputList = []
    weight = [1, 0.8, 0.5, 0.2, 0.1]

    for index, value in enumerate(zaxis):
        last = max(index - num + 1, 0)
        reverseList = zaxis[last:index+1][::-1]
        output = 0
        for w_index, w_value in enumerate(reverseList):
            output += (w_value * weight[w_index])
        output /= sum(weight)
        outputList.append(output)
    return outputList

def Kalmen(num, zaxis, sigma, Q, R):
    Noise_std = np.random.normal(0,sigma,size=num)      #測量noise
    Y = np.zeros(num)

    P = np.zeros(num)         #每次的最佳偏差
    K = np.zeros(num)         #卡爾曼增益
    S =  zaxis + Noise_std        #測量值

    for i in range(1,num):
        P[i] =  P[i-1] + Q
        K[i] =  P[i]/( R + P[i])
        Y[i] =  Y[i-1] + K[i] * (S[i] - Y[i-1])
        P[i] =  (1-K[i])*P[i]

    return Y

def callbackSpeed(data):
    global now_speed
    now_speed = float(data.data)

def callbackZaxis(data):
    global now_zaxis
    now_zaxis = float(data.data)

def callbackNofObject(data):
    temp = str(data.data)
    index = temp.find('M',20,30)
    temp = temp[20:index]
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

        fs = midFilter(speedY)
        fz = midFilter(zaxisY)
        fk = Kalmen(len(zaxisY), zaxisY, 0.1, 4e-4, 0.1**2)

        plt.plot(t[:len(fs)], fs, label=r'speed(m/s)')
        plt.plot(t[:len(fz)], fz, label=r'zaxis(degree/s)')
        plt.plot(t[:len(fk)], fk, label=r'Kalmen(degree/s)')
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
