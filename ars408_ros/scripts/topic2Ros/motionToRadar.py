#! /usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import Float32

import os
import math
from subprocess import Popen, PIPE

import numpy as np


MIDFILTER = False
KALMAN = True


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
    Noise_std = np.random.normal(0,sigma,size=num)      # 測量noise
    Y = np.zeros(num)

    P = np.zeros(num)               # 每次的最佳偏差
    K = np.zeros(num)               # 卡爾曼增益
    S =  zaxis + Noise_std          # 測量值

    for i in range(1,num):
        P[i] =  P[i-1] + Q
        K[i] =  P[i]/( R + P[i])
        Y[i] =  Y[i-1] + K[i] * (S[i] - Y[i-1])
        P[i] =  (1-K[i])*P[i]

    return Y

def main():
    rospy.init_node("motion")
    pub1 = rospy.Publisher("/speed", Float32, queue_size=1)
    pub2 = rospy.Publisher("/zaxis", Float32, queue_size=1)
    if MIDFILTER:
        pub3 = rospy.Publisher("/zaxisFilter", Float32, queue_size=1)
        zaxisArray = []

    if KALMAN:
        pub4 = rospy.Publisher("/zaxisKalman", Float32, queue_size=1)
        zaxisArray = []
        sigma, Q, R = 0.1, 4e-4, 0.1**2

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        with Popen(['adb shell cat /storage/emulated/0/sensor.txt'], shell=True, stdout=PIPE) as proc:
            string = proc.stdout.readline().decode('UTF-8')
            try:
                speed, zaxis = string.split(' ')
                speed, zaxis = float(speed), float(zaxis)
                speedDir = 0x1
                print("======================")
                print("Speed: ", "{: .4f}".format(speed), " m/s")
                print("Zaxis: ", "{: .4f}".format(zaxis), " radian/s")
                zaxis = zaxis * 180.0 / math.pi
                print("Zaxis: ", "{: .4f}".format(zaxis), " degree/s")

                zaxis = min(327, zaxis)
                zaxis = max(-327, zaxis)

                if MIDFILTER:
                    zaxisArray.append(zaxis)
                    if len(zaxisArray) > 5:
                        np.delete(zaxisArray, 0)
                    zaxisFilterList = midFilter(zaxisArray)
                    zaxisFilter = zaxisFilterList[len(zaxisFilterList)-1]
                    print("zaxisFilter: ", "{: .4f}".format(zaxisFilter), " degree/s")

                if KALMAN:
                    zaxisArray.append(zaxis)
                    if len(zaxisArray) > 1000:
                        np.delete(zaxisArray, 0)
                    zaxisKalmanList = Kalmen(len(zaxisArray), zaxisArray, sigma, Q, R)
                    zaxisKalman = zaxisKalmanList[len(zaxisKalmanList)-1]
                    print("zaxisKalman: ", "{: .4f}".format(zaxisKalman), " degree/s")

                pub1.publish(speed)
                pub2.publish(zaxis)

                if MIDFILTER:
                    pub3.publish(zaxisFilter)
                if KALMAN:
                    pub4.publish(zaxisKalman)

                sendcodeStr = "{0:04x}".format((speedDir << 14) + int(speed / 0.02 + 0.5))
                sendText = "cansend can0 300#" + sendcodeStr
                os.popen(sendText)
                print(sendText)

                sendcodeStr = "{0:04x}".format(int((zaxis + 327.68) / 0.01 + 0.5))
                if MIDFILTER:
                    sendcodeStr = "{0:04x}".format(int((zaxisFilter + 327.68) / 0.01 + 0.5))
                if KALMAN:
                    sendcodeStr = "{0:04x}".format(int((zaxisKalman + 327.68) / 0.01 + 0.5))
                sendText = "cansend can0 301#" + sendcodeStr
                os.popen(sendText)
                print(sendText)

            except Exception as _:
                print("No value.")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
