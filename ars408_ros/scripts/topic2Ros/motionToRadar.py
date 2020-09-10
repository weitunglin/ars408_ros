#! /usr/bin/env python3
# coding=utf-8
import rospy
from std_msgs.msg import Float32

import os
import math
from subprocess import Popen, PIPE

import numpy as np

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
    zaxisArray = []
    speedArray = []
    sigma, Q, R = 0.1, 4e-4, 0.1**2

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        with Popen(['adb shell cat /storage/emulated/0/sensor.txt'], shell=True, stdout=PIPE) as proc:
            string = proc.stdout.readline().decode('UTF-8')
            try:
                speed, zaxis = string.split(' ')
                speed, zaxis = float(speed), float(zaxis)
                speedDir = 0x1

                zaxis = zaxis * 180.0 / math.pi
                zaxis = min(327, zaxis)
                zaxis = max(-327, zaxis)

                zaxisArray.append(zaxis)
                speedArray.append(speed)
                if len(zaxisArray) > 1000:
                    np.delete(zaxisArray, 0)
                if len(speedArray) > 1000:
                    np.delete(speedArray, 0)

                zaxisKalmanList = Kalmen(len(zaxisArray), zaxisArray, sigma, Q, R)
                zaxisKalman = zaxisKalmanList[len(zaxisKalmanList)-1]
                speedKalmanList = Kalmen(len(speedArray), speedArray, sigma, Q, R)
                speedKalman = speedKalmanList[len(speedKalmanList)-1]
                speedKalman = max(0, speedKalman)

                pub1.publish(speedKalman)
                pub2.publish(zaxisKalman)

                sendcodeStr = "{0:04x}".format((speedDir << 14) + int(speedKalman / 0.02 + 0.5))    # Kalman
                sendText = "cansend can0 300#" + sendcodeStr
                os.popen(sendText)
                print(sendText)

                sendcodeStr = "{0:04x}".format(int((zaxisKalman + 327.68) / 0.01 + 0.5))            # Kalman
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
