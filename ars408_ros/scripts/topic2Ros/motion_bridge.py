#! /usr/bin/env python3
# coding=utf-8
import os
import math
from subprocess import PIPE, Popen

import rospy
import numpy as np
from std_msgs.msg import Header

from ars408_msg.msg import Motion

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
    rospy.init_node("Motion Bridge")

    pub = rospy.Publisher("raw", Motion, queue_size=5)
    rate = rospy.Rate(25)
    zaxis_array = np.array([])
    speed_array = np.array([])
    sigma, Q, R = 0.1, 4e-4, 0.1**2
        
    while not rospy.is_shutdown():
        with Popen(['adb shell cat /storage/emulated/0/Documents/sensor.txt'], shell=True, stdout=PIPE) as proc:
            string = proc.stdout.readline().decode('UTF-8')
            try:
                msg = Motion()
                # speed, zaxis, longitude, latitude, accX, accY, accZ = string.split(' ')
                # speed, zaxis, longitude, latitude, accX, accY, accZ = float(speed), float(zaxis), float(longitude), float(latitude), float(accX), float(accY), float(accZ)
                accX, accY, accZ, speed, longitude, latitude = string.split(' ')
                accX, accY, accZ, speed, longitude, latitude = float(accX), float(accY), float(accZ), float(speed), float(longitude), float(latitude)
                speedDir = 0x1

                zaxis = accZ * 180.0 / math.pi
                zaxis = min(327, zaxis)
                zaxis = max(-327, zaxis)

                zaxis_array = np.append(zaxis_array,zaxis)
                speed_array = np.append(speed_array,speed)
                if len(zaxis_array) > 100:
                    np.delete(zaxis_array, 0)
                if len(speed_array) > 100:
                    np.delete(speed_array, 0)

                zaxis_kalman_list = Kalmen(len(zaxis_array), zaxis_array, sigma, Q, R)
                zaxis_kalman = zaxis_kalman_list[len(zaxis_kalman_list)-1]
                speed_kalman_list = Kalmen(len(speed_array), speed_array, sigma, Q, R)
                speed_kalman = speed_kalman_list[len(speed_kalman_list)-1]
                speed_kalman = max(0, speed_kalman)
                    
                msg.speed, msg.zaxis, msg.longitude, msg.latitude, msg.accX, msg.accY, msg.accZ = speed_kalman, zaxis_kalman, longitude, latitude, accX, accY, accZ
                msg.header = Header(stamp=rospy.Time.now())
                pub.publish(msg)

                # TODO
                # send to all radars instead of only can0
                # both speed and zaxis
                sendcodeStr = "{0:04x}".format((speedDir << 14) + int(speed_kalman / 0.02 + 0.5))    # Kalman
                sendText = "cansend can0 300#" + sendcodeStr
                os.popen(sendText)
                # print(sendText)

                sendcodeStr = "{0:04x}".format(int((zaxis_kalman + 327.68) / 0.01 + 0.5))            # Kalman
                sendText = "cansend can0 301#" + sendcodeStr
                os.popen(sendText)
                # print(sendText)

            except Exception as e:
                rospy.logerr(str(e))
                rospy.logerr_throttle(3, "No value.")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
