#! /usr/bin/env python3

from subprocess import Popen, PIPE
import os
import rospy
from std_msgs.msg import Float32
import math
import numpy as np

def main():
    rospy.init_node("motion")
    pub1 = rospy.Publisher("/speed", Float32, queue_size=1)
    pub2 = rospy.Publisher("/zaxis", Float32, queue_size=1)
    # pub3 = rospy.Publisher("/zaxisAvg", Float32, queue_size=1)
    # pub4 = rospy.Publisher("/zaxisFilter", Float32, queue_size=1)
    rate = rospy.Rate(20)
    # avgNum = 5
    # index = 0
    # zaxisArray = np.zeros(avgNum)
    # threshold = 10
    # myFilter = [0.6, 0.2, 0.1, 0.05, 0.05]

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

                # if (zaxis - zaxisArray[index]) > threshold:
                #     zaxisThreshold = zaxisArray[index] + threshold
                # elif (zaxisArray[index] - zaxis) > threshold:
                #     zaxisThreshold = zaxisArray[index] - threshold
                # else:
                #     zaxisThreshold = zaxis

                # # zaxisThreshold = zaxis

                # zaxisArray[index] = zaxisThreshold
                # zaxisAvg = np.average(zaxisArray)
                # zaxisFilter = 0
                # for i in range(avgNum):
                #     zaxisFilter += zaxisArray[i] * myFilter[(i - index)%avgNum]
                # index += 1
                # index %= avgNum
                
                # print("zaxisAvg: ", "{: .4f}".format(zaxisAvg), " degree/s")
                # print("zaxisFilter: ", "{: .4f}".format(zaxisFilter), " degree/s")

                pub1.publish(speed)
                pub2.publish(zaxis)
                # pub3.publish(zaxisAvg)
                # pub4.publish(zaxisFilter)
                zaxis = min(327, zaxis)
                zaxis = max(-327, zaxis)
                sendcodeStr = "{0:02x}".format((speedDir << 14) + int(speed / 0.02 + 0.5))
                sendText = "cansend can0 300#" + sendcodeStr
                os.popen(sendText)
                print(sendText)

                sendcodeStr = "{0:02x}".format(int((zaxis + 327.68) / 0.01 + 0.5))
                # sendcodeStr = "{0:02x}".format(int((zaxisThreshold + 327.68) / 0.01 + 0.5)) # zaxisThreshold
                # sendcodeStr = "{0:02x}".format(int((zaxisAvg + 327.68) / 0.01 + 0.5)) # zaxisAvg
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