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

                pub1.publish(speed)
                sendcodeStr = "{0:02x}".format((speedDir << 14) + int(speed / 0.02 + 0.5))
                sendText = "cansend can0 300#" + sendcodeStr
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