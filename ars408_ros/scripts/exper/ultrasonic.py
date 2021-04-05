#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
from ars408_msg.msg import Usonic

ultrasonic = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=2, rtscts=True, dsrdtr=True)
# print(ultrasonic)

if __name__ == '__main__':
    rospy.init_node('RS232', anonymous=True)
    pub = rospy.Publisher('Ultrasonic', Usonic, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ser = ultrasonic.read(1)
        packet = []

        if ord(ser) == 255:
            info = Usonic()
            ser = ultrasonic.read(9)

            d1 = ord(ser[0]) * 256 +  ord(ser[1])
            d2 = ord(ser[2]) * 256 +  ord(ser[3])
            d3 = ord(ser[4]) * 256 +  ord(ser[5])
            d4 = ord(ser[6]) * 256 +  ord(ser[7])
            # print("[Hex] #1: {}, #2: {}, #3: {}, #4: {}".format(hex(d1), hex(d2), hex(d3), hex(d4)))
            print("[Distance] #1: {}mm, #2: {}mm, #3: {}mm, #4: {}mm".format(d1, d2, d3, d4))
            # packet = str(d1) + 'mm, ' + str(d2) + 'mm, ' + str(d3) + 'mm, ' + str(d4) + 'mm'
            # rospy.loginfo(packet)
            info.d1, info.d2, info.d3, info.d4 = d1, d2, d3, d4
            pub.publish(info)

        rate.sleep()
