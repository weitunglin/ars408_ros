#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

ultrasonic = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=2, rtscts=True, dsrdtr=True)
print(ultrasonic)

def dec2hex(__dec__):
    return str(hex(ord(__dec__))[2:])

def hex2dec(__hex__):
    return int(__hex__, 16)

if __name__ == '__main__':
    rospy.init_node('RS232', anonymous=True)
    pub = rospy.Publisher('UART', String, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ser = ultrasonic.read(1)
        packet = []

        if ord(ser) == 255:
            print('\nStart')
            packet = []
            raw = ''
            ser = ultrasonic.read(9)

            for i in range(0, len(ser)-1, 2):
                raw = raw + dec2hex(ser[i]) + dec2hex(ser[i+1]) + ' '

                value = dec2hex(ser[i]) + dec2hex(ser[i+1])
                packet.append(str(hex2dec(value))  + ' mm')
            print('[INFO]Raw data: %s') % raw                
                
        pub.publish(packet)
        rospy.loginfo(packet)
        rate.sleep()
