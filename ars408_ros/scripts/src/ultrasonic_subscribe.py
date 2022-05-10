#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ':%s' % data.data)

if __name__ == '__main__':
    rospy.init_node('listenser', anonymous=True)
    pub = rospy.Subscriber('Ultrasonic', String, callback)
    rospy.spin()

    
