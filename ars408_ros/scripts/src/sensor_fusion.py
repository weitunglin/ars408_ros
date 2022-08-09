#! /usr/bin/env python3
# coding=utf-8
import rospy
from functools import partial

from sensor_msgs.msg import Image
from ars408_ros import default_config
from ars408_msg.msg import RadarPoints, Objects


class SensorFusion():
    def __init__(self):
        for i in default_config.sensor_fusion:
            # rgb topic
            rospy.Subscriber("/rgb/" + i.rgb_name + "/original_image", partial(self.rgb_callback, i.rgb_name), Image, queue_size=1)
            # radar topic
            rospy.Subscriber("/radar/" + i.radar_name + "/decoded_messages", partial(self.radar_callback, i.radar_name), RadarPoints, queue_size=1)

        # objects publisher
        self.pub_object = rospy.Publisher("/objects", Objects, queue_size=1)
    
    def rgb_callback(self, rgb_name, image):
        pass

    def radar_callback(self, radar_name, radar_points):
        pass

    def loop(self):
        pass

def main():
    rospy.init_node("Sensor Fusion")

    rate = rospy.Rate(default_config.frame_rate)
    sensor_fusion = SensorFusion()

    while not rospy.is_shutdown():
        sensor_fusion.loop()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
