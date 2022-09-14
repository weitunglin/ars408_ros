#!/usr/bin/env python3
# coding=utf-8
import rospy
import message_filters
from sensor_msgs.msg import Image

from ars408_msg.msg import RadarPoints
from config.config import rgb_config, radar_config, CameraType

class Synchronizer():
    """
    Synchronizer syncs two sensors: rgb and radar.
    """

    def __init__(self):
        self.rgb_names = rgb_config.names
        self.radar_names = radar_config.names

        self.sub: list[message_filters.Subscriber] = list()
        self.pub: list[rospy.Publisher] = list()

        """
        subscribe unsynchronized topic. (rgbs and radars)
        """
        for rgb_name in self.rgb_names:
            if rgb_config[rgb_name].camera_type == CameraType.RGB:
                self.sub.append(
                    message_filters.Subscriber(f"/rgb/{rgb_name}/calib_image", Image))
                self.pub.append(
                    rospy.Publisher(f"/rgb/{rgb_name}/synced_image", Image, queue_size=5))

            # FIXME
            # add thermal synchronization support
            # elif rgb_config[rgb_name].camera_type == CameraType.THERMAL:
            #     self.sub_rgb.append(
            #         message_filters.Subscriber(f"/rgb/{rgb_name}/original_image", Image))

        for radar_name in self.radar_names:
            self.sub.append(
                message_filters.Subscriber(
                    f"/radar/{radar_name}/decoded_messages", RadarPoints))
            self.pub.append(
                rospy.Publisher(
                    f"/radar/{radar_name}/synced_messages", RadarPoints, queue_size=5))

        """
        use `message_filters.ApproximateTimeSynchronizer` two sync messages.
        """
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.sub, queue_size=3, slop=0.2)
        self.synchronizer.registerCallback(self.callback)

    def callback(self, *msgs):
        for pub, msg in zip(self.pub, msgs):
            pub.publish(msg)

def main():
    rospy.init_node("synchronization node")

    synchronizer = Synchronizer()
    rospy.spin()


if __name__ == "__main__":
    main()
