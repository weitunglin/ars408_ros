#!/usr/bin/env python3
# coding=utf-8
from ast import Call
import enum
from functools import partial
from multiprocessing import allow_connection_pickling

import rospy
import message_filters
from sensor_msgs.msg import Image

from ars408_msg.msg import RadarPoints
from config.config import rgb_config, radar_config, CameraType


class CallbackType(enum.Enum):
    RGB = 0
    RADAR = 1


class Synchronizer():
    """
    Synchronizer syncs two sensors: rgb and radar.
    """

    def __init__(self):
        self.rgb_names = rgb_config.names
        self.radar_names = radar_config.names

        self.sub_rgb: list[message_filters.Subscriber] = list()
        self.pub_rgb: list[rospy.Publisher] = list()
        self.sub_radar: list[message_filters.Subscriber] = list()
        self.pub_radar: list[rospy.Publisher] = list()

        """
        subscribe unsynchronized topic. (rgbs and radars)
        """
        for rgb_name in self.rgb_names:
            if rgb_config[rgb_name].camera_type == CameraType.RGB:
                self.sub_rgb.append(
                    message_filters.Subscriber(f"/rgb/{rgb_name}/calib_image", Image))
            # TODO
            # added thermal synchronization support
            # elif rgb_config[rgb_name].camera_type == CameraType.THERMAL:
            #     self.sub_rgb.append(
            #         message_filters.Subscriber(f"/rgb/{rgb_name}/original_image", Image))
            self.pub_rgb.append(
                rospy.Publisher(f"/rgb/{rgb_name}/synced_image", Image, queue_size=5))

        for radar_name in self.radar_names:
            self.sub_radar.append(
                message_filters.Subscriber(
                    f"/radar/{radar_name}/decoded_messages", RadarPoints))
            self.pub_radar.append(
                rospy.Publisher(
                    f"/radar/{radar_name}/synced_messages", RadarPoints, queue_size=5))

        """
        use `message_filters.ApproximateTimeSynchronizer` two sync messages.
        """
        self.rgb_synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.sub_rgb, queue_size=20, slop=1, allow_headerless=True, reset=True)
        self.rgb_synchronizer.registerCallback(
            partial(self.callback, CallbackType.RGB))
        self.radar_synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.sub_radar, queue_size=10, slop=0.3)
        self.radar_synchronizer.registerCallback(
            partial(self.callback, CallbackType.RADAR))

    def callback(self, type: CallbackType, *msgs):
        if type == CallbackType.RGB:
            pub_list = self.pub_rgb
        elif type == CallbackType.RADAR:
            pub_list = self.pub_radar
        else:
            return

        for pub, msg in zip(pub_list, msgs):
            pub.publish(msg)

def main():
    rospy.init_node("synchronization node")

    synchronizer = Synchronizer()
    rospy.spin()


if __name__ == "__main__":
    main()
