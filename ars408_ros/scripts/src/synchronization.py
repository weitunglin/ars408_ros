#!/usr/bin/env python3
# coding=utf-8
from functools import partial
import rospy
import message_filters
from sensor_msgs.msg import Image

from ars408_msg.msg import RadarPoints
from config.config import default_config

class Synchronizer():
    """
    Synchronizer syncs two sensors: rgb and radar.
    """

    def __init__(self):
        self.synchronizers: dict[str, message_filters.ApproximateTimeSynchronizer] = dict()
        self.publishers: dict[str, list[rospy.Publisher]] = dict()

        """
        subscribe unsynchronized topic. (rgbs and radars)
        """
        # via sensor aspect
        # for rgb_name in self.rgb_names:
        #     if rgb_config[rgb_name].camera_type == CameraType.RGB:
        #         self.sub.append(
        #             message_filters.Subscriber(f"/rgb/{rgb_name}/calib_image", Image))
        #         self.pub.append(
        #             rospy.Publisher(f"/rgb/{rgb_name}/synced_image", Image, queue_size=1))

        #     # FIXME
        #     # add thermal synchronization support
        #     # elif rgb_config[rgb_name].camera_type == CameraType.THERMAL:
        #     #     self.sub_rgb.append(
        #     #         message_filters.Subscriber(f"/rgb/{rgb_name}/original_image", Image))

        # for radar_name in self.radar_names:
        #     self.sub.append(
        #         message_filters.Subscriber(
        #             f"/radar/{radar_name}/decoded_messages", RadarPoints))
        #     self.pub.append(
        #         rospy.Publisher(
        #             f"/radar/{radar_name}/synced_messages", RadarPoints, queue_size=1))
        # """
        # use `message_filters.ApproximateTimeSynchronizer` two sync messages.
        # """
        # self.synchronizer = message_filters.ApproximateTimeSynchronizer(
        #     self.sub, queue_size=10, slop=0.2)
        # self.synchronizer.registerCallback(self.callback)
    
        # via sensor fusion aspect
        for i in default_config.sensor_fusion:
            sub = [
                message_filters.Subscriber(f"/rgb/{i.rgb_name}/calib_image", Image),
                message_filters.Subscriber(f"/radar/{i.radar_name}/decoded_messages", RadarPoints)]
            self.publishers[i.name] = [
                rospy.Publisher(f"/rgb/{i.rgb_name}/synced_image", Image, queue_size=1),
                rospy.Publisher(
                    f"/radar/{i.radar_name}/synced_messages", RadarPoints, queue_size=1)]
            self.synchronizers[i.name] = message_filters.ApproximateTimeSynchronizer(
                sub, queue_size=5, slop=0.1)
            self.synchronizers[i.name].registerCallback(partial(self.callback, i.name))
            


    def callback(self, name: str, *msgs):
        for pub, msg in zip(self.publishers[name], msgs):
            pub.publish(msg)

def main():
    rospy.init_node("synchronization node")

    synchronizer = Synchronizer()
    rospy.spin()


if __name__ == "__main__":
    main()
