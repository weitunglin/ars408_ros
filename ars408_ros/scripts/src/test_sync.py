#!/usr/bin/env python3
# coding=utf-8
import rospy
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from ars408_msg.msg import BatchImage

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
        self.sub = []
        self.pub: rospy.Publisher
        self.channels = ["front_center", "front_left", "front_right", "rear_center", "rear_left", "rear_right"]
        for rgb_name in self.channels:
            self.sub.append(
                message_filters.Subscriber(f"/rgb/{rgb_name}/calib_image", Image))

        self.pub = rospy.Publisher(f"/rgb/synced_batch_image", BatchImage, queue_size=1)
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
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.sub, queue_size=10, slop=0.2)
        self.synchronizer.registerCallback(self.callback)
    
    def callback(self, *msgs):
        msg = BatchImage(header=Header(stamp=rospy.Time.now()))
        for c, m in zip(self.channels, msgs):
            setattr(msg, c, m)
        self.pub.publish(msg)

def main():
    rospy.init_node("test_sync_node")

    synchronizer = Synchronizer()
    rospy.spin()


if __name__ == "__main__":
    main()
