#!/usr/bin/env python3
# coding=utf-8
from functools import partial
import rospy
import message_filters
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
from pypcd import numpy_pc2
from pypcd import pypcd
import numpy as np
import pandas as pd

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

        self.bridge = CvBridge()
    
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
                sub, queue_size=5, slop=1)
            self.synchronizers[i.name].registerCallback(partial(self.callback, i.name))

    def callback(self, name: str, *msgs):
        for pub, msg in zip(self.publishers[name], msgs):
            pub.publish(msg)

        # save static file
        if False and name == "front_center":
            base_path = "/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/"
            radar_data = np.array([])
            radar_msg = msgs[1]
            if len(radar_msg.rps) == 0:
                return
            for i in radar_msg.rps:
                radar_data = np.append(radar_data, [i.header.stamp, i.id, i.vrelX, i.vrelY, i.distX, i.distY, i.dynProp, i.rcs, i.distX, i.vrelX, i.distY, i.vrelY, 0, 0, 0, 0, 0, 0, 0, 0, i.width, i.height, 0, 0, 0])
            radar_data = radar_data.reshape((-1, 25))
            df = pd.DataFrame(radar_data, columns=["time_ns", "track_id", "velocity_x", "velocity_y", "position_x", "position_y", "dynprop", "rcs", "dist_long_rms", "vrel_long_rms", "dist_lat_rms", "vrel_lat_rms", "arel_lat_rms", "arel_long_rms", "orientation_rms", "meas_state", "prob_of_existobject_type", "acceleration_x", "acceleration_y", "orientation_angel", "length", "width", "false_alarm", "ambig_state", "invalid_state"])
            df.to_csv(base_path+"data/front_radar.csv")

            img = self.bridge.imgmsg_to_cv2(msgs[0])
            cv2.imwrite(base_path+"data/image.jpg", img)

def main():
    rospy.init_node("synchronization node")

    synchronizer = Synchronizer()
    rospy.spin()


if __name__ == "__main__":
    main()
