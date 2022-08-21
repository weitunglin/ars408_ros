#! /usr/bin/env python3
# coding=utf-8
from functools import partial
from collections import defaultdict

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from config import default_config, rgb_config
from ars408_msg.msg import RadarPoints, Objects


class SensorFusion():
    def __init__(self):
        self.sub_fusion = defaultdict()
        self.sub_fusion["rgb"] = defaultdict()
        self.sub_fusion["radar"] = defaultdict()
        self.pub_fusion = defaultdict()
        self.pub_fusion["radar_image"] = defaultdict()
        self.fusion_data = defaultdict()
        self.fusion_data["radar"] = defaultdict()
        self.fusion_data["rgb"] = defaultdict()
        self.config = default_config.sensor_fusion

        self.bridge = CvBridge()

        for i in self.config:
            # rgb topic
            self.sub_fusion["rgb"][i.rgb_name] = rospy.Subscriber("/rgb/" + i.rgb_name + "/calib_image", Image, partial(self.rgb_callback, i.rgb_name), queue_size=1)
            # radar topic
            self.sub_fusion["radar"][i.radar_name] = rospy.Subscriber("/radar/" + i.radar_name + "/decoded_messages", RadarPoints, partial(self.radar_callback, i.radar_name), queue_size=1)

            # render radar on image
            if default_config.use_radar_image:
                name = i.rgb_name + "/" + i.radar_name
                self.pub_fusion["radar_image"][name] = rospy.Publisher("/fusion/" + name + "/radar_image", Image, queue_size=1)

        # objects publisher
        self.pub_object = rospy.Publisher("/objects", Objects, queue_size=1)
    
    def rgb_callback(self, rgb_name, image):
        self.fusion_data["rgb"][rgb_name] = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    def radar_callback(self, radar_name, radar_points):
        self.fusion_data["radar"][radar_name] = radar_points

    def project_radar_to_rgb(self, rgb_name, radar_name):
        P_radar_to_rgb = np.vstack([np.array([7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -2.717806000000e-01]).reshape((3,4)), np.array([0., 0., 0., 1.])])
        P_rgb = rgb_config[rgb_name].P
        projection_matrix = np.dot(P_rgb, P_radar_to_rgb)
        return projection_matrix

    def project_to_rgb(self, radar_points, projection_matrix):
        num_pts = radar_points.shape[1]
        points = np.vstack([radar_points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        points[:2, :] /= points[2, :]
        return points[:2, :]

    def loop(self):
        for i in self.config:
            if not i.radar_name in self.fusion_data["radar"] or not i.rgb_name in self.fusion_data["rgb"]:
                continue

            radar_points = self.fusion_data["radar"][i.radar_name].rps
            points_3d = np.empty(shape=(0, 3))
            for p in radar_points:
                points_3d = np.append(points_3d, [[p.distX, p.distY, 1]], axis=0)

            # print(points_3d)
            # obtain projection matrix
            proj_radar_to_rgb = self.project_radar_to_rgb(radar_name=i.radar_name, rgb_name=i.rgb_name)
            # print(proj_radar_to_rgb)
            # apply projection
            points_2d = self.project_to_rgb(points_3d.transpose(), proj_radar_to_rgb)
            # print(points_2d)


            # filter out pixels points
            inds = np.where((points_2d[0, :] < rgb_config[i.rgb_name].size[0]) & (points_2d[0, :] >= 0) &
                (points_2d[1, :] < rgb_config[i.rgb_name].size[1]) & (points_2d[1, :] >= 0) &
                (points_3d[:, 0] > 0)
                )[0]
            points_2d = points_2d[:, inds]

            # retrieve depth from radar (x)
            if default_config.use_radar_image:
                radar_image = self.fusion_data["rgb"][i.rgb_name].copy()
                for p in range(points_2d.shape[1]):
                    depth = 1
                    cv2.circle(radar_image, (int(points_2d[0, p]), int(points_2d[1, p])), 20, color=(255, 0, 0),thickness=-1)

                self.pub_fusion["radar_image"][i.rgb_name + "/" + i.radar_name].publish(self.bridge.cv2_to_imgmsg(radar_image))

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
