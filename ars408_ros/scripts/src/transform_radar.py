#! /usr/bin/env python3
# coding=utf-8
from collections import defaultdict
from functools import partial
import math
import sys

import rospy
import numpy as np

from ars408_ros import radar_config, default_config
from ars408_msg.msg import RadarPoints


class RadarTransformer():
    def __init__(self):
        self.sub_decoded = defaultdict()
        self.radar_points = defaultdict()
        for radar_name in radar_config.names:
            self.sub_decoded[radar_name] = rospy.Subscriber("/radar/" + radar_name + "/decoded_messages", RadarPoints, partial(self.radar_callback, radar_name), queue_size=1)
        
        self.pub_transformed = rospy.Publisher("/radar/transformed_messages", RadarPoints, queue_size=1)

    def radar_callback(self, radar_name, radar_points):
        self.radar_points[radar_name] = radar_points
    
    def loop(self):
        transformed_radar_points = RadarPoints()

        for radar_name in self.radar_points:
            x_translate, y_translate, rotate = radar_config[radar_name].transform
            for i in self.radar_points[radar_name].rps:
                """
                Matrix
                [cos, -sin, tx]
                [sin, cos,  ty]
                [0,      0, 1]
                """
                mat_rotate_dist = np.array([[math.cos(rotate), -1 * math.sin(rotate), 0],
                                            [math.sin(rotate), math.cos(rotate), 0],
                                            [0, 0, 1]])
                mat_trans_dist = np.array([[1, 0, x_translate],
                                        [0, 1, y_translate],
                                        [0, 0, 1]])
                src_dist = np.array([i.distX, i.distY, 1])
                mat_vrel = np.array([[math.cos(rotate), -1 * math.sin(rotate)],
                                    [math.sin(rotate), math.cos(rotate)],])
                src_vrel = np.array([i.vrelX, i.vrelY])
                
                dist = np.dot(np.dot(src_dist, mat_rotate_dist), mat_trans_dist)
                # dist = np.dot(np.dot(src_dist, mat_trans_dist), mat_rotate_dist)
                
                vrel = np.dot(src_vrel, mat_vrel)
                
                i.distX = dist[0]
                i.distY = dist[1]
                i.angle = math.atan2(i.distY, i.distX)
                i.vrelX = vrel[0]
                i.vrelY = vrel[1]
                transformed_radar_points.rps.append(i)
        
        self.pub_transformed.publish(transformed_radar_points)


def main():
    rospy.init_node("Transform Radar")

    rate = rospy.Rate(default_config.frame_rate)
    radar_transformer = RadarTransformer()

    while not rospy.is_shutdown():
        radar_transformer.loop()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
