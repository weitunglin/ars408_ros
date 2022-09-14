#! /usr/bin/env python3
# coding=utf-8
from collections import defaultdict
from functools import partial
import math
import threading

import rospy
import message_filters
import numpy as np

from config.config import radar_config, default_config
from ars408_msg.msg import RadarPoints, RadarPoint


class RadarTransformer():
    def __init__(self):
        self.sub_decoded = defaultdict()
        self.radar_points = defaultdict()
        self.radar_matrices = defaultdict()
        self.radar_names = radar_config.names
        
        for radar_name in radar_config.names:
            self.sub_decoded[radar_name] = message_filters.Subscriber("/radar/" + radar_name + "/decoded_messages", RadarPoints)

            # set up tranformation matrix for each radar
            """
            Matrix
            [cos, -sin, tx]
            [sin, cos,  ty]
            [0,      0, 1]
            """
            self.radar_matrices[radar_name] = defaultdict()
            x_translate, y_translate, rotate = radar_config[radar_name].transform
            rotate = rotate * -1
            self.radar_matrices[radar_name]["rotate_dist"] = np.array([[math.cos(rotate), -1 * math.sin(rotate), 0],
                                        [math.sin(rotate), math.cos(rotate), 0],
                                        [0, 0, 1]])
            self.radar_matrices[radar_name]["translate_dist"] = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [x_translate, y_translate, 1]])
            self.radar_matrices[radar_name]["transform_dist"] = np.dot(self.radar_matrices[radar_name]["rotate_dist"], self.radar_matrices[radar_name]["translate_dist"])
            self.radar_matrices[radar_name]["transform_vrel"] = np.array([[math.cos(rotate), -1 * math.sin(rotate)],
                                [math.sin(rotate), math.cos(rotate)],])

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.sub_decoded[radar_name] for radar_name in radar_config.names], queue_size=1, slop=0.5)
        self.synchronizer.registerCallback(self.radar_callback)
        
        self.pub_transformed = rospy.Publisher("/radar/transformed_messages", RadarPoints, queue_size=1)

    def radar_callback(self, *radar_points_array):
        
        transformed_radar_points = RadarPoints()

        for index in range(len(radar_points_array)):
            for i in radar_points_array[index].rps:
                src_dist = np.array([i.distX, i.distY, 1])
                src_vrel = np.array([i.vrelY, -i.vrelX])
                
                # dist = np.dot(np.dot(src_dist, mat_rotate_dist), mat_trans_dist)
                dist = np.dot(src_dist, self.radar_matrices[self.radar_names[index]]["transform_dist"])
                
                vrel = np.dot(src_vrel, self.radar_matrices[self.radar_names[index]]["transform_vrel"])
                
                transformed_radar_points.rps.append(RadarPoint(
                    id=index,
                    dynProp=i.dynProp,
                    distX=dist[0],
                    distY=dist[1],
                    vrelX=vrel[0],
                    vrelY=vrel[1],
                    rcs=i.rcs,
                    strs=i.strs,
                    prob=i.prob,
                    classT=i.classT,
                    angle=math.atan2(i.distX, i.distY),
                    width=i.width,
                    height=i.height
                ))
        self.pub_transformed.publish(transformed_radar_points)


def main():
    rospy.init_node("Transform Radar")

    radar_transformer = RadarTransformer()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
