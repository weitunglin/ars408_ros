#! /usr/bin/env python3
# coding=utf-8
from collections import defaultdict
from functools import partial
import math
import threading

import rospy
import numpy as np

from config import radar_config, default_config
from ars408_msg.msg import RadarPoints, RadarPoint


class RadarTransformer():
    def __init__(self):
        self.sub_decoded = defaultdict()
        self.radar_points = defaultdict()
        self.radar_matrices = defaultdict()
        self.mutex = threading.Lock()
        for radar_name in radar_config.names:
            self.sub_decoded[radar_name] = rospy.Subscriber("/radar/" + radar_name + "/decoded_messages", RadarPoints, partial(self.radar_callback, radar_name), queue_size=1)

            # set up tranformation matrix for each radar
            """
            Matrix
            [cos, -sin, tx]
            [sin, cos,  ty]
            [0,      0, 1]
            """
            self.radar_matrices[radar_name] = defaultdict()
            x_translate, y_translate, rotate = radar_config[radar_name].transform
            self.radar_matrices[radar_name]["rotate_dist"] = np.array([[math.cos(rotate), -1 * math.sin(rotate), 0],
                                        [math.sin(rotate), math.cos(rotate), 0],
                                        [0, 0, 1]])
            self.radar_matrices[radar_name]["translate_dist"] = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [y_translate, x_translate, 1]])
            self.radar_matrices[radar_name]["transform_dist"] = np.dot(self.radar_matrices[radar_name]["rotate_dist"], self.radar_matrices[radar_name]["translate_dist"])
            self.radar_matrices[radar_name]["transform_vrel"] = np.array([[math.cos(rotate), -1 * math.sin(rotate)],
                                [math.sin(rotate), math.cos(rotate)],])

        self.pub_transformed = rospy.Publisher("/radar/transformed_messages", RadarPoints, queue_size=1)

    def radar_callback(self, radar_name, radar_points):
        while not self.mutex.acquire():
            pass
        self.radar_points[radar_name] = radar_points
        # if radar_name == "front_center":
        #     import pandas as pd

        #     arr = np.array([])
        #     for i in radar_points.rps:
        #         a = np.array([rospy.Time.now().to_time(), i.id, i.vrelX, i.vrelY, i.distX, i.distY])
        #         arr = np.append(arr, a)
        #     arr = arr.reshape((int(arr.shape[0]/6), 6))
        #     df = pd.DataFrame(arr, columns=["time_ns", "track_id", "velocity_x", "velocity_y", "position_x", "position_y"])
        #     df.to_csv("/home/allen/catkin_ws/front_radar.csv", index=False)

        self.mutex.release()
    
    def loop(self):
        transformed_radar_points = RadarPoints()

        while not self.mutex.acquire():
            pass
        for radar_name in self.radar_points:
            for i in self.radar_points[radar_name].rps:
                src_dist = np.array([i.distX, i.distY, 1])
                src_vrel = np.array([i.vrelX, i.vrelY])
                
                # dist = np.dot(np.dot(src_dist, mat_rotate_dist), mat_trans_dist)
                dist = np.dot(src_dist, self.radar_matrices[radar_name]["transform_dist"])
                
                vrel = np.dot(src_vrel, self.radar_matrices[radar_name]["transform_vrel"])
                
                transformed_radar_points.rps.append(RadarPoint(
                    id=i.id,
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
        self.mutex.release()
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
