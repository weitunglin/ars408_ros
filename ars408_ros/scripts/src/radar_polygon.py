#! /usr/bin/env python3
# coding=utf-8
import math
import rospy

import numpy as np
from geometry_msgs.msg import PolygonStamped, Polygon, Point32

from ars408_msg.msg import RadarPoints, RadarPoint

class RadarPolygon():
    def __init__(self):
        self.sub_radar = rospy.Subscriber(
            "/radar/transformed_messages", RadarPoints, self.callbackRadarPubRT, queue_size=1)
        self.pub_polygon = rospy.Publisher(
            "/radar_polygon", PolygonStamped, queue_size=1)
        self.radar_data = np.array([])

    def callbackRadarPubRT(self, data: RadarPoints):
        self.radar_data: list[RadarPoint] = data.rps
        sampling_angle = 2

        num_sampling = int(360 / sampling_angle)
        sections = [[] for _ in range(num_sampling)]

        radar_polygon = RadarPoints()
        self.radar_data.sort(key=lambda x: math.atan2(x.distY, x.distX))

        # sort radar_data by angle
        rcs_filter = -1000
        for i in self.radar_data:
            if i.rcs > rcs_filter:
                index = (math.floor(math.degrees(i.angle) + 360) % 360) // sampling_angle
                sections[index].append(i)
        
        last_vertex = None
        virtual_vertex_length_threshold = 7.5
        pending_virtual_vertices = []
        for i in range(len(sections)):
            # sort by distance
            # find the closest point
            points = sections[i]
            points.sort(key=lambda x: math.pow(math.pow(x.distY, 2) + math.pow(x.distX, 2), 0.5))
            
            max_range = 20
            if len(points) and math.pow(math.pow(points[0].distY, 2) + math.pow(points[0].distX, 2), 0.5) < max_range:
                radar_polygon.rps.append(points[0])
                if last_vertex is None:
                    last_vertex = points[0]
                else:
                    valid = math.pow(math.pow(points[0].distY-last_vertex.distY, 2) + math.pow(points[0].distX-last_vertex.distX, 2), 0.5) < virtual_vertex_length_threshold
                    if valid:
                        # print(points[0], last_vertex, valid)
                        radar_polygon.rps.extend(pending_virtual_vertices)
                    pending_virtual_vertices.clear()
            else:
                theta = math.radians(i * 2)
                p = RadarPoint(id=-1,distX=max_range*math.cos(theta), distY=max_range*math.sin(theta),angle=math.radians(i*2))
                if last_vertex is not None:
                    pending_virtual_vertices.append(p)

        radar_polygon.rps.sort(key=lambda x: math.atan2(x.distY, x.distX))

        # plot on rviz
        polygon_plot = PolygonStamped(header=rospy.Header(stamp=rospy.Time.now(), frame_id="base_link"))
        polygon_plot.polygon = Polygon()
        for i in radar_polygon.rps:
            polygon_plot.polygon.points.append(Point32(x=i.distX, y=i.distY, z=0))
        polygon_plot.polygon.points.append(Point32(x=radar_polygon.rps[0].distX, y=radar_polygon.rps[0].distY, z=0))
        self.pub_polygon.publish(polygon_plot)

if __name__ == "__main__":
    try:
        rospy.init_node("Radar Polygon")
        radar_polygon = RadarPolygon()
        
        rospy.spin()
    except rospy.ROSInternalException:
        pass
