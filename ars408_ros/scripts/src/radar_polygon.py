#! /usr/bin/env python3
# coding=utf-8
import math

import message_filters
import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Polygon, Point32
from nav_msgs.msg import Path

from ars408_msg.msg import RadarPoints, RadarPoint

class RadarPolygon():
    def __init__(self):
        self.sub_radar = message_filters.Subscriber(
            "/radar/transformed_messages", RadarPoints)
        self.sub_motion = message_filters.Subscriber("/motion/path", Path)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.sub_radar, self.sub_motion], queue_size=3, slop=1)
        self.synchronizer.registerCallback(self.callback)
        self.pub_polygon = rospy.Publisher(
            "/radar_polygon", PolygonStamped, queue_size=1)
        self.radar_data = np.array([])

    def callback(self, radar_points: RadarPoints, path: Path):
        self.radar_data: list[RadarPoint] = radar_points.rps
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
        #polygon_plot.polygon.points.append(Point32(x=radar_polygon.rps[0].distX, y=radar_polygon.rps[0].distY, z=0))
        self.pub_polygon.publish(polygon_plot)

        self.collision_detection(polygon_plot.polygon, path)
    
    def get_P_matrix(self, n):
        P = np.eye(n)
        for i in range(n):
            j = (i + 1) % n
            P[i][j] = -1
        return P

    def collision_detection(self, polygon: Polygon, path: Path):
        """ref: https://arxiv.org/pdf/2203.01442.pdf. section: V. COLLISION DETECTION FOR RADAR POLYGON"""
        n = len(polygon.points)
        m = len(path.poses)
        P = self.get_P_matrix(n)
        points = np.zeros((2, n)) # [[x1, x2, x3, ...], [y1, y2, y3, ...]]
        A = np.zeros((2, m))
        result = np.zeros((m))

        for i in range(n):
            points[0][i] = polygon.points[i].x
            points[1][i] = polygon.points[i].y

        for i in range(m):
            A[0][i] = path.poses[i].pose.position.x
            A[1][i] = path.poses[i].pose.position.y

        x_points, y_points = points[0], points[1]

        F_1 = np.stack((y_points, -x_points)).T
        F_1 = np.dot(P, F_1)
        F_1 = np.dot(F_1, A)
        F_2 = np.dot(P, y_points)
        F_2 = np.multiply(F_2, x_points)
        F_2 = F_2.reshape((F_2.shape[0], 1))
        F_3 = np.dot(P, x_points)
        F_3 = np.multiply(F_3, y_points)
        F_3 = F_3.reshape((F_3.shape[0], 1))
        F = F_1 - F_2 + F_3

        C = np.dot(P, y_points)
        C = C.reshape((C.shape[0], 1))
        C = np.multiply(F, C)
        Q = np.zeros((m, n))
        for i in range(m):
            b = A[1][i]
            for j in range(n):
                k = (j + 1) % n
                y_0, y_1 = y_points[j], y_points[k]
                if min(y_0, y_1) < b < max(y_0, y_1):
                    Q[i][j] = 1

        C = np.multiply(C, Q.T).T
        for i in range(C.shape[0]):
            x = C[i]
            x = x[x>0]

            if x.shape[0] % 2 == 0:
                result[i] = 1

        # rospy.loginfo_throttle(3, result)
        return result


if __name__ == "__main__":
    try:
        rospy.init_node("Radar Polygon")
        radar_polygon = RadarPolygon()
        
        rospy.spin()
    except rospy.ROSInternalException:
        pass
