#! /usr/bin/env python3
# coding=utf-8
import os
import math
from typing import List

import rospy
import yaml

# [FIXME] rename GPSinfo & pathPoint & pathPoints
from ars408_msg.msg import GPSinfo, pathPoint, pathPoints
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point, PoseStamped
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Path


# load config
with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)


RVIZ_TRAJECTORY = True
PREDICT_TRAJECTORY = True


class GPS():
    def __init__(self, radar_channel):
        """
        Subscribe:
            topic_GPS

        Publish:
            topic_PathPoints
            topic_PredictPath

            topic_WorldTrajectory
            topic_Trajectory
            topic_PredictTrajectory
        """
        self.radar_channel = radar_channel
        self.predict_points: List[pathPoint] = []
        self.gps_points: List[pathPoint] = []
        self.gps_time_diffs: List[float] = []
        self.gps_period = rospy.Time.now()

        self.initial_longitude = None
        self.initial_latitude = None

        print("GPS Subscribe: {}".format(radar_channel))
        self.sub_gps_info = rospy.Subscriber(
            config["topic_GPS"], GPSinfo, self.callbackGPS, queue_size=1)
        self.pub_world_trajectory = rospy.Publisher(
            self.radar_channel + "/" + config["topic_WorldTrajectory"], Marker, queue_size=1)
        self.pub_trajectory = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Trajectory"], Marker, queue_size=1)
        self.pub_predict_trajectory = rospy.Publisher(
            self.radar_channel + "/" + config["topic_PredictTrajectory"], Path, queue_size=1)
        self.pub_predict_path = rospy.Publisher(
            self.radar_channel + "/" + config["topic_PredictPath"], Path, queue_size=1)
        self.pub_path_points = rospy.Publisher(
            self.radar_channel + "/" + config["topic_PathPoints"], pathPoints, queue_size=1)

    def getHeader(self) -> Header:
        return Header(frame_id=self.radar_channel, stamp=rospy.Time.now())

    def callbackGPS(self, data: GPSinfo):
        # calculate time period
        time_now = rospy.Time.now()
        time_diff = (time_now - self.gps_period).to_sec()
        self.gps_period = time_now

        # [TODO] overlayText

        # process gps point data
        gps_point = pathPoint()
        gps_point.X = data.latitude / 0.00000899823754
        gps_point.Y = ((data.longitude - 121.0) * math.cos(data.latitude *
                       math.pi / 180.0)) / 0.000008983152841195214 + 250000.0

        is_gps = False
        if len(self.gps_points) == 0 or gps_point.X != self.gps_points[-1].X or gps_point.Y != self.gps_points[-1].Y:
            is_gps = True
            self.gps_points.append(gps_point)
            self.gps_time_diffs.append(time_diff)

        if RVIZ_TRAJECTORY and len(self.gps_points) >= 2:
            world_trajectory = Marker(header=self.getHeader(), ns="world_trajectory_lines", id=1, type=Marker.LINE_STRIP, action=Marker.ADD,
                                      pose=Pose(orientation=Quaternion(w=1.0)), scale=Vector3(x=0.2), color=ColorRGBA(r=1.0, g=1.0, a=1.0))
            trajectory = Marker(header=self.getHeader(), ns="trajectory_lines", id=0, type=Marker.LINE_STRIP, action=Marker.ADD,
                                pose=Pose(orientation=Quaternion(w=1.0)), scale=Vector3(x=0.2), color=ColorRGBA(r=1.0, g=1.0, a=1.0))

            p_x1 = self.gps_points[-1].X
            p_y1 = self.gps_points[-1].Y
            p_x0 = self.gps_points[-2].X
            p_y0 = self.gps_points[-2].Y
            d = pow(pow(p_x1 - p_x0, 2) + pow(p_y1 - p_y0, 2), 0.5)
            d_y = abs(p_y1 - p_y0)
            d_x = abs(p_x1 - p_x0)

            angle = math.acos(d_y / d) * 180.0 / math.pi

            if p_x0 > p_x1 and p_y0 < p_y1:
                angle = 90 + angle
            elif p_x0 > p_x1 and p_y0 > p_y1:
                angle = 270 - angle
            elif p_x0 < p_x1 and p_y0 > p_y1:
                angle = 270 + angle
            elif p_x0 < p_x1 and p_y0 < p_y1:
                angle = 90 - angle

            if d_y == 0 and p_x1 > p_x0:
                angle = 0
            elif d_y == 0 and p_x1 < p_x0:
                angle = 180
            elif d_x == 0 and p_y1 > p_y0:
                angle = 90
            elif d_x == 0 and p_y1 < p_y0:
                angle = 270

            angle = angle * math.pi / 180.0

            for point in self.gps_points:
                if self.initial_longitude is None:
                    self.initial_longitude = gps_point.X
                if self.initial_latitude is None:
                    self.initial_latitude = gps_point.Y

                world_point = Point(
                    x=point.X - self.initial_latitude, y=-(point.Y - self.initial_longitude), z=1)
                world_trajectory.points.append(world_point)

                # [FIXME] not sure if `local_point` is suitable
                longitude = -(point.Y - p_y1)
                latitude = (point.X - p_x1)
                local_point = Point(
                    x=(math.cos(angle) * latitude - math.sin(angle) * longitude),
                    y=(math.sin(angle) * latitude + math.cos(angle) * longitude), z=1)
                trajectory.points.append(local_point)

            self.pub_world_trajectory.publish(world_trajectory)
            self.pub_trajectory.publish(trajectory)

        # [FIXME] `now_speed` and `now_zaxis` not used in GPS but in AEB
        # should publish another topic or subscribe topic_GPS in AEB
        now_speed = data.speed
        now_zaxis = data.zaxis
        predict_speed = (data.speed * 4.0) / 100.0
        predict_zaxis = (data.zaxis * 4.0) / 100.0

        if PREDICT_TRAJECTORY and is_gps:
            predict_trajectory = Path(header=self.getHeader())

            predict_point = pathPoint()
            predict_point.X = (math.sin(90.0 - data.zaxis)
                               * math.pi / 180.0) * (data.speed)
            predict_point.Y = (math.cos(90.0 - data.zaxis)
                               * math.pi / 180.0) * (data.speed)
            self.predict_points.append(predict_point)

            angle = (-data.zaxis + 180.0) * math.pi / 180.0

            predict_points_length = len(self.predict_points)
            for index, point in enumerate(self.predict_points):
                point.X += predict_point.X
                point.Y += predict_point.Y

                if index != predict_points_length - 1:
                    point.X = math.cos(data.zaxis * math.pi / 180.0) * point.X - \
                        math.sin(data.zaxis * math.pi / 180.0) * point.Y
                    point.Y = math.sin(data.zaxis * math.pi / 180.0) * point.X + \
                        math.cos(data.zaxis * math.pi / 180.0) * point.Y

                ps = PoseStamped()
                p = Point(x=math.cos(angle) * point.X - math.sin(angle) * point.Y,
                          y=-(math.sin(angle) * point.X + math.cos(angle) * point.Y), z=-1)
                
                ps.pose.position = p
                predict_trajectory.poses.append(ps)

                if index == predict_points_length - 1:
                    p.x = 0.0
                    p.y = 0.0
                    ps.pose.position = p
                    predict_trajectory.poses.append(ps)

            self.pub_predict_trajectory.publish(predict_trajectory)

        predict_path = Path(header=self.getHeader())

        path_points = pathPoints()
        x0, y0, x1, y1 = 0.0, 0.0, 0.0, 0.0
        for i in range(100):
            ps = PoseStamped()
            p = Point(z=-1)

            if i != 0:
                x1 = math.cos((90.0 - predict_zaxis * i) * math.pi / 180.0) * predict_speed + x0
                y1 = math.sin((90.0 - predict_zaxis * i) * math.pi / 180.0) * predict_speed + y0
            
            p.x = y1
            p.y = x1

            x0 = x1
            y0 = y1

            path_points.pathPoints.append(pathPoint(X=p.y, Y=p.y))
            ps.pose.position = p
            predict_path.poses.append(ps)
        
        self.pub_path_points.publish(path_points)
        self.pub_predict_path.publish(predict_path)


def listener(radar_channel):
    rospy.init_node("GPS")

    gps = GPS(radar_channel)

    rospy.spin()


if __name__ == "__main__":
    try:
        # get param
        radar_channel = rospy.get_param('GPS/radarChannel')

        # only process first (middle) radar
        if radar_channel == "/radar/first":
            listener(radar_channel)
    except rospy.ROSInternalException:
        pass

"""
Problems
1. 
    gps_points, predict_points, gps_time_diffs 有需要清空嗎
    若是要的話我該訂一個 max list length 還是 時間為限 (幾秒以前的資料就丟掉)
2.
    應該要每 callbackGPS 做一次 predict / trajectorya 然後 publish
    還是要照著 frame_rate 去 publish
"""
