#! /usr/bin/env python3
# coding=utf-8
import os
import math
import rospy
import yaml

from ars408_msg.msg import GPSinfo as GPSInfo, pathPoint as PathPoint, pathPoints as PathPoints
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from jsk_rviz_plugins.msg import OverlayText


# load config
with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)


RVIZ_TRAJECTORY = True
PREDICT_TRAJECTORY = True


class GPS():
    def __init__(self, radar_channel: str):
        """
        Subscribe:
            topic_GPS

        Publish:
            topic_PathPoints
            topic_PredictPath
        """
        self.radar_channel: str = radar_channel
        self.gps_data: GPSInfo = None

        print("GPS Subscribe: {}".format(radar_channel))
        self.sub_gps_info = rospy.Subscriber(
            config["topic_GPS"], GPSInfo, self.callbackGPS, queue_size=1)
        self.pub_predict_path = rospy.Publisher(
            self.radar_channel + "/" + config["topic_PredictPath"], Path, queue_size=1)
        self.pub_path_points = rospy.Publisher(
            self.radar_channel + "/" + config["topic_PathPoints"], PathPoints, queue_size=1)
        self.pub_overlay_text_300 = rospy.Publisher(
            self.radar_channel + "/" + config["topic_OverlayText300"], OverlayText, queue_size=1)

    def callbackGPS(self, data: GPSInfo) -> None:
        self.gps_data = data

    def loop(self) -> None:
        if self.gps_data is None:
            return

        # overlayText
        overlay_text = OverlayText(action=OverlayText.ADD)
        text = f'Speed: {self.gps_data.speed}\n\
            ZAxis: {self.gps_data.zaxis}\n\
            Longitude: {self.gps_data.longitude:.15f}\n\
            Latitude: {self.gps_data.latitude:.15f}\n\
            AccX: {self.gps_data.accX}\n\
            AccY: {self.gps_data.accY}\n\
            AccZ: {self.gps_data.accZ}\n'
        overlay_text.text = text
        self.pub_overlay_text_300.publish(overlay_text)

        # predict_path path_points
        predict_speed = (self.gps_data.speed * 4.0) / 100.0
        predict_zaxis = (self.gps_data.zaxis * 4.0) / 100.0

        predict_path = Path(header=Header(
            frame_id=self.radar_channel, stamp=rospy.Time.now()))

        path_points = PathPoints()
        x0, y0, x1, y1 = 0.0, 0.0, 0.0, 0.0
        for i in range(100):
            ps = PoseStamped()
            p = Point(z=-1)

            if i != 0:
                x1 = math.cos((90.0 - predict_zaxis * i) *
                              math.pi / 180.0) * predict_speed + x0
                y1 = math.sin((90.0 - predict_zaxis * i) *
                              math.pi / 180.0) * predict_speed + y0

            p.x = y1
            p.y = x1

            x0 = x1
            y0 = y1

            path_points.pathPoints.append(PathPoint(X=p.y, Y=p.y))
            ps.pose.position = p
            predict_path.poses.append(ps)

        self.pub_path_points.publish(path_points)
        self.pub_predict_path.publish(predict_path)


def listener(radar_channel):
    rospy.init_node("GPS")
    rosrate = rospy.Rate(config["frameRate"])

    gps = GPS(radar_channel)

    while not rospy.is_shutdown():
        gps.loop()
        rosrate.sleep()


if __name__ == "__main__":
    try:
        radar_channel = rospy.get_param('GPS/radarChannel')

        if radar_channel == "/radar/first":
            listener(radar_channel)
    except rospy.ROSInternalException:
        pass
