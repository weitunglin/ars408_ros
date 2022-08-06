#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy

sys.path("../../config")
from ars408_msg.msg import RadarPoints
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

class RadarVisualizer():
    def __init__(self):
        self.sub_transformed = rospy.Subscriber("/radar/transformed_messages", RadarPoints, self.radar_callback, queue_size=1)
        self.pub_marker = rospy.Publisher("/radar/transformed_marker", MarkerArray, queue_size=1)

    def radar_callback(self, radar_points: RadarPoints):
        markers = MarkerArray()

        # clear previous markers
        markers.markers.append(Marker(
            header=Header(stamp=rospy.Time.now()),
            action=Marker.DELETEALL
        ))
        self.pub_marker.publish(markers)
        markers.markers.clear()

        # radar points
        for i in radar_points.rps:
            marker = Marker(
                header=Header(stamp=rospy.Time.now()),
                id=i.id,
                type=Marker.SPHERE,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=i.distX, y=i.distY, z=1.0),
                    orientation=Quaternion(x=0, y=0, z=1, w=1)
                ),
                scale=Vector3(x=1, y=1, z=1),
                color=ColorRGBA(r=0.0, g=0.0, b=0.9, a=1.0)
            )
            markers.markers.append(marker)

        self.pub_marker.publish(markers)
        # TODO
        # radar range

def main():
    rospy.init_node("Visual Radar")

    radar_visualizer = RadarVisualizer()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass