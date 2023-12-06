#! /usr/bin/env python3
# coding=utf-8
import math
import rospy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

from ars408_msg.msg import RadarPoints

class RadarVisualizer():
    def __init__(self):
        self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_callback, queue_size=1)
        self.pub_marker = rospy.Publisher("/radar/marker", MarkerArray, queue_size=1)
        self.pub_range = rospy.Publisher("/radar/range", MarkerArray, queue_size=1)

    def radar_callback(self, radar_points: RadarPoints):
        markers = MarkerArray()

        # # clear previous markers
        markers.markers.append(Marker(
            header=Header(frame_id="base_link", stamp=rospy.Time.now()),
            action=Marker.DELETEALL
        ))

        # # radar points
        id = 0
        for i in radar_points.rps:

            speed = math.sqrt(pow(i.vrelX, 2)+pow(i.vrelY,2))
            if speed > 1 and abs(i.vrelX / (i.vrelY+1e-6)) > 0.2:
                marker = Marker(
                    header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                    id=id,
                    ns="front_center",
                    type=Marker.CYLINDER,
                    action=Marker.ADD,
                    pose=Pose(
                        position=Point(x=i.distX, y=i.distY, z=1.0),
                        orientation=Quaternion(x=0, y=0, z=1)
                    ),
                    scale=Vector3(x=1, y=1, z=1.5),
                    color=ColorRGBA(r=0.0, g=0.0, b=0.9, a=1.0)
                )
                markers.markers.append(marker)
                dist = math.sqrt(pow(i.distX, 2)+pow(i.distY,2))
                text_marker = Marker(
                    header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                    id=id,
                    ns="front_center_text",
                    type=Marker.TEXT_VIEW_FACING,
                    action=Marker.ADD,
                    text=f"{dist:.2f}m\n{speed*3.6:.2f}km/h",
                    pose=Pose(
                        position=Point(x=i.distX, y=i.distY, z=1.5),
                        orientation=Quaternion(x=0, y=0, z=1)
                    ),
                    scale=Vector3(x=1, y=1, z=2.5),
                    color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                )
                markers.markers.append(text_marker)
                id = id + 1

        self.pub_marker.publish(markers)

        # radar range
        range_markers = MarkerArray()

        # ego arrow
        range_markers.markers.append(
            Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=0,
                ns="ego_arrow",
                type=Marker.ARROW,
                action=Marker.ADD,
                scale=Vector3(x=5, y=0.5, z=0.5),
                color=ColorRGBA(r=1.0, b=0.5, g=0.0, a=1.0)
            )
        )

        i = 0
        for t in [[0, 0, 0]]:
            radar_transform = t
            range_marker = Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=id,
                ns="range_marker_wide",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=0, y=0, z=0.1),
                    orientation=Quaternion(x=0, y=0, z=0, w=1.0)
                ),
                scale=Vector3(x=0.5, y=0.1, z=0.1),
                color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            )
            id = id + 1
            # p = Point(z=0.5)
            # range_marker.points.append(Point(x=0, y=0, z=0.5))
            # range_marker.points.append(Point(x=1, y=0, z=0.5))
            # range_marker.points.append(Point(x=2, y=0, z=0.5))
            # range_marker.points.append(Point(x=2, y=1, z=0.5))
            # range_marker.points.append(Point(x=2, y=2, z=0.5))
            # range_marker.points.append(Point(x=0, y=0, z=0.5))

            # wide range
            rotate = -40 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 70 + math.cos(rotate) * 0
            ))
            rotate = -46 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 35 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 35 + math.cos(rotate) * 0
            ))
            range_marker.points.append(Point(
                x=0 + radar_transform[0],
                y=0 + radar_transform[1]
            ))
            rotate = 46 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 35 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 35 + math.cos(rotate) * 0
            ))
            rotate = 40 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 70 + math.cos(rotate) * 0
            ))
            for i in range(40, -41, -5):
                rotate = i * math.pi / 180.0 + radar_transform[2]
                range_marker.points.append(Point(
                    x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                    y=math.sin(rotate) * 70 + math.cos(rotate) * 0
                ))
            range_markers.markers.append(range_marker)

            # narrow range
            range_marker = Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=id,
                ns="range_marker_narrow",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=0, y=0, z=0.1),
                    orientation=Quaternion(x=0, y=0, z=0, w=1.0)
                ),
                scale=Vector3(x=0.5, y=0.1, z=0.1),
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            )
            id = id + 1
            
            rotate = 4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            rotate = 9 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 150 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 150 + math.cos(rotate) * 0
            ))
            range_marker.points.append(Point(
                x=0 + radar_transform[0],
                y=0 + radar_transform[1]
            ))
            rotate = -9 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 150 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 150 + math.cos(rotate) * 0
            ))
            rotate = -4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            rotate = 4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            range_markers.markers.append(range_marker)
        self.pub_range.publish(range_markers)

def main():
    rospy.init_node("Visualize Radar")

    radar_visualizer = RadarVisualizer()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
