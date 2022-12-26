#! /usr/bin/env python3
# coding=utf-8
import math

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from pacmod_msgs.msg import VehicleSpeedRpt, SystemRptFloat
import message_filters

class MotionVisualizer():
    def __init__(self):
        self.sub_speed = message_filters.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt)
        self.sub_steer = message_filters.Subscriber("/parsed_tx/steer_rpt", SystemRptFloat)
        self.syn = message_filters.ApproximateTimeSynchronizer([self.sub_speed, self.sub_steer], queue_size=10, slop=0.1, reset=True)
        self.syn.registerCallback(self.callback)
        self.pub_path = rospy.Publisher("path", Path, queue_size=1)

        self.predict_points = 20

    def callback(self, speed_msg: VehicleSpeedRpt, steer_msg: SystemRptFloat):
        predict_path = Path(header=Header(frame_id="base_link", stamp=rospy.Time.now()))

        speed = speed_msg.vehicle_speed
        steer = steer_msg.output

        x0 = 0
        y0 = 0
        x1 = 0
        y1 = 0

        for i in range(self.predict_points):
            ps = PoseStamped()
            p = Point()

            p.z = -1

            x1 = math.cos((90 - steer * i) * math.pi / 180.0) * speed + x0
            y1 = math.sin((90 - steer * i) * math.pi / 180.0) * speed + y0

            if i == 0:
                x1 = 0
                y1 = 0

            p.x = y1
            p.y = x1

            x0 = x1
            y0 = y1

            ps.pose.position = p
            predict_path.poses.append(ps)

        self.pub_path.publish(predict_path)

def main():
    rospy.init_node("Visualize Motion")

    visual_motion = MotionVisualizer()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
