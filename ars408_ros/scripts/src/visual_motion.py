#! /usr/bin/env python3
# coding=utf-8
import math

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point

from ars408_msg.msg import Motion

class MotionVisualizer():
    def __init__(self):
        self.sub_motion = rospy.Subscriber("raw", Motion, self.callback, queue_size=1)
        self.pub_path = rospy.Publisher("path", Path, queue_size=1)

        self.predict_points = 100

    def callback(self, msg):
        predict_path = Path(header=Header(frame_id="base_link", stamp=rospy.Time.now()))

        x0 = 0
        y0 = 0
        x1 = 0
        y1 = 0

        for i in range(self.predict_points):
            ps = PoseStamped()
            p = Point()

            p.z = -1

            x1 = math.cos((90 - msg.zaxis * i) * math.pi / 180.0) * msg.speed + x0
            y1 = math.sin((90 - msg.zaxis * i) * math.pi / 180.0) * msg.speed + y0

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
