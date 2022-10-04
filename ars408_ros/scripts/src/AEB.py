#! /usr/bin/env python3
# coding=utf-8
import math

import rospy
import message_filters

from ars408_msg.msg import Object, Objects, Motion

def get_dist(x, y):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))

class AEB():
    """
    AEB function

    Subscribe:
        /object_array
        /motion/raw
    
    Publish:
        /AEB/warning_array
        /AEB/danger_array
    """

    def __init__(self):
        self.sub_object_array = message_filters.Subscriber("/object_array", Objects)
        self.sub_motion_raw = message_filters.Subscriber("/motion/raw", Motion)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.sub_object_array, self.sub_motion_raw], queue_size=1, slop=1)
        self.synchronizer.registerCallback(self.callback)

    def callback(self, object_array: Objects, motion_raw: Motion):
        for i in object_array.objects:
            i: Object
            
            """
            Intersection-AEB
            see more at: https://doi.org/10.1080/00423114.2021.1998558
            """
            x_plus = 0
            y_plus = i.radar_info.distY + \
                i.radar_info.distX * math.tan(math.radians(i.radar_info.angle))

            ego_speed = motion_raw.speed if motion_raw.speed > 0 else 0.3
            target_speed = get_dist(i.radar_info.vrelX, i.radar_info.vrelY)

            ttr_target = get_dist(x_plus - i.radar_info.distX, y_plus - i.radar_info.distY) \
                / (target_speed + 1e-6) # avoid divide-by-zero error
            ttr_ego = get_dist(x_plus, y_plus) / (ego_speed)

            ttc = min(ttr_target, ttr_ego)

            """
            calculate using warning threshold
            """
            ttc_threshold = abs(target_speed) / (2 * 9.8 * 0.5) + (abs(target_speed) * 1.5) + 1.5
            if abs(ttr_target - ttr_ego) < 1.5 and ttc < ttc_threshold:
                # TODO
                # add to msg and publish
                rospy.loginfo("FCTA Warning")
                rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))

            """
            calculate using danger threshold
            """
            ttc_threshold = abs(target_speed) / (2 * 9.8 * 0.3) + 1
            if (abs(ttr_target - ttr_ego) < 1 and ttc < ttc_threshold):
                # TODO
                # add to msg and publish
                rospy.loginfo("FCTA Danger")
                rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))


def main():
    rospy.init_node("AEB")

    aeb = AEB()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
