#! /usr/bin/env python3
# coding=utf-8

import rospy

from pacmod_msgs.msg import VehicleSpeedRpt

class HexagonAPI():
    def __init__(self):
        self.sub = rospy.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback, queue_size=20)
    
    def speed_callback(self, msg: VehicleSpeedRpt):
        if not msg.vehicle_speed_valid:
            rospy.loginfo(msg.vehicle_speed_valid)
        else:
            rospy.loginfo(msg.vehicle_speed * 3.6)

def main():
    rospy.init_node("t")

    h = HexagonAPI()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
