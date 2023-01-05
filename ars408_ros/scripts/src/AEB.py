#! /usr/bin/env python3
# coding=utf-8
import math
import cv2
from enum import Enum

import rospy
import message_filters
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from pacmod_msgs.msg import VehicleSpeedRpt

from ars408_msg.msg import Object, Objects, Motion, RadarPoints

def get_dist(x, y):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2))


class DummyMotionBridge():
    """
    Dummy Motion Bridge.
    Takes motion message in, change the timestamp to now.

    Subscribe:
        /motion/raw
    
    Publish:
        /motion/synced
    """
    def __init__(self):
        self.sub = rospy.Subscriber("/motion/raw", Motion, self.callback)
        self.pub = rospy.Publisher("/motion/synced", Motion, queue_size=2)
        
    def callback(self, msg: Motion) -> None:
        msg.header.stamp = rospy.Time.now()
        msg.speed = 5 # 10 m/s, 36 km/h
        self.pub.publish(msg)


class AEBState(Enum):
    WARNING = 0
    DANGER = 1


class AEBResult(object):
    def __init__(self, state: AEBState, object: Object, ttr: float, ttc: float):
        self.state = state
        self.object = object
        self.ttr = ttr
        self.ttc = ttc


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
        # fusion based AEB
        # self.sub_object_array = message_filters.Subscriber("/object_array", Objects)
        # pure radar AEB
        self.sub_object_array = message_filters.Subscriber("/radar/front_center/decoded_messages", RadarPoints)
        self.sub_speed = message_filters.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt)
        self.pub_text = rospy.Publisher("/AEB/text", MarkerArray, queue_size=1)
        self.sub_image = message_filters.Subscriber("/rgb/front_center/calib_image", Image)
        self.pub_img = rospy.Publisher("/AEB/img", Image, queue_size=1)

        self.bridge = CvBridge()


        self.numoftriggerbrake = 20
        self.numofwarningbrake = 10
        self.numoflastappearlimit = -5
        self.closeDistanceThreshold = 2
        self.resultLog = []

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.sub_object_array, self.sub_speed, self.sub_image], queue_size=20, slop=10)
        self.synchronizer.registerCallback(self.callback)

    # def callback(self, object_array: Objects, motion_raw: Motion):
    def callback(self, radar_array: RadarPoints, speed: VehicleSpeedRpt, img: Image):
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        status = ""

        object_array = Objects()
        for i in radar_array.rps:
            object_array.objects.append(Object(radar_info=i))
        result: list[AEBResult] = list()

        for i in object_array.objects:
            i: Object
            
            """
            Intersection-AEB
            see more at: https://doi.org/10.1080/00423114.2021.1998558
            """
            x_plus = 0
            
            # Original Method
            # y_plus = i.radar_info.distY + \
            #     i.radar_info.distX * math.tan(math.radians(i.radar_info.angle))

            # Our Thought

            y_plus = ((i.radar_info.distX - 0) / (i.radar_info.vrelX if i.radar_info.vrelX != 0 else 1e-6)) * speed.vehicle_speed + i.radar_info.distY

            ego_speed = speed.vehicle_speed if speed.vehicle_speed > 0 else 0.1
            target_speed = get_dist(i.radar_info.vrelX, i.radar_info.vrelY)

            ttr_target = get_dist(x_plus - i.radar_info.distX, y_plus - i.radar_info.distY) \
                / (target_speed + 1e-6) # avoid divide-by-zero error
            ttr_ego = get_dist(x_plus, y_plus) / (ego_speed)

            ttc = min(ttr_target, ttr_ego)
            """
            calculate using warning threshold
            """
            ttc_threshold = abs(target_speed) / (2 * 9.8 * 0.3) + .5
            if (abs(ttr_target - ttr_ego) < 1 and ttc < ttc_threshold):
                # TODO
                # add to msg and publish

                if(next((j for j in self.resultLog if j['id'] == i.radar_info.id), None) == None):
                    self.resultLog.append({'id':i.radar_info.id,'count':1,'appear':True,'last_appear':0})
                else:
                    cnt = next((j for j in self.resultLog if j['id'] == i.radar_info.id))
                    cnt['count'] = cnt['count'] + 1 if cnt['count'] < self.numofappearlimit else cnt['count']
                    cnt['last_appear'] = 0
                    cnt['appear'] = True

                print(self.resultLog)

                if len(self.resultLog) and (next((j for j in self.resultLog if j['id'] == i.radar_info.id))['count'] >= self.numofwarningbrake):
                    rospy.logwarn("Should Brake")
                    status = "Should Brake"
                elif len(self.resultLog) and (next((j for j in self.resultLog if j['id'] == i.radar_info.id))['count'] >= self.numoftriggerbrake):
                    rospy.logwarn("Trigger Brake")
                    status = "Trigger Brake"
                else:
                    # rospy.loginfo("AEB Danger")
                    # rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))
                    result.append(AEBResult(state=AEBState.DANGER, object=i, ttr=abs(ttr_target - ttr_ego), ttc=ttc))
                
                continue
            
            ttc_threshold = abs(target_speed) / (2 * 9.8 * 0.5) + (abs(target_speed) * 1.5) + 1.5
            if abs(ttr_target - ttr_ego) < 1.5 and ttc < ttc_threshold:
                # TODO
                # add to msg and publish
                # rospy.loginfo("AEB Warning")
                # rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))
                result.append(AEBResult(state=AEBState.WARNING, object=i, ttr=abs(ttr_target - ttr_ego), ttc=ttc))

            if speed.vehicle_speed > 0 and get_dist(i.radar_info.distX, i.radar_info.distY) < self.closeDistanceThreshold:
                rospy.logwarn("Closing Distance Brake")
                status = "Closing Distance Brake"

            """
            calculate using danger threshold
            """
        cv2.putText(img, status,(20, 70), cv2.FONT_HERSHEY_PLAIN, 6, (255, 255, 255), 5)
        top = (int) (0.05*img.rows); bottom = top;
        left = (int) (0.05*img.cols); right = left;
        if status == "Closing Distance Brake" or status == "Trigger Brake":
            cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,(255,0,0))    
        elif status == "Should Brake":
            cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,(255,69,0))
        msg = self.bridge.cv2_to_imgmsg(img)
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_img.publish(msg)

        for eachLog in self.resultLog:
            if(eachLog['appear'] == False):
                eachLog['last_appear'] -= 1
            else:
                eachLog['last_appear'] = 0
                eachLog["appear"] = False

        while(next((cnt for cnt in self.resultLog if cnt['last_appear'] == self.numoflastappearlimit), None) != None):
            self.resultLog.remove(next(cnt for cnt in self.resultLog if cnt['last_appear'] == self.numoflastappearlimit))

        marker_array = MarkerArray()
        marker_array.markers.append(Marker(header=Header(stamp=rospy.Time.now(), frame_id="base_link"), ns="AEB_text", type=Marker.TEXT_VIEW_FACING, action=Marker.DELETEALL))


        for id, i in enumerate(result):
            i: AEBResult
            marker = Marker(
                header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
                ns="AEB_text",
                id=id,
                type=Marker.TEXT_VIEW_FACING,
                action=Marker.ADD)

            text = "AEB " + ("Warning " if i.state == AEBState.WARNING else "Danger ")
            text += "\nttr: {}\nttc: {}".format(i.ttr, i.ttc)
            marker.text = text
            marker.scale = Vector3(z=0.5)
            marker.pose = Pose(
                position=Point(x=i.object.radar_info.distX, y=i.object.radar_info.distY, z=0.4),
                orientation=Quaternion(x=0, y=0, z=0, w=1.0))
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker_array.markers.append(marker)
        self.pub_text.publish(marker_array)


def main():
    rospy.init_node("AEB")

    dummy_motion_bridge = DummyMotionBridge()
    aeb = AEB()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
