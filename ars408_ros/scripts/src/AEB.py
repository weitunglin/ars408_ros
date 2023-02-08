#! /usr/bin/env python3
# coding=utf-8
import math
import cv2
from enum import Enum
from math import sqrt, pow, acos, sin
import rospy
import message_filters
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge

from pacmod_msgs.msg import VehicleSpeedRpt, SystemRptFloat

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
        self.sub = rospy.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.callback)
        self.pub = rospy.Publisher("/dummy/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, queue_size=2)
        
    def callback(self, msg: Motion) -> None:
        msg.header.stamp = rospy.Time.now()
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
        self.sub_image = message_filters.Subscriber("/rgb/front_center/calib_image", Image)
        self.pub_text = rospy.Publisher("/AEB/text", MarkerArray, queue_size=1)
        self.pub_img = rospy.Publisher("/AEB/img", Image, queue_size=1)

        self.bridge = CvBridge()

        # Hyperparameters
        self.DangerAlertingThreshold = 10
        self.BrakeTriggeringThreshold = 20
        self.LowSpeedAndCloseDistanceThreshold = 2
        self.LastAppearedFrameThreshold = -5

        """
        MonitoringList : List of Dictionary
            Elements in Dictionary:
                RPS_ID : Integer, To identify every single dangerous radar point.
                ContinuousAppearingCounter : Integer, To count how many continuous frames are there a radar point is algorithm-judged dangerous.
                IfAppearedInThisFrame : Boolean, To record if a radar point is algorithm-judged dangerous in this frame.
                LastAppearedFrame : Integer, To count how many frames passed while last time that a radar point is algorithm-judged dangerous.
        
        """
        self.MonitoringList = []

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.sub_object_array, self.sub_speed, self.sub_image], queue_size=20, slop=10)
        self.synchronizer.registerCallback(self.callback)


    def callback(self, radar_array: RadarPoints, speed: VehicleSpeedRpt, img: Image):
        img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        cv2.putText(img,f'Current Speed: {round(speed.vehicle_speed * 3.6,2)} KM/H',(20,70),cv2.FONT_HERSHEY_PLAIN, 4, (255,255,255), 2)
        status = ""

        object_array = Objects()
        for i in radar_array.rps:
            object_array.objects.append(Object(radar_info=i))
        result: list[AEBResult] = list()

        for i in object_array.objects:
            i: Object
    
            distX, distY = round(i.radar_info.distX), round(i.radar_info.distY)
            vrelX, vrelY = round(i.radar_info.vrelX), round(i.radar_info.vrelY)
            v_ego = round(speed.vehicle_speed)
            v_target = round(sqrt(pow((vrelX + v_ego),2)+pow(vrelY,2)))

            ttr_target = None
            ttr_ego = None
            posX_collision = None

            if (vrelY * distY) < 0:
                ttr_target = abs(distY / vrelY)
                posX_collision = distX + (vrelX + v_ego) * ttr_target
                if v_ego:
                    ttr_ego = posX_collision / v_ego
            elif abs(distY) <= 1 and vrelY == 0:
                if vrelX >= 0:
                    continue
                else:
                    rospy.loginfo("L.134")
                    rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                    ttr_ego = abs(distX / (v_ego + 1e-6))
                    if(ttr_ego < 2 + v_ego * 0.02):
                        #rospy.loginfo(f'type: {i.radar_info.classT, }distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                        if(next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id), None) == None):
                            self.MonitoringList.append({'RPS_ID':i.radar_info.id,'ContinuousAppearingCounter':1,'IfAppearedInThisFrame':True,'LastAppearedFrame':0})
                        else:
                            cnt = next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id))
                            cnt['ContinuousAppearingCounter'] = cnt['ContinuousAppearingCounter'] + 1 if cnt['ContinuousAppearingCounter'] < self.BrakeTriggeringThreshold else cnt['ContinuousAppearingCounter']
                            cnt['LastAppearedFrame'] = 0
                            cnt['IfAppearedInThisFrame'] = True
            elif abs(distY) < 1 and vrelY != 0:
                if vrelX >= 0:
                    continue
                else:
                    rospy.loginfo("L.149")
                    rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                    ttr_ego = abs(distX / (v_ego + 1e-6))
                    if(ttr_ego < 2 + v_ego * 0.02):
                        if(next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id), None) == None):
                            self.MonitoringList.append({'RPS_ID':i.radar_info.id,'ContinuousAppearingCounter':1,'IfAppearedInThisFrame':True,'LastAppearedFrame':0})
                        else:
                            cnt = next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id))
                            cnt['ContinuousAppearingCounter'] = cnt['ContinuousAppearingCounter'] + 1 if cnt['ContinuousAppearingCounter'] < self.BrakeTriggeringThreshold else cnt['ContinuousAppearingCounter']
                            cnt['LastAppearedFrame'] = 0
                            cnt['IfAppearedInThisFrame'] = True
            else:
                if ttr_target and ttr_ego:
                    ttc = min(ttr_ego, ttr_target)
                    ttc_threshold = abs(v_target) / (2 * 9.8 * 0.2) + 3
                    if(ttc < ttc_threshold and abs(ttr_ego - ttr_target) < 1):
                        rospy.loginfo("L.164")
                        rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                        if(next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id), None) == None):
                            self.MonitoringList.append({'RPS_ID':i.radar_info.id,'ContinuousAppearingCounter':1,'IfAppearedInThisFrame':True,'LastAppearedFrame':0})
                        else:
                            cnt = next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id))
                            cnt['ContinuousAppearingCounter'] = cnt['ContinuousAppearingCounter'] + 1 if cnt['ContinuousAppearingCounter'] < self.BrakeTriggeringThreshold else cnt['ContinuousAppearingCounter']
                            cnt['LastAppearedFrame'] = 0
                            cnt['IfAppearedInThisFrame'] = True
        rospy.loginfo("-"*50)
        flg = 0

        for it in self.MonitoringList:
            if it['ContinuousAppearingCounter'] == self.BrakeTriggeringThreshold:
                flg = 1
                continue
            if it['ContinuousAppearingCounter'] >= self.DangerAlertingThreshold:
                flg = 2

        if flg == 1:
            rospy.logwarn("AEB SYSTEM ENFORCES BRAKE")
            status = "AEB SYSTEM ENFORCES BRAKE"
            cv2.putText(img,status,(20,120),cv2.FONT_HERSHEY_PLAIN, 4, (255,255,255), 2)
            img = cv2.copyMakeBorder(img, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=(0,0,255))
        elif flg == 2:
            rospy.logwarn("YOU SHOULD BRAKE")
            status = "YOU SHOULD BRAKE"
            cv2.putText(img,status,(20,120),cv2.FONT_HERSHEY_PLAIN, 4, (255,255,255), 2)
            img = cv2.copyMakeBorder(img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=(0,69,255))

        for it in self.MonitoringList:
            if it['IfAppearedInThisFrame'] == False:
                it['LastAppearedFrame'] -= 1
            else:
                it['LastAppearedFrame'] = 0
                it['IfAppearedInThisFrame'] = False

        while(next((it for it in self.MonitoringList if it['LastAppearedFrame'] == self.LastAppearedFrameThreshold), None) != None):
            self.MonitoringList.remove(next((it for it in self.MonitoringList if it['LastAppearedFrame'] == self.LastAppearedFrameThreshold)))
            
        
        msg = self.bridge.cv2_to_imgmsg(img)
        msg.header = Header(stamp=rospy.Time.now())
        self.pub_img.publish(msg)
        

        # marker_array = MarkerArray()
        # marker_array.markers.append(Marker(header=Header(stamp=rospy.Time.now(), frame_id="base_link"), ns="AEB_text", type=Marker.TEXT_VIEW_FACING, action=Marker.DELETEALL))


        # for id, i in enumerate(result):
        #     i: AEBResult
        #     marker = Marker(
        #         header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
        #         ns="AEB_text",
        #         id=id,
        #         type=Marker.TEXT_VIEW_FACING,
        #         action=Marker.ADD)

        #     text = "AEB " + ("Warning " if i.state == AEBState.WARNING else "Danger ")
        #     text += "\nttr: {}\nttc: {}".format(i.ttr, i.ttc)
        #     marker.text = text
        #     marker.scale = Vector3(z=0.5)
        #     marker.pose = Pose(
        #         position=Point(x=i.object.radar_info.distX, y=i.object.radar_info.distY, z=0.4),
        #         orientation=Quaternion(x=0, y=0, z=0, w=1.0))
        #     marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        #     marker_array.markers.append(marker)
        # self.pub_text.publish(marker_array)


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