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

# ------------------------------------------------ #
# CONTROL UNIT
CONNECTED = False

import enum
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from pynput import keyboard
import threading
mutex = threading.Lock()

''' DEFINE '''
BUTTON_PRESSED = 1.0
BUTTON_DEPRESSED = 0.0
class CmdSlot(enum.IntFlag):
    steering_value = 0
    headlight_change = 1
    turn_signal_cmd = 2
    brake_value = 3
    accelerator_value = 4
    shift_cmd_park = 5
    shift_cmd_neutral = 6
    shift_cmd_drive = 7
    shift_cmd_reverse = 8
    horn_cmd = 9
    engagement = 10
    disengagement = 11
    wiper_change = 12
    hazards_cmd = 13
    lastslot = 14

class CmdControlRemote:
    def __init__(self):
        rospy.loginfo("CmdControlRemote::Init in")
        # Publisher
        self.user_cmd_pub_ = rospy.Publisher('user_cmd', Float32MultiArray, queue_size=20)
        # Subscriber
        self.enable_sub_ = rospy.Subscriber("as_tx/enabled", Bool, self.PacmodEnabledCb)
        self.sub_speed = rospy.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_cb)
        self.sp = 0
        self.pacmod_enabled_rpt_ = False
        self.cmd_list = [0] * 15
        
        keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        keyboard_listener.start()

    def speed_cb(self, speed:VehicleSpeedRpt):
        self.sp = speed.vehicle_speed * 3.6
        
    def on_press(self, key):
        global mutex, CONNECTED
        try:
            print('key {0} pressed'.format(key))
            if key == keyboard.Key.esc: # disconnect Hexagon
                print('--------------disconnect Hexagon------------------')
                CONNECTED = False
                mutex.acquire()
                cmdarray = [0.0] * 15
                cmdarray[CmdSlot.disengagement.value] = 1.0
                cmd = Float32MultiArray(data = cmdarray)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                

            if key.char == '0': # connect Hexagon
                print('----------------connect Hexagon--------------------')
                CONNECTED = True
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.engagement.value] = 1.0
                cmd = Float32MultiArray(data = cmdarray)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                
            if key.char == 'w': # accelerator_value ++ (speed up)
                print('----------------speed up--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                acv = cmdarray[CmdSlot.accelerator_value.value] + 0.01
                veh_accelerator_value = 0.3
                cmdarray[CmdSlot.accelerator_value.value] = veh_accelerator_value if acv > veh_accelerator_value else acv
                cmdarray[CmdSlot.brake_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()


            if key.char == 'a': # steering_value ++ (steering left)
                print('----------------steering left--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                stev = cmdarray[CmdSlot.steering_value.value] + 0.01
                veh_max_steering_val = 0.2
                cmdarray[CmdSlot.steering_value.value] = veh_max_steering_val if stev > veh_max_steering_val else stev
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == 's': # accelerator_value -- (speed down)
                print('----------------speed down--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                acv = cmdarray[CmdSlot.accelerator_value.value] - 0.02
                cmdarray[CmdSlot.accelerator_value.value] = 0 if acv < 0.0 else acv
                cmdarray[CmdSlot.brake_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

            if key.char == 'd': # steering_value -- (turn right)
                print('----------------steering right--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                stev = cmdarray[CmdSlot.steering_value.value] - 0.01
                veh_max_steering_val = 0.2
                cmdarray[CmdSlot.steering_value.value] = -(veh_max_steering_val) if stev < -(veh_max_steering_val) else stev
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                

            if key.char == 'f': # steering_value = 0.0
                print('------------steering back----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.steering_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                
            if key.char == 'x': # brake_value = 0.4
                print('------------emergency brake----------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                cmdarray[CmdSlot.brake_value.value] = min(1, 0.3 + self.sp * 0.013)
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()

        except AttributeError:
            print('special key {0} pressed'.format(key))
            if key == keyboard.Key.space: # brake
                print('----------------brake--------------------')
                mutex.acquire()
                cmdarray = self.cmd_list
                bkv = cmdarray[CmdSlot.brake_value.value] + 0.02
                veh_max_break_val = 0.4
                cmdarray[CmdSlot.brake_value.value] = veh_max_break_val if bkv > veh_max_break_val else bkv
                cmdarray[CmdSlot.accelerator_value.value] = 0.0
                cmd = Float32MultiArray(data = cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                cmd = self.KeyRelease(cmdarray)
                rospy.loginfo(cmd.data)
                self.user_cmd_pub_.publish(cmd)
                self.cmd_list = cmd.data
                mutex.release()
                
    def on_release(self, key):
        pass
    
    def PacmodEnabledCb(self, msg):
        prev_pacmod_enabled_rpt = self.pacmod_enabled_rpt_
        self.pacmod_enabled_rpt_ = msg.data

    def KeyRelease(self, cmd):
        #cmdarray = list(cmd)
        cmd[CmdSlot.shift_cmd_park.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_neutral.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_drive.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.shift_cmd_reverse.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.engagement.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.disengagement.value] = BUTTON_DEPRESSED
        cmd[CmdSlot.headlight_change.value] = BUTTON_DEPRESSED

        return Float32MultiArray(data=cmd)

    def speed_up(self, speed ):
        print('----------------speed up--------------------')
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        acv = cmdarray[CmdSlot.accelerator_value.value] + 0.00005 * speed
        veh_accelerator_value = 0.2
        cmdarray[CmdSlot.accelerator_value.value] = veh_accelerator_value if acv > veh_accelerator_value else acv
        cmdarray[CmdSlot.brake_value.value] = 0.0
        cmd = Float32MultiArray(data = cmdarray)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        self.user_cmd_pub_.publish(cmd)
        self.cmd_list = cmd.data
        mutex.release()
        return acv

    def brake(self, speed):
        print('----------------brake--------------------')
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        
        bkv = cmdarray[CmdSlot.brake_value.value] + 0.002 * speed + 0.01
        veh_max_break_val = 0.3
        cmdarray[CmdSlot.brake_value.value] = veh_max_break_val if bkv > veh_max_break_val else bkv
        cmdarray[CmdSlot.accelerator_value.value] = 0.0
        cmd = Float32MultiArray(data = cmdarray)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        self.user_cmd_pub_.publish(cmd)
        self.cmd_list = cmd.data
        mutex.release()
        return bkv

    def brake_aeb(self, speed):
        print('----------------brake--------------------')
        global mutex
        mutex.acquire()
        cmdarray = self.cmd_list
        
        bkv = cmdarray[CmdSlot.brake_value.value] + 0.002 * speed + 0.01
        cmdarray[CmdSlot.brake_value.value] = min(1, 0.3 + self.sp * 0.013)
        cmdarray[CmdSlot.accelerator_value.value] = 0.0
        cmd = Float32MultiArray(data = cmdarray)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        self.user_cmd_pub_.publish(cmd)
        self.cmd_list = cmd.data
        mutex.release()
        return bkv
    
    def turn_right(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.turn_signal_cmd] = -1.0
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()

    def turn_left(self):
        mutex.acquire()
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list
        cmdarray[CmdSlot.turn_signal_cmd] = 1.0
        self.cmd_list = cmdarray
        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        mutex.release()

    def reset_turn_signal(self):
        cmd = Float32MultiArray()
        cmdarray = self.cmd_list

        cmdarray[CmdSlot.turn_signal_cmd] = 0.0
        self.cmd_list = cmdarray

        cmd = Float32MultiArray(data = cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
        cmd = self.KeyRelease(cmdarray)
        rospy.loginfo(cmd)
        self.user_cmd_pub_.publish(cmd)
# ------------------------------------------------ #

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
        self.DangerAlertingThreshold = 5
        self.BrakeTriggeringThreshold = 13
        self.LowSpeedAndCloseDistanceThreshold = 2
        self.LastAppearedFrameThreshold = -5

        self.controllUnit = CmdControlRemote()

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
                    # CONSOLE INFO
                    # rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                    ttr_ego = abs(distX / ((v_ego - vrelX) + 1e-6)) # ADD 1e-6 TO PREVENT FROM ZERO DIVISION
                    if(ttr_ego < 3.5 + v_ego * 0.05):
                        if(next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id), None) == None):
                            self.MonitoringList.append({'RPS_ID':i.radar_info.id,'ContinuousAppearingCounter':1,'IfAppearedInThisFrame':True,'LastAppearedFrame':0})
                        else:
                            cnt = next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id))
                            cnt['ContinuousAppearingCounter'] = cnt['ContinuousAppearingCounter'] + 1 if cnt['ContinuousAppearingCounter'] < self.BrakeTriggeringThreshold else cnt['ContinuousAppearingCounter']
                            cnt['LastAppearedFrame'] = 0
                            cnt['IfAppearedInThisFrame'] = True
            # elif abs(distY) < 1 and vrelY != 0:
            #     if vrelX >= 0:
            #         continue
            #     else:
            #         # CONSOLE INFO
            #         # rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
                    
            #         ttr_ego = abs(distX / ((v_ego - vrelX) + 1e-6))
            #         if(ttr_ego < 3.5 + v_ego * 0.05):
            #             if(next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id), None) == None):
            #                 self.MonitoringList.append({'RPS_ID':i.radar_info.id,'ContinuousAppearingCounter':1,'IfAppearedInThisFrame':True,'LastAppearedFrame':0})
            #             else:
            #                 cnt = next((it for it in self.MonitoringList if it['RPS_ID'] == i.radar_info.id))
            #                 cnt['ContinuousAppearingCounter'] = cnt['ContinuousAppearingCounter'] + 1 if cnt['ContinuousAppearingCounter'] < self.BrakeTriggeringThreshold else cnt['ContinuousAppearingCounter']
            #                 cnt['LastAppearedFrame'] = 0
            #                 cnt['IfAppearedInThisFrame'] = True
            elif abs(distY) < 2 and distX < 5 and speed.vehicle_speed != 0:
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
                        # CONSOLE INFO
                        # rospy.loginfo(f'type: {i.radar_info.strs}, distX: {distX}, VrelX: {vrelX}, ttr_ego: {ttr_ego}')
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
            self.controllUnit.turn_right()
            self.controllUnit.brake_aeb(v_ego)
        elif flg == 2:
            rospy.logwarn("YOU SHOULD BRAKE")
            status = "YOU SHOULD BRAKE"
            cv2.putText(img,status,(20,120),cv2.FONT_HERSHEY_PLAIN, 4, (255,255,255), 2)
            img = cv2.copyMakeBorder(img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=(0,255,0))
            self.controllUnit.turn_left()
        else:
            self.controllUnit.reset_turn_signal()

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

    # dummy_motion_bridge = DummyMotionBridge()
    aeb = AEB()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass  