#! /usr/bin/env python3
# coding=utf-8
from PIL import ImageFont, ImageDraw
import PIL
import cv2, math, os, time
import rospy
import sensor_msgs
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from ars408_msg.msg import RadarPoints, RadarPoint
from ars408_msg.msg import Bboxes, Bbox
from ars408_msg.msg import Motion
from pacmod_msgs.msg import VehicleSpeedRpt

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

from config.config import rgb_config

import sys
sys.path.append(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/data")
os.chdir(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/data")

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
        
        self.pacmod_enabled_rpt_ = False
        self.cmd_list = [0] * 15
        
        keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        keyboard_listener.start()
        
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
# ------------------------------------------------ #

# 內部參數
img_width = 1280
img_height = 712

textScale = 0.75


global nowImg, nowPath, myBBs, myPoints, myACC

class ACC():
    def __init__(self):
        
        self.controlUnit = CmdControlRemote()
        
        # not accurate
        self.DynProp = ["moving", "stationary", "oncoming", "crossing left", "crossing right", "unknown", "stopped"]
        self.Class = ["point", "car", "truck", "reserved", "motorcycle", "bicycle", "wide", "reserved", "others"]
        self.AccClass = ["car", "truck"]
        
        self.status = ""
        self.speed = 20 # 20 m/s (72 km/h)

        ## for only radar
        self.ridCount = [[0, 0, 0, 0, 0, (0,0)] for i in range(100)] # list[[frameCount, dist, vrel, dynProp, missingFrame, distXY]]
        self.ridlist = [] # list[[id, dist, vrel, dynProp, distXY]]
        self.trackID = -1
        self.trackIDPre = -1
        self.trackIDList = [] # max count id in list
        self.trackData = [0, 0, 0, 0, 0, (0,0)] # [frameCount, dist, vrel, dynProp, id, distXY]
        self.maxframe = 0

        ## for bounding box
        self.trackIDPreBbox = -1
        self.trackIDListBbox = [] # in trackIDList and is AccClass

        self.limitFrame = 20
        self.limitFrameBBox = 20
        self.refreshFrame = 20
        self.refreshDist = 5
        self.limitX = 150
        self.limitY = 2
        self.limitDistToPath = 2
        
        self.keepDist = 30 # (meter) 當距離大於此值加速
        
        self.alarmCount= 0  # 沒跟到車超過某個frames停止acc
        self.alarmThres= 60
        self.alarm = False

    ## fixACCrange means use fix range or predict path to be a filter
    ## skip for reducing computing power
    def getAccPoint(self, nowPath, dist, dist_XY, fixACCrange = False, skip = 2):
        if fixACCrange and abs(dist_XY[1]) < self.limitY and dist < self.limitX:
            return True
        elif not fixACCrange:
            count = 0
            for path in nowPath:
                count += 1
                if skip > 1 and count % skip == 0:
                    count -= skip
                    continue
                distToPath = math.sqrt((dist_XY[0] - path.pose.position.x)**2 + (dist_XY[1] - path.pose.position.y)**2)
                if distToPath < self.limitDistToPath:
                    return True
        return False
  
    def refreshridCount(self):
        # rid 為空 ， 所有雷達過20個frame重置
        if not self.ridlist: 
            for idx in range(len(self.ridCount)):
                if self.ridCount[idx][4] < self.refreshFrame:
                    self.ridCount[idx][4] += 1
                if self.ridCount[idx][4] == self.refreshFrame:
                    self.ridCount[idx] = [0, 0, 0, 0, 0, (0,0)]
        else:
            last = cur = [-1, 0, 0, 0, (0,0)] # [id, dist, vrel, dynProp, distXY]
            for i in range(len(self.ridlist)): # for all point in range of ACC
                last = cur # 上一幀的雷達點
                cur = self.ridlist[i]
                ## list[[frameCount, dist, vrel, dynProp, missingFrame, distXY]]
                self.ridCount[cur[0]][0] += 1
                self.ridCount[cur[0]][1] = cur[1]
                self.ridCount[cur[0]][2] = cur[2]
                self.ridCount[cur[0]][3] = cur[3]
                self.ridCount[cur[0]][4] = 0
                
                self.ridCount[cur[0]][5] = cur[4] # distXY

                ## 前個雷達到目前的雷達點 過20個frame重置
                for idx in range(last[0] + 1, cur[0]):
                    if self.ridCount[idx][4] < self.refreshFrame:
                        self.ridCount[idx][4] += 1
                    if self.ridCount[idx][4] == self.refreshFrame:
                        self.ridCount[idx] = [0, 0, 0, 0, 0, (0,0)]
                if i == len(self.ridlist) - 1:
                    for idx in range(cur[0] + 1, 100):
                        if self.ridCount[idx][4] < self.refreshFrame:
                            self.ridCount[idx][4] += 1
                        if self.ridCount[idx][4] == self.refreshFrame:
                            self.ridCount[idx] = [0, 0, 0, 0, 0, (0,0)]

    def trackRadar(self):
        self.trackID = -1
        self.trackIDList = []
        self.trackData = [0, 0, 0, 0, 0, (0,0)] # [frameCount, dist, vrel, dynProp, id, distXY]
        ## ridCount: list[[frameCount, dist, vrel, dynProp, missingFrame, distXY]]

        for i in range(len(self.ridCount)):
            # 假設id 沒被重置且持續出現20個frame則加入跟蹤名單
            if self.ridCount[i][0] >= self.limitFrame:
                self.trackIDList.append(i)
                # 假設原目標出現次數更多 and 距離更近，則照舊
                #if self.trackData[0] >= self.limitFrame and self.trackData[5][1] <= self.ridCount[i][5][1]:
                if self.trackData[0] >= self.limitFrame and self.trackData[1] <= self.ridCount[i][1]:
                    self.trackData = self.trackData
                # 假設有其他目標次數更多 or 距離更近 換跟車目標
                else:
                    self.trackData = self.ridCount[i][:4] + [i] + [self.ridCount[i][5]]
        
        if self.trackData[0] >= self.limitFrame:
            # 車並非停止或移動則跟蹤前一個在移動的車
            self.trackID = self.trackData[4]
            self.trackIDPre = self.trackID if self.trackID != -1 else self.trackIDPre # 沒跟到車照舊pre=pre 跟到pre=cur
    
            if self.trackData[1] > self.keepDist:
                self.status = "加速" if self.trackData[2] > -1 else "等速"

            else:
                self.status = "加速" if self.trackData[2] > 0 else "減速"
                self.status = "等速" if abs(self.trackData[2]) < 1 else self.status
                
                if (self.speed + self.trackData[2]) < 1:
                    self.status = "減速"


            # self.status = "加速" if self.trackData[2] > 0 else "減速"
            # self.status = "等速" if abs(self.trackData[2]) < 1 else self.status
            # self.status = "加速" if self.trackData[1] > self.keepDist and self.trackData[2] > -1\
            #      else self.status
            self.maxframe = max([x[0] for x in self.ridCount])
    
    def controllCar(self):
        
        if CONNECTED:
            if myACC.trackIDPre in myACC.trackIDList and not myACC.alarm:
                if self.status == '加速':
                    if self.trackData[1] > self.keepDist:
                        self.controlUnit.speed_up(5)
                    else:
                        self.controlUnit.speed_up(self.trackData[2]*3.6)
                elif self.status == '減速':
                    self.controlUnit.brake(abs(self.trackData[2])*3.6*0.20)
                elif self.status == '等速':
                    self.controlUnit.speed_up(self.trackData[2]*3.6)
            else : # 定速
                acv = 0
                keep_speed = 15/3.6 #時速30km/h 

                if self.speed < keep_speed: 
                    acv = self.controlUnit.speed_up((keep_speed - self.speed)*3.6)
                if acv < 0 :
                    print("brake to remain speed")
                    self.controlUnit.brake((self.speed - keep_speed)*3.6)
                else:
                    print("accelerate to remain speed")
                

    def ACCwithBbox(self, rid, objClass):
        self.trackIDListBbox = []
        if rid in self.trackIDList and objClass in self.AccClass:
            self.trackIDListBbox.append(rid)
            self.trackIDPreBbox = self.trackIDPre if rid == self.trackIDPre else self.trackIDPreBbox

    def printACC(self, printMode = 1):
        ## ridCount: list[[frameCount, dist, vrel, dynProp, missingFrame]]
        # output [0:id  1:frameCnt  2:dist  3:vrel  4:Prop  5:miss]
        
        if self.trackIDPre in self.trackIDList:
            if self.trackIDPre == self.trackIDPreBbox or self.trackIDPreBbox in self.trackIDListBbox:
                output = [self.trackIDPreBbox] + self.ridCount[self.trackIDPreBbox]
            else:
                output = [self.trackIDPre] + self.ridCount[self.trackIDPre]
            
            if printMode == 0:
                print("MaxFrame:{0:<8d}  TrackFrame:{1:<8}".format(self.maxframe, self.trackData[0]))
                print("      ID:{0:<4d}  Dist:{1:.4f}m  Speed:{2:.4f}m/s  Vrel:{3:.4f}m/s  status:{4:<4}  dynProp:{5:<10}".format(
                    output[0],
                    output[2],
                    self.speed,
                    output[3],
                    self.status,
                    self.DynProp[output[4]]))
            elif printMode == 1:
                print("雷達ID:{0:<4d}  相對距離:{1:.4f}m  當前速度:{2:.4f}m/s  相對速度:{3:.4f}m/s  狀態:{4:<4}".format(
                    output[0],
                    output[2],
                    self.speed,
                    output[3],
                    self.status))
            elif printMode == 3:
                print()
        # else:
        #     print("未針測到前方車輛 維持車速:20m/s")

    def maker(self, point):
        collision_markers = MarkerArray()

        collision_marker = Marker(
            header=Header(frame_id="base_link", stamp=rospy.Time.now()),
            ns="collision_marker",
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            pose=Pose(
                position=Point(x=point.distX,y=point.distY,z=0.1),
                orientation=Quaternion(x=0, y=0, z=0, w=1.0)
            ),
            scale=Vector3(x=0.5, y=0.1, z=0.1),
            color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            lifetime=rospy.Duration(0.01)
        )

        collision_range = 0.5 * self.speed + 0.5 * math.sqrt(point.vrelX**2 + point.vrelY**2) + 5
        for i in range(0,361,10):
            rot = i* math.pi/180.0
            p = Point()
            p.x = math.cos(rot) * collision_range
            p.y = math.sin(rot) * collision_range
            collision_marker.points.append(p)

        collision_markers.markers.append(collision_marker)

        
        return collision_markers
    
    def filterRadar(self, point):
        # 濾除來車
        if abs(point.vrelX) > 5:
            return True
        
        # 過濾旁車
        if abs(point.distX) < 3:
            return True
        if abs(point.distX) < 5 and abs(point.distY) > 1.2:
            return True
        
        # 過濾非車
        # if self.Class[point.classT] not in self.AccClass:
        #     return True
        
        # # 過濾靜止
        # if self.DynProp[point.dynProp] == 'stationary':
        #     return True
        
        return False


class RadarState():
    def __init__(self):
        self.radarPoints = []
        self.speed = 0
        self.zaxis = 0

    def toString(self):
        s = "{0}, {1}\r\n".format(
            self.speed,
            self.zaxis
        )

        for i in self.radarPoints:
            s += "{0:3d}, {1}, {2:>9.3f}, {3:>9.3f}, {4:>9.3f}, {5:>9.3f}, {6:>9.3f}, {7:>9.3f}, {8:>9.3f}\r\n".format(
                i.id,
                i.dynProp,
                i.distX,
                i.distY,
                i.vrelX,
                i.vrelY,
                i.rcs,
                i.width,
                i.height
            )
        return s

class BoundingBox():
    def __init__(self):
        self.bboxes = []

def project_radar_to_cam2():
    P_radar_to_rgb = np.array(
        [7.533745000000e-03, -1, -6.166020000000e-04, 0,
        -4.069766000000e-03, 1.480249000000e-02, -1, 0,
        1, 7.523790000000e-03, 1.480755000000e-02, 0,
        0, 0, 0, 1]
    ).reshape((4, 4))
    P_rgb = rgb_config["front_center"].P
    projection_matrix = np.dot(P_rgb, np.dot(np.eye(4), P_radar_to_rgb))
    return projection_matrix

def project_to_image(points, proj_mat):
    num_pts = points.shape[1]

    # Change to homogenous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = np.dot(proj_mat, points)
    points[:2, :] /= points[2, :]
    return points[:2, :]

def render_radar_on_image(pts_radar, img, img_width, img_height, distTTC):
    global myACC
    dist_img = img.copy()
    h,w,c = img.shape    
    cv2.line(img, (w//2,h//2-100), (w//2,h//2+100), (0,0,0), thickness=2)
    
    # projection matrix (project from radar2cam2)
    proj_radar2cam2 = project_radar_to_cam2()

    # apply projection
    pts_2d = project_to_image(pts_radar.transpose(), proj_radar2cam2)

    # Filter radar points to be within image FOV
    # 照片內的雷達點
    inds = np.where((pts_2d[0, :] < img_width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_height) & (pts_2d[1, :] >= 0) &
                    (pts_radar[:, 0] > 0)
                    )[0]
    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from radar
    imgfov_pc_radar = pts_radar[inds, :]
    imgfov_pc_radar = np.hstack((imgfov_pc_radar, np.ones((imgfov_pc_radar.shape[0], 1))))
    imgfov_pc_cam2 = np.dot(proj_radar2cam2, imgfov_pc_radar.transpose())

    # cmap = plt.cm.get_cmap('hsv', 256)
    # cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255

    distTTC = distTTC[inds] # [dist, point.isDanger, point.id, class]
    fusion_radar = []
    range_id = [a[0] for a in myACC.ridlist]
    k=1
    for i in range(imgfov_pc_pixel.shape[1]):
        color = (255,0,0) #一般
        depthV = min(255, int(820 / imgfov_pc_cam2[2, i]))
        circle_size = 30 / 255 * depthV + 4 * textScale
        x,y = (int(np.round(imgfov_pc_pixel[0, i]) ), int(np.round(imgfov_pc_pixel[1, i]) ))
        
        if distTTC[i][2] in range_id:
            color = (255,255,255) # ACC範圍內
            
            if distTTC[i][2] == myACC.trackIDPre and myACC.trackIDPre in myACC.trackIDList:
                if myACC.trackIDPre == myACC.trackIDPreBbox or myACC.trackIDPreBbox in myACC.trackIDListBbox:
                    color = (0, 0, 255) # 物件框輔助
                else:
                    color = (0, 255, 0) # 雷達跟車
                length = max(int(80 * (80 - distTTC[i][0]) / 80), 4)
                cv2.line(dist_img, (x,y-length), (x,y), (0,255,0), thickness=5)
            cv2.putText(img, str(f'{distTTC[i][0]:3.1f}'), (x, y - 7 * k*3), cv2.FONT_HERSHEY_PLAIN, 2, color, 4)
            cv2.circle(img, (x,y), int(circle_size), color=tuple(color), thickness=-1)
            k+=1

        fusion_radar.append((int(np.round(imgfov_pc_pixel[0, i]) ) , int(np.round(imgfov_pc_pixel[1, i]) ), distTTC[i]))
    

    return img, fusion_radar, dist_img

def drawBbox2Img(img, bboxes, fusion_radar):
    global myACC
    for i in bboxes.bboxes:

        bboxColor = (255, 0, 0)
        textColor = (255, 255, 255)

        # 找到框內最近的雷達點並計算距離
        minDist = 99999
        for radarpoint in fusion_radar:
            # fusion_radar: list[circleX, circleY, list[distTTC] ]
            # radarpoint: [circleX, circleY, list[distTTC] ]
            # distTTC: [dist, point.isDanger, point.id, class]
            # trackInfo: [point.id, dist, missingFrame]
            if radarpoint[0] > i.x_min and radarpoint[0]< i.x_max and \
                radarpoint[1] > i.y_min and radarpoint[1]< i.y_max:
                if radarpoint[2][0] < minDist:
                    bboxColor = (0, 255, 0)
                    minDist = radarpoint[2][0]
                    myACC.ACCwithBbox(radarpoint[2][2], i.objClass)


        yoloText =  "{0}".format(i.objClass)
        disText = ": {0:0.2f} m".format(minDist) if minDist != 99999 else ": Null"

        ## draw bounding box and text for object
        cv2.putText(img, yoloText + disText, (int(i.x_min), int(i.y_min) - 7), cv2.FONT_HERSHEY_PLAIN, 2, textColor, 4)
        cv2.rectangle(img, (int(i.x_min), int(i.y_min)), (int(i.x_max), int(i.y_max)), color=bboxColor, thickness=4)
    

    ## put STATUS
    font = ImageFont.truetype('NotoSansTC-Regular.otf', 50)
    imgPil = PIL.Image.fromarray(img)
    draw = ImageDraw.Draw(imgPil)
    if myACC.trackIDPre in myACC.trackIDList and not myACC.alarm:
        myACC.alarmCount=0
        dir_text = '左側' if myACC.trackData[5][1] >0 else '右側' 
        dist_text=  '前方' + str(int(myACC.trackData[5][0])) + 'm, ' + \
                    (dir_text +  f'{abs(myACC.trackData[5][1]):1.1f}' +'m') if myACC.trackData[5][1]!=0 else ''
        all_text = 'ACC: '+ myACC.status + \
                    '\n我方時速： ' + str(myACC.speed* 3.6) + \
                    '\n前車時速： ' + str(int(abs(myACC.speed + myACC.trackData[2])* 3.6)) + \
                    '\n' + dist_text
                    
        draw.text((0, 0), all_text, fill=(0, 255, 0), font=font)
        
    else:
        if not myACC.alarm:
            myACC.alarmCount+=1
            if myACC.alarmCount > myACC.alarmThres:
                myACC.alarmCount=0
                print('超過時間未跟車，停止ACC')
                #myACC.alarm=True
        all_text = 'ACC: 定速' if not myACC.alarm else '停止ACC'
        draw.text((0, 0), all_text, fill=(255, 0, 0) if not myACC.alarm else (0,0,255), font=font)
        
    img = np.array(imgPil) 

    return img

def callbackPoint(data):
    global myPoints
    myPoints.radarPoints = data.rps

def callbackBbox(data):
    global myBBs
    myBBs.bboxes = data.bboxes

def callbackPath(data):
    global nowPath
    nowPath = data.poses

def callbackImg(data):
    global nowImg
    bridge = CvBridge()
    nowImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def callbackGPS(data):
    global myACC
    myACC.speed = data.vehicle_speed

def listener():
    global nowImg, nowPath, myBBs, myPoints, myACC
    rospy.init_node("plotRadar")
    rate = rospy.Rate(20)
    myPoints = RadarState()
    myBBs = BoundingBox()
    myACC = ACC()
    nowPath = []
    #sub
    rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, callbackPoint, queue_size=1)
    rospy.Subscriber("/rgb/front_center/yolo_bboxes", Bboxes, callbackBbox, queue_size=1)
    rospy.Subscriber("/rgb/front_center/calib_image", Image, callbackImg, queue_size=1)
    rospy.Subscriber("/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, callbackGPS, queue_size=1)
    rospy.Subscriber("/motion/path", Path, callbackPath, queue_size=1)
    #pub
    pub1 = rospy.Publisher("radarImg", Image, queue_size=1)
    pub2 = rospy.Publisher("distImg", Image, queue_size=1)
    pub3 = rospy.Publisher("collision", MarkerArray, queue_size=1)
    
    while not rospy.is_shutdown():
        if not ("nowImg"  in globals() and "myPoints" in globals()):
            continue
        radarList = []
        distTTCList = [] # [dist, point.isDanger, point.id, class]
        myACC.ridlist = [] # list[[id, dist, vrel, dynProp, distXY]]
        markers = MarkerArray()

        for point in np.array(myPoints.radarPoints):
            # 三維雷達點
            radarList.append([point.distX, point.distY, 0]) 

            # 雷達資訊
            dist = math.sqrt(point.distX**2 + point.distY**2) 
            point.classT = min(point.classT, 8) # class id greater than 8 is "other"
            distTTCList.append([dist, point.isDanger, point.id, point.classT])
            
            # 過濾雜訊
            if myACC.filterRadar(point):
                continue

            # 目標速度
            vrel = math.sqrt(point.vrelX**2 + point.vrelY**2)
            vrel = -vrel if point.vrelX < 0 else vrel

            ## getAccPoint return True of False
            ## 如車在 範圍內 加入ridlist
            if myACC.getAccPoint(nowPath, dist, (point.distX, point.distY), True):
                #print ('id:', f'{point.id:02d}','dist: ', f'{int(dist):03d}', 'XY: ', f'{int(point.distX):03d},{point.distY:+1.1f}')
                myACC.ridlist.append([point.id, dist, vrel, point.dynProp, (point.distX, point.distY)])
                markers = myACC.maker(point)
  
        ## refresh radar point if missing frame greater than "refreshFrame"
        myACC.refreshridCount()

        ## calcu possible radar id base on only radar info
        myACC.trackRadar()
        myACC.controllCar()
        #myACC.printACC()
        
        distTTC = np.array(distTTCList) #雷達資訊
        nowImg_radar = np.array(radarList) #雷達座標
        if distTTC.size and nowImg_radar.size:
            radarImg, fusion_radar, dist_img = render_radar_on_image(nowImg_radar, nowImg.copy(), img_width, img_height, distTTC)
            DistImg = drawBbox2Img(dist_img, myBBs, fusion_radar)
            bridge = CvBridge()

            pub1.publish(bridge.cv2_to_imgmsg(radarImg))
            pub2.publish(bridge.cv2_to_imgmsg(DistImg))
            pub3.publish(markers)
        rate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
