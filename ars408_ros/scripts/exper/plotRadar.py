#! /usr/bin/env python3
# coding=utf-8

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
from ars408_msg.msg import GPSinfo
import yaml
import random

with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']
oldCamera = config['oldCamera']
printACC = False
fixACCrange = False

topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']
# topic_Dual = config['topic_RGB_Calib']
topic_yolo = config['topic_yolo']
topic_Bbox = config['topic_Bbox']
# topic_Radar = config['topic_Radar']
topic_Radar = config['topic_Radar1']

topic_RadarImg = config['topic_RadarImg']
topic_DistImg = config['topic_DistImg']
topic_GPS = config['topic_GPS']
topic_PredictPath = config['topic_PredictPath']
# radarChannel = rospy.get_param("plotRadar/radarChannel")
# print("[SInfo]: radarChannel: {}".format(radarChannel))

# size_RGB = config['size_RGB_Calib']
size_RGB = config['size_RGB_720p']
size_TRM = config['size_TRM']
# size_Dual = config['size_Dual']
size_Dual = config['size_RGB_720p']

# 內部參數
img_width = size_RGB[0]
img_height = size_RGB[1]
pixelScale = img_width / size_RGB[0]

# pixelScale = 1
textScale = config['textTime']
scoreScale = math.sqrt(size_RGB[0] ** 2 + size_RGB[1] ** 2)

# 外部參數
img2Radar_x = config['img2Radar_x']
img2Radar_y = config['img2Radar_y']
img2Radar_z = config['img2Radar_z']

# crop
ROI = config['ROI']
crop_x = (ROI[1][0], ROI[1][1])
crop_y = (ROI[0][0], ROI[0][1])

cmatrix = np.array(config['K1']).reshape(3,3)
dmatrix = np.array(config['D1']).reshape(1,5)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cmatrix, dmatrix, size_RGB, 1, size_RGB)

if oldCamera:
    pixelScale = 1
    newcameramtx = cmatrix
    size_Dual = (800, 600)

global nowImg, nowPath, myBBs, myPoints, myGPS, myACC

calib = {
    'P_rect':np.hstack((newcameramtx, np.array([[0.], [0.], [0.]]))),
    'R_rect':np.array(config['R1']),
    'Tr_radar_to_cam':np.array(config['r2c']),
}

class ACC():
    def __init__(self):
        self.DynProp = ["moving", "stationary", "oncoming", "crossing left", "crossing right", "unknown", "stopped"]
        self.AccDynProp = ["moving", "stopped"]
        self.Class = ["point", "car", "truck", "reserved", "motorcycle", "bicycle", "wide", "reserved", "others"]
        self.AccClass = ["car", "truck"]
        self.status = ""
        self.speed = 0
        self.zaxis = 0

        ## for only radar
        self.ridCount = [[0, 0, 0, 0, 0] for i in range(100)] # list[[frameCount, dist, vrel, dynProp, missingFrame]]
        self.ridlist = [] # list[[id, dist, vrel, dynProp]]
        self.trackID = -1
        self.trackIDPre = -1
        self.trackIDList = [] # max count id in list
        self.trackData = [0, 0, 0, 0, 0] # [frameCount, dist, vrel, dynProp, id]
        self.maxframe = 0

        ## for bounding box
        self.trackIDPreBbox = -1
        self.trackIDListBbox = [] # in trackIDList and is AccClass

        self.limitFrame = 20
        self.limitFrameBBox = 20
        self.refreshFrame = 20
        self.refreshDist = 5
        self.limitX = 100
        self.limitY = 2
        self.limitDistToPath = 2
        
        # show ACC
        self.showDanger = False

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
                # print(path.pose.position.x, path.pose.position.y)
                distToPath = math.sqrt((dist_XY[0] - path.pose.position.x)**2 + (dist_XY[1] - path.pose.position.y)**2)
                if distToPath < self.limitDistToPath:
                    return True

    def refreshridCount(self):
        if not self.ridlist:
            for idx in range(len(self.ridCount)):
                if self.ridCount[idx][4] < self.refreshFrame:
                    self.ridCount[idx][4] += 1
                if self.ridCount[idx][4] == self.refreshFrame:
                    self.ridCount[idx] = [0, 0, 0, 0, 0]
        else:
            last = cur = [-1, 0, 0, 0] # [id, dist, vrel, dynProp]
            for i in range(len(self.ridlist)): # for all point in range of ACC
                last = cur
                cur = self.ridlist[i]
                ## list[[frameCount, dist, vrel, dynProp, missingFrame]]
                self.ridCount[cur[0]][0] += 1
                self.ridCount[cur[0]][1] = cur[1]
                self.ridCount[cur[0]][2] = cur[2]
                self.ridCount[cur[0]][3] = cur[3]
                self.ridCount[cur[0]][4] = 0

                ## init missing id points if missing frame greater than "refreshFrame"
                for idx in range(last[0] + 1, cur[0]):
                    if self.ridCount[idx][4] < self.refreshFrame:
                        self.ridCount[idx][4] += 1
                    if self.ridCount[idx][4] == self.refreshFrame:
                        self.ridCount[idx] = [0, 0, 0, 0, 0]
                if i == len(self.ridlist) - 1:
                    for idx in range(cur[0] + 1, 100):
                        if self.ridCount[idx][4] < self.refreshFrame:
                            self.ridCount[idx][4] += 1
                        if self.ridCount[idx][4] == self.refreshFrame:
                            self.ridCount[idx] = [0, 0, 0, 0, 0]

    def trackRadar(self):
        self.trackID = -1
        self.trackIDList = []
        self.trackData = [0, 0, 0, 0, 0] # [frameCount, dist, vrel, dynProp, id]
        ## ridCount: list[[frameCount, dist, vrel, dynProp, missingFrame]]
        ## all possible id in "trackIDList", min dist for "trackData"
        for i in range(len(self.ridCount)):
            if self.ridCount[i][0] >= self.limitFrame:
                self.trackIDList.append(i)
                if self.trackData[0] >= self.limitFrame and self.trackData[1] <= self.ridCount[i][1]:
                    self.trackData = self.trackData
                else:
                    self.trackData = self.ridCount[i][:4] + [i]
        if self.trackData[0] >= self.limitFrame:
            self.trackID = self.trackData[4] if self.DynProp[self.trackData[3]] in self.AccDynProp else -1
            self.trackIDPre = self.trackID if self.trackID != -1 else self.trackIDPre
            self.status = "加速" if self.trackData[2] > 0 else "減速"
            self.status = "等速" if abs(self.trackData[2]) < 1 else self.status
            self.maxframe = max([x[0] for x in self.ridCount])

    def ACCwithBbox(self, rid, objClass):
        self.trackIDListBbox = []
        if rid in self.trackIDList and objClass in self.AccClass:
            self.trackIDListBbox.append(rid)
            self.trackIDPreBbox = self.trackIDPre if rid == self.trackIDPre else self.trackIDPreBbox

    def printACC(self, printMode = 1):
        ## ridCount: list[[frameCount, dist, vrel, dynProp, missingFrame]]
        if self.trackIDPre in self.trackIDList:
            self.showDanger = True
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
                print("Radar ID:{0:<4d}  Relative Distance:{1:.4f}m  Current Speed:{2:.4f}m/s  Relative Speed:{3:.4f}m/s  Status:{4:<4}".format(
                    output[0],
                    output[2],
                    self.speed,
                    output[3],
                    self.status))
                # print("雷達ID:{0:<4d}  相對距離:{1:.4f}m  當前速度:{2:.4f}m/s  相對速度:{3:.4f}m/s  狀態:{4:<4}".format(
                #     output[0],
                #     output[2],
                #     self.speed,
                #     output[3],
                #     self.status))
        else:
            # print("未針測到前方車輛 維持車速:20m/s")
            # print("Maintain Speed: 20m/s")
            if self.showDanger:
                self.showDanger = False
                print("ACC->\n Maintain Speed: 20m/s")
            # pass


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

class GPS():
    def __init__(self):
        self.speed = 0
        self.zaxis = 0
        self.longitude = 0
        self.latitude = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0

def project_radar_to_cam2(calib):
    r2c = calib['Tr_radar_to_cam'].reshape(3, 3)
    rotate = np.eye(3)
    rtheta = config['rtheta'] * math.pi / 180
    rotate[0][0] = math.cos(rtheta)
    rotate[0][1] = -math.sin(rtheta)
    rotate[1][0] = math.sin(rtheta)
    rotate[1][1] = math.cos(rtheta)
    # rotate[0][0] = -0.555
    # rotate[0][1] = 0.832
    # rotate[0][2] = 0.0
    # rotate[1][0] = -0.832
    # rotate[1][1] = -0.555
    # rotate[1][2] = 0.0
    # rotate[2][0] = 0.0
    # rotate[2][1] = 0.0
    # rotate[2][2] = 1

    # r2c = rotate
    r2c = np.dot(r2c, rotate)
    r2c = np.append(r2c, np.array([[config['img2Radar_x']],[config['img2Radar_y']],[config['img2Radar_z']]]), axis = 1)
    P_radar2cam_ref = np.vstack((r2c, np.array([0., 0., 0., 1.])))  # radar2ref_cam
    R_ref2rect = np.eye(4)
    R0_rect = calib['R_rect'].reshape(3, 3)  # ref_cam2rect
    R_ref2rect[:3, :3] = R0_rect
    P_rect2cam2 = calib['P_rect']
    proj_mat = np.dot(P_rect2cam2 , np.dot(R_ref2rect, P_radar2cam_ref))
    return proj_mat

def project_to_image(points, proj_mat):
    num_pts = points.shape[1]

    # Change to homogenous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = np.dot(proj_mat, points)
    points[:2, :] /= points[2, :]
    return points[:2, :]

def render_radar_on_image(pts_radar, img, calib, img_width, img_height, distTTC):
    global myACC
    # projection matrix (project from radar2cam2)
    proj_radar2cam2 = project_radar_to_cam2(calib)

    # apply projection
    pts_2d = project_to_image(pts_radar.transpose(), proj_radar2cam2)

    # Filter radar points to be within image FOV
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

    cmap = plt.cm.get_cmap('hsv', 256)
    cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255

    distTTC = distTTC[inds]
    fusion_radar = []
    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        depthV = min(255, int(820 / depth))
        color = cmap[depthV, :]
        color = (255, 0, 0)
        if distTTC[i][4]:
            color = (0, 0, 255)
        elif distTTC[i][1]:
            color = (0, 165, 255)
        elif distTTC[i][2] == myACC.trackIDPre and myACC.trackIDPre in myACC.trackIDList:
            if myACC.trackIDPre == myACC.trackIDPreBbox or myACC.trackIDPreBbox in myACC.trackIDListBbox:
                color = (0, 0, 0)
            else:
                color = (127, 127, 127)
        circle_size = 30 / 255 * depthV + 4 * textScale
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i]) * pixelScale),
                         int(np.round(imgfov_pc_pixel[1, i]) * pixelScale)),
                   int(circle_size), color=tuple(color), thickness=-1)
        fusion_radar.append((int(np.round(imgfov_pc_pixel[0, i]) * pixelScale) , int(np.round(imgfov_pc_pixel[1, i]) * pixelScale), distTTC[i], int(circle_size)))
    return img, fusion_radar

def drawBbox2Img(img, bboxes, fusion_radar):
    global myACC
    for i in bboxes.bboxes:
        ## float to int
        intbbox = Bbox()
        intbbox.x_min = int(i.x_min * size_Dual[0])
        intbbox.y_min = int(i.y_min * size_Dual[1])
        intbbox.x_max = int(i.x_max * size_Dual[0])
        intbbox.y_max = int(i.y_max * size_Dual[1])
        intbbox.score = i.score
        intbbox.objClassNum = i.objClassNum
        intbbox.objClass = i.objClass

        bboxColor = (0, 255, 0)
        textColor = (255, 255, 255)
        fontSize = 0.5
        fontThickness = 1
        scale_x = size_Dual[0] / (crop_x[1] - crop_x[0])
        scale_y = size_Dual[1] / (crop_y[1] - crop_y[0])
        leftTop = (int(crop_x[0] + intbbox.x_min / scale_x), int(crop_y[0] + intbbox.y_min / scale_y))
        rightBut = (int(crop_x[0] + intbbox.x_max / scale_x), int(crop_y[0] + intbbox.y_max / scale_y))
        if oldCamera:
            leftTop = (intbbox.x_min, intbbox.y_min)
            rightBut = (intbbox.x_max, intbbox.y_max)

        useFP = False  ## Use 2D radar info to reduce FP of bounding box
        printBBoxcircle = False ## print the corresponding radar point
        bboxcircle = (-1, -1)
        bboxcirclesize = -1
        bboxcirclecolor = (18, 153, 255)

        minDist = 99999
        scoreDist = 99999
        for radarpoint in fusion_radar:
            # fusion_radar: list[circleX, circleY, list[distTTC], circleSize]
            # radarpoint: [circleX, circleY, list[distTTC], circleSize]
            # distTTC: [dist, point.isDanger, point.id, class]
            # trackInfo: [point.id, dist, missingFrame]
            if radarpoint[0] > leftTop[0] and radarpoint[0]< rightBut[0] and radarpoint[1] > leftTop[1] and radarpoint[1]< rightBut[1]:
                if radarpoint[2][0] < minDist:
                    bboxColor = (0, 255, 0)
                    minDist = radarpoint[2][0]
                    if printBBoxcircle:
                        bboxcircle = (radarpoint[0], radarpoint[1])
                        bboxcirclesize = radarpoint[3]
                    if radarpoint[2][1]:
                        bboxColor = (0, 0, 255)
                    myACC.ACCwithBbox(radarpoint[2][2], intbbox.objClass)
            if useFP:
                x = ((leftTop[0] + rightBut[0]) / 2 - radarpoint[0]) ** 2
                y = ((leftTop[1] + rightBut[1]) / 2 - radarpoint[1]) ** 2
                scoreDist = min(scoreDist, math.sqrt(x+y))

        ## reduce false positive
        if useFP:
            intbbox.score = (i.score - scoreDist / scoreScale) if scoreDist != 99999 and (leftTop[1] + rightBut[1]) > img_height else intbbox.score
            if intbbox.score < 0.25:
                print(intbbox.score, leftTop, rightBut)
                continue

        yoloText =  "{0}".format(intbbox.objClass) ## without score
        # yoloText =  "{0}: {1:0.2f}, ".format(intbbox.objClass, intbbox.score) ## with score
        disText = ": Null"
        if minDist != 99999:
            disText = ": {0:0.2f} m".format(minDist)

        ## just for output video
        textPosOffset = 0
        # if ((minDist >= 5 and minDist <= 10) or (minDist >= 36 and minDist <= 37)):
        #     textPosOffset = 10
        # elif minDist >= 20 and minDist <= 21:
        #     textPosOffset = -2
        # textPosOffset = 0 if random.random() > 0.5 else 10

        ## draw bounding box and text for object
        labelSize = cv2.getTextSize(yoloText + disText, cv2.FONT_HERSHEY_SIMPLEX, fontSize * textScale, int(fontThickness * textScale))[0]
        sub_img = img[leftTop[1] - int(12 * textScale) + textPosOffset:leftTop[1] + textPosOffset, leftTop[0]:leftTop[0] + labelSize[0] - int(4 * textScale)]
        blue_rect = np.ones(sub_img.shape, dtype=np.uint8)
        blue_rect[:][:] = (255, 0, 0)
        res = cv2.addWeighted(sub_img, 0.0, blue_rect, 1.0, 0)
        img[leftTop[1] - int(12 * textScale) + textPosOffset:leftTop[1] + textPosOffset, leftTop[0]:leftTop[0] + labelSize[0] - int(4 * textScale)] = res
        cv2.rectangle(img, leftTop, rightBut, bboxColor, int(textScale))
        if printBBoxcircle and bboxcircle[0] != -1:
            cv2.circle(img, bboxcircle, bboxcirclesize, bboxcirclecolor, thickness=-1)
            cv2.line(img, bboxcircle, (int((leftTop[0] + rightBut[0]) / 2), int((leftTop[1] + rightBut[1]) / 2)), bboxcirclecolor, max(1, int(textScale)))
        cv2.putText(img, yoloText + disText, (leftTop[0], leftTop[1] - int(2 * textScale) + textPosOffset), cv2.FONT_HERSHEY_SIMPLEX, fontSize * textScale, textColor, int(fontThickness * textScale), cv2.LINE_AA)

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
    global myGPS, myACC
    myGPS.speed = data.speed
    myGPS.zaxis = data.zaxis
    myGPS.longitude = data.longitude
    myGPS.latitude = data.latitude
    myGPS.accX = data.accX
    myGPS.accY = data.accY
    myGPS.accZ = data.accZ

    myACC.speed = data.speed
    myACC.zaxis = data.zaxis

def listener():
    global nowImg, nowPath, myBBs, myPoints, myGPS, myACC
    rospy.init_node("plotRadar")
    rate = rospy.Rate(frameRate)
    myPoints = RadarState()
    myBBs = BoundingBox()
    myGPS = GPS()
    myACC = ACC()
    nowPath = []
    sub1 = rospy.Subscriber(topic_Radar, RadarPoints, callbackPoint, queue_size=1)
    sub2 = rospy.Subscriber(topic_Bbox, Bboxes, callbackBbox, queue_size=1)
    # sub3 = rospy.Subscriber(topic_Dual, Image, callbackImg, queue_size=1)
    sub3 = rospy.Subscriber(topic_RGB, Image, callbackImg, queue_size=1)
    
    sub4 = rospy.Subscriber(topic_GPS, GPSinfo, callbackGPS, queue_size=1)
    sub5 = rospy.Subscriber(topic_PredictPath, Path, callbackPath, queue_size=1)
    pub1 = rospy.Publisher(topic_RadarImg, Image, queue_size=1)
    pub2 = rospy.Publisher(topic_DistImg, Image, queue_size=1)

    while not rospy.is_shutdown():
        if not ("nowImg"  in globals() and "myPoints" in globals()):
            continue
        radarList = []
        distTTCList = [] # [dist, point.isDanger, point.id, class]
        myACC.ridlist = [] # list[[id, dist, vrel, dynProp]]

        for point in np.array(myPoints.radarPoints):
            pt_x = point.distX
            pt_y = point.distY
            pt_z = 0
            radarList.append([pt_x,pt_y,pt_z])
            dist = math.sqrt(point.distX**2 + point.distY**2)
            point.classT = min(point.classT, 8) # class id greater than 8 is "other"
            distTTCList.append([dist, point.isDanger, point.id, point.classT, point.aeb])

            vrel = math.sqrt(point.vrelX**2 + point.vrelY**2)
            vrel = -vrel if point.vrelX < 0 else vrel

            ## getAccPoint return True of False
            if myACC.getAccPoint(nowPath, dist, (point.distX, point.distY)) and myACC.Class[point.classT] in myACC.AccClass:
                myACC.ridlist.append([point.id, dist, vrel, point.dynProp])

        ## refresh radar point if missing frame greater than "refreshFrame"
        myACC.refreshridCount()

        ## calcu possible radar id base on only radar info
        myACC.trackRadar()
        myACC.printACC()

        distTTC = np.array(distTTCList)
        nowImg_radar = np.array(radarList)
        if distTTC.size and nowImg_radar.size:
            radarImg, fusion_radar = render_radar_on_image(nowImg_radar, nowImg.copy(), calib, img_width, img_height, distTTC)
            # print(radarImg)
            
            DistImg = drawBbox2Img(nowImg.copy(), myBBs, fusion_radar)
            bridge = CvBridge()

            # crop dual img roi and resize to "size_Dual"
            # if not oldCamera:
            #     radarImg = radarImg[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            # radarImg = cv2.resize(radarImg , size_Dual)

            if not oldCamera:
                DistImg = DistImg[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            DistImg = cv2.resize(DistImg , size_Dual)

            pub1.publish(bridge.cv2_to_imgmsg(radarImg))
            pub2.publish(bridge.cv2_to_imgmsg(DistImg))
        rate.sleep()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass