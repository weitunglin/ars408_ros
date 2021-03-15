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
from ars408_msg.msg import RadarPoints, RadarPoint
from ars408_msg.msg import Bboxes, Bbox
from ars408_msg.msg import GPSinfo
import yaml
import random

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

frameRate = config['frameRate']
oldCamera = config['oldCamera']
printACC = False

topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']
topic_yolo = config['topic_yolo']
topic_Bbox = config['topic_Bbox']
topic_Radar = config['topic_Radar']
topic_RadarImg = config['topic_RadarImg']
topic_DistImg = config['topic_DistImg']
topic_GPS = config['topic_GPS']

size_RGB = config['size_RGB_Calib']
size_TRM = config['size_TRM']
size_Dual = config['size_Dual']

# 內部參數
img_width = config['size_RGB_Calib_output'][0]
img_height = config['size_RGB_Calib_output'][1]
pixelTime = img_width / config['size_RGB_Calib'][0]
textTime = config['textTime']
scoreScale = math.sqrt(config['size_RGB_Calib_output'][0] ** 2 + config['size_RGB_Calib_output'][1] ** 2)

# 外部參數
img2Radar_x = config['img2Radar_x']
img2Radar_y = config['img2Radar_y']
img2Radar_z = config['img2Radar_z']

# crop
ROI = config['ROI']
crop_x = (ROI[1][0], ROI[1][1])
crop_y = (ROI[0][0], ROI[0][1])

cmatrix = np.array(config['K']).reshape(3,3)
dmatrix = np.array(config['D']).reshape(1,5)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cmatrix, dmatrix, size_RGB, 1, size_RGB)

if oldCamera:
    pixelTime = 1
    newcameramtx = cmatrix
    size_Dual = (800, 600)

global nowImg, myBBs, myPoints, myGPS, trackID, trackIDList, trackInfo

calib = {
    'P_rect':np.hstack((newcameramtx, np.array([[0.], [0.], [0.]]))),
    'R_rect':np.array(config['R']),
    'Tr_radar_to_cam':np.array(config['r2c']),
}

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
    r2c = r2c@rotate
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
    global trackID
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
        if distTTC[i][1]:
            color = (0, 0, 255)
        elif distTTC[i][2] == trackID and trackID == trackInfo[0]:
            color = (0, 0, 0)
        circle_size = 30 / 255 * depthV + 4 * textTime
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i]) * pixelTime),
                         int(np.round(imgfov_pc_pixel[1, i]) * pixelTime)),
                   int(circle_size), color=tuple(color), thickness=-1)
        fusion_radar.append((int(np.round(imgfov_pc_pixel[0, i]) * pixelTime) , int(np.round(imgfov_pc_pixel[1, i]) * pixelTime), distTTC[i], int(circle_size)))
    return img, fusion_radar

def drawBbox2Img(img, bboxes, fusion_radar):
    global trackID, trackIDList, trackInfo
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

        useFP = False
        printBBoxcircle = False
        bboxcircle = (-1, -1)
        bboxcirclesize = -1
        bboxcirclecolor = (18, 153, 255)
        minDist = 99999
        scoreDist = 99999
        trackMin = 99999
        limitFrame = 20
        refreshFrame = 10
        for radarpoint in fusion_radar:
            # fusion_radar: list[circleX, circleY, list[distTTC], circleSize]
            # radarpoint: [circleX, circleY, list[distTTC], circleSize]
            # distTTC: [dist, point.isDanger, point.id]
            # trackInfo: [point.id, dist, missingFrame]
            # print(trackInfo)
            if radarpoint[0] > leftTop[0] and radarpoint[0]< rightBut[0] and radarpoint[1] > leftTop[1] and radarpoint[1]< rightBut[1]:
                if radarpoint[2][0] < minDist:
                    bboxColor = (0, 255, 0)
                    minDist = radarpoint[2][0]
                    if printBBoxcircle:
                        bboxcircle = (radarpoint[0], radarpoint[1])
                        bboxcirclesize = radarpoint[3]
                    if radarpoint[2][1]:
                        bboxColor = (0, 0, 255)
                    if radarpoint[2][2] in trackIDList and intbbox.objClass == "car" and radarpoint[2][0] < trackMin:
                        trackMin = radarpoint[2][0]
                        # printBBoxcircle = True
                        # bboxcircle = (radarpoint[0], radarpoint[1])
                        # bboxcirclesize = radarpoint[3]
                        # bboxcirclecolor = (0, 0, 0)
                        if trackInfo[0] == radarpoint[2][2] or trackMin <= trackInfo[1]:
                            trackInfo = [radarpoint[2][2], radarpoint[2][0], 0]
                        else:
                            trackInfo = [trackInfo[0], trackInfo[1], trackInfo[2] + 1]
                            if trackInfo[2] >= refreshFrame:
                                trackInfo = [-1, 99999, 0]
            if useFP:
                x = ((leftTop[0] + rightBut[0]) / 2 - radarpoint[0]) ** 2
                y = ((leftTop[1] + rightBut[1]) / 2 - radarpoint[1]) ** 2
                scoreDist = min(scoreDist, math.sqrt(x+y))

        # reduce false positive
        if useFP:
            intbbox.score = (i.score - scoreDist / scoreScale) if scoreDist != 99999 and (leftTop[1] + rightBut[1]) > img_height else intbbox.score
            if intbbox.score < 0.25:
                print(intbbox.score, leftTop, rightBut)
                continue
        
        yoloText =  "{0}".format(intbbox.objClass)
        # yoloText =  "{0}: {1:0.2f}, ".format(intbbox.objClass, intbbox.score)
        disText = ": Null"
        if minDist != 99999:
            disText = ": {0:0.2f} m".format(minDist)

        textPosOffset = 0
        # if ((minDist >= 5 and minDist <= 10) or (minDist >= 36 and minDist <= 37)):
        #     textPosOffset = 10
        # elif minDist >= 20 and minDist <= 21:
        #     textPosOffset = -2
        # textPosOffset = 0 if random.random() > 0.5 else 10
        labelSize = cv2.getTextSize(yoloText + disText, cv2.FONT_HERSHEY_SIMPLEX, fontSize * textTime, int(fontThickness * textTime))[0]
        sub_img = img[leftTop[1] - int(12 * textTime) + textPosOffset:leftTop[1] + textPosOffset, leftTop[0]:leftTop[0] + labelSize[0] - int(4 * textTime)]
        blue_rect = np.ones(sub_img.shape, dtype=np.uint8) 
        blue_rect[:][:] = (255, 0, 0)
        res = cv2.addWeighted(sub_img, 0.0, blue_rect, 1.0, 0)
        img[leftTop[1] - int(12 * textTime) + textPosOffset:leftTop[1] + textPosOffset, leftTop[0]:leftTop[0] + labelSize[0] - int(4 * textTime)] = res
        cv2.rectangle(img, leftTop, rightBut, bboxColor, int(textTime))
        if printBBoxcircle and bboxcircle[0] != -1:
            cv2.circle(img, bboxcircle, bboxcirclesize, bboxcirclecolor, thickness=-1)
            cv2.line(img, bboxcircle, (int((leftTop[0] + rightBut[0]) / 2), int((leftTop[1] + rightBut[1]) / 2)), bboxcirclecolor, max(1, int(textTime)))
        cv2.putText(img, yoloText + disText, (leftTop[0], leftTop[1] - int(2 * textTime) + textPosOffset), cv2.FONT_HERSHEY_SIMPLEX, fontSize * textTime, textColor, int(fontThickness * textTime), cv2.LINE_AA)

    return img

def callbackPoint(data):
    global myPoints
    myPoints.radarPoints = data.rps

def callbackBbox(data):
    global myBBs
    myBBs.bboxes = data.bboxes

def callbackImg(data):
    global nowImg
    bridge = CvBridge()
    nowImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def callbackGPS(data):
    global myGPS
    myGPS.speed = data.speed
    myGPS.zaxis = data.zaxis
    myGPS.longitude = data.longitude
    myGPS.latitude = data.latitude
    myGPS.accX = data.accX
    myGPS.accY = data.accY
    myGPS.accZ = data.accZ

def listener(): 
    global nowImg, myBBs, myPoints, myGPS, trackID, trackIDList, trackInfo
    rospy.init_node("plotRadar")
    rate = rospy.Rate(frameRate)
    myPoints = RadarState()
    myBBs = BoundingBox()
    myGPS = GPS()
    trackID = -1
    trackIDList = []
    trackInfo = [-1, 99999, 0]
    sub1 = rospy.Subscriber(topic_Radar, RadarPoints, callbackPoint, queue_size=1)
    sub2 = rospy.Subscriber(topic_Bbox, Bboxes, callbackBbox, queue_size=1)
    sub3 = rospy.Subscriber(topic_Dual, Image, callbackImg, queue_size=1)
    sub4 = rospy.Subscriber(topic_GPS, GPSinfo, callbackGPS, queue_size=1)
    pub1 = rospy.Publisher(topic_RadarImg, Image, queue_size=1)
    pub2 = rospy.Publisher(topic_DistImg, Image, queue_size=1)

    ridCount = [[0, 0, 0, 0] for i in range(100)] # [frameCount, dist, vrel, missingFrame] 
    limitX = 100 
    limitY = 2
    limitFrame = 20
    refreshFrame = 10

    while not rospy.is_shutdown():
        if not ("nowImg"  in globals() and "myPoints" in globals()):
            continue
        radarList = []
        distTTCList = []
        ridlist = [] # list[[id, dist, vrel]]
        
        for point in np.array(myPoints.radarPoints):
            pt_x = point.distX
            pt_y = point.distY 
            pt_z = 0
            radarList.append([pt_x,pt_y,pt_z])
            dist = math.sqrt(point.distX**2 + point.distY**2)
            vrel = math.sqrt(point.vrelX**2 + point.vrelY**2)
            distTTCList.append([dist, point.isDanger, point.id])

            vrel = -vrel if point.vrelX < 0 else vrel
            if abs(point.distY) < limitY and dist < limitX:
                ridlist.append([point.id, dist, vrel])

        last = cur = [-1, 0, 0] # [id, dist, vrel]
        for i in range(len(ridlist)):
            last = cur
            cur = ridlist[i]
            ridCount[cur[0]][0] = ridCount[cur[0]][0] + 1
            ridCount[cur[0]][1] = cur[1]
            ridCount[cur[0]][2] = cur[2]
            ridCount[cur[0]][3] = 0

            # init missing id points
            for idx in range(last[0] + 1, cur[0]):
                ridCount[idx][3] = ridCount[idx][3] + 1 if ridCount[idx][3] < refreshFrame else ridCount[idx][3]
                if ridCount[idx][3] == refreshFrame:
                    ridCount[idx] = [0, 0, 0, 0]
            if i == len(ridlist) - 1:
                for idx in range(cur[0] + 1, 100):
                    ridCount[idx][3] = ridCount[idx][3] + 1 if ridCount[idx][3] < refreshFrame else ridCount[idx][3]
                    if ridCount[idx][3] == refreshFrame:
                        ridCount[idx] = [0, 0, 0, 0]


        maxval = max([x[0] for x in ridCount])
        trackData = [0, 0, 0, 0] # [frameCount, dist, vrel, id]
        trackID = -1
        trackIDList = []
        for i in range(len(ridCount)):
            if ridCount[i][0] >= limitFrame:
                if trackData[0] >= limitFrame and trackData[1] < ridCount[i][1]:
                    trackData = trackData  
                else: 
                    trackData = ridCount[i][:3] + [i]
                    trackIDList.append(trackData[3])

        if trackData[0] >= limitFrame:
            trackID = trackData[3]
            status = "加速" if trackData[2] > 0 else "減速"
            status = "等速" if abs(trackData[2]) < 1 else status
            if printACC:
                print("MaxFrame:" + str(maxval) + "  TrackFrame:" + str(trackData[0]))
                print("    ID:" + str(trackData[3]) + "  Dist:{:.4f}".format(trackData[1]) + "m  Speed:{:.4f}".format(myGPS.speed) + "m/s  Vrel:{:.4f}".format(trackData[2]) + "m/s  status:" + status)
        elif printACC:
            print("未針測到前方測量 維持車速:20m/s")
            
        distTTC = np.array(distTTCList)
        nowImg_radar = np.array(radarList)
        if distTTC.size and nowImg_radar.size:
            radarImg, fusion_radar = render_radar_on_image(nowImg_radar, nowImg.copy(), calib, img_width, img_height, distTTC)
            DistImg = drawBbox2Img(nowImg.copy(), myBBs, fusion_radar)
            bridge = CvBridge()

            # crop dual img roi and resize to "size_Dual"
            if not oldCamera:
                radarImg = radarImg[crop_y[0]:crop_y[1], crop_x[0]:crop_x[1]]
            radarImg = cv2.resize(radarImg , size_Dual)
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