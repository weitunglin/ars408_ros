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

# 內部參數
# size_RGB = (640 * pixelTime, 480 * pixelTime)  
img_width = 640
img_height = 480
pixelTime = 1

# 外部參數
img2Radar_x = -0.1        # 影像到雷達的距離 (m)
img2Radar_y = 1.3         # 影像到雷達的距離 (m)
img2Radar_z = 1.6         # 影像到雷達的距離 (m)

global nowImg, pub1, pub2, myBB

calib = {
    # 'P_rect':np.array([350, 0., 300.09341, 0., 0, 300, 250, 0, 0, 0, 1, 0]),
    'P_rect':np.array([309.89199829, 0., 301.24400423, 0., 0, 314.19360352, 255.7728296, 0, 0, 0, 1, 0]),
    'R0_rect':np.array([1., 0., 0., 0., 1., 0., 0., 0., 1.]),
    'Tr_velo_to_cam':np.array([0., -1., 0., img2Radar_x, 0., 0., -1., img2Radar_y, 1., 0., 0., img2Radar_z]),
}

class BoundingBox():
    def __init__(self):
        self.bboxes = []
        
def project_velo_to_cam2(calib):
    P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4), np.array([0., 0., 0., 1.])))  # velo2ref_cam
    R_ref2rect = np.eye(4)
    R0_rect = calib['R0_rect'].reshape(3, 3)  # ref_cam2rect
    R_ref2rect[:3, :3] = R0_rect
    P_rect2cam2 = calib['P_rect'].reshape((3, 4))
    proj_mat = np.dot(P_rect2cam2 , np.dot(R_ref2rect, P_velo2cam_ref))
    return proj_mat

def project_to_image(points, proj_mat):
    num_pts = points.shape[1]
    
    # Change to homogenous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = np.dot(proj_mat, points)
    points[:2, :] /= points[2, :]
    return points[:2, :]

def render_lidar_on_image(pts_velo, img, calib, img_width, img_height, distTTC):
    # projection matrix (project from velo2cam2)
    proj_velo2cam2 = project_velo_to_cam2(calib)

    # apply projection
    pts_2d = project_to_image(pts_velo.transpose(), proj_velo2cam2)

    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < img_width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_height) & (pts_2d[1, :] >= 0) &
                    (pts_velo[:, 0] > 0)
                    )[0]
    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from lidar
    imgfov_pc_velo = pts_velo[inds, :]
    imgfov_pc_velo = np.hstack((imgfov_pc_velo, np.ones((imgfov_pc_velo.shape[0], 1))))
    imgfov_pc_cam2 = np.dot(proj_velo2cam2, imgfov_pc_velo.transpose())

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
        circlr_size = 30 / 255 * depthV + 4 * pixelTime
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i]) * pixelTime),
                         int(np.round(imgfov_pc_pixel[1, i]) * pixelTime)),
                   int(circlr_size), color=tuple(color), thickness=-1)
        fusion_radar.append((int(np.round(imgfov_pc_pixel[0, i]) * pixelTime) , int(np.round(imgfov_pc_pixel[1, i]) * pixelTime), distTTC[i][0], distTTC[i][1], int(circlr_size)))
    return img, fusion_radar

def drawBbox2Img(img, bboxes, fusion_radar):
    for i in bboxes.bboxes:
        bboxColor = (0, 255, 0)
        textColor = (255, 255, 255)
        fontSize = 0.5
        fontThickness = 1
        leftTop = (i.x_min, i.y_min)
        rightBut = (i.x_max, i.y_max)

        bboxcircle = (-1, -1)
        bboxcirclesize = -1
        minDist = 99999
        for radarpoint in fusion_radar:
            if radarpoint[0] > leftTop[0] and radarpoint[0]< rightBut[0] and radarpoint[1] > leftTop[1] and radarpoint[1]< rightBut[1]:
                if radarpoint[2] < minDist:
                    bboxColor = (0, 255, 0)
                    bboxcircle = (radarpoint[0], radarpoint[1])
                    bboxcirclesize = radarpoint[4]
                    minDist = radarpoint[2]
                    if radarpoint[3] == True:
                        bboxColor = (0, 0, 255)
        
        yoloText =  "{0}".format(i.objClass)
        # yoloText =  "{0}: {1:0.2f}, ".format(i.objClass, i.score)
        disText = ": Null"
        if minDist != 99999:
            disText = ": {0:0.2f} m".format(minDist)

        labelSize = cv2.getTextSize(yoloText + disText, cv2.FONT_HERSHEY_SIMPLEX, fontSize * pixelTime, int(fontThickness * pixelTime))[0]
        sub_img = img[leftTop[1] - int(12 * pixelTime):leftTop[1], leftTop[0]:leftTop[0] + labelSize[0] - int(4 * pixelTime)]
        blue_rect = np.ones(sub_img.shape, dtype=np.uint8) 
        blue_rect[:][:] = (255, 0, 0)
        res = cv2.addWeighted(sub_img, 0.5, blue_rect, 0.5, 0)
        img[leftTop[1] - int(12 * pixelTime):leftTop[1], leftTop[0]:leftTop[0] + labelSize[0] - int(4 * pixelTime)] = res
        cv2.rectangle(img, leftTop, rightBut, bboxColor, int(pixelTime))
        if bboxcircle[0] != -1:
            cv2.circle(img, bboxcircle, bboxcirclesize, (18, 153, 255), thickness=-1)
            cv2.line(img, bboxcircle, (int((leftTop[0] + rightBut[0]) / 2), int((leftTop[1] + rightBut[1]) / 2)), (18, 153, 255), int(pixelTime))
        cv2.putText(img, yoloText + disText, (leftTop[0], leftTop[1] - int(2 * pixelTime)), cv2.FONT_HERSHEY_SIMPLEX, fontSize * pixelTime, textColor, int(fontThickness * pixelTime), cv2.LINE_AA)

    return img

def callbackPoint(data):
    radarList = []
    distTTCList = []
    for point in np.array(data.rps):
            pt_x = point.distX
            pt_y = point.distY 
            pt_z = 0
            radarList.append([pt_x,pt_y,pt_z])
            dist = math.sqrt(point.distX**2 + point.distY**2)
            TTC = False
            if point.isDanger:
                TTC = True
            distTTCList.append((dist, TTC))

    distTTC = np.array(distTTCList)
    nowImg_radar = np.array(radarList)
    if ("nowImg" in globals() and nowImg_radar.size != 0):
        myimg = nowImg[141:475, 164:550]
        myimg = cv2.resize(myimg , (640, 480))
        radarImg, fusion_radar = render_lidar_on_image(nowImg_radar, myimg.copy(), calib, img_width, img_height, distTTC)
        DistImg = drawBbox2Img(myimg.copy(), myBB, fusion_radar)
        bridge = CvBridge()
        pub1.publish(bridge.cv2_to_imgmsg(radarImg))
        pub2.publish(bridge.cv2_to_imgmsg(DistImg))

def callbackImg(data):
    global nowImg
    bridge = CvBridge()
    nowImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def callback_Bbox(data):
    global myBB
    myBB.bboxes = data.bboxes

def listener(): 
    global nowImg, pub1, pub2, myBB
    rospy.init_node("plotRadar")
    myBB = BoundingBox()
    sub1 = rospy.Subscriber("/radarPub", RadarPoints, callbackPoint, queue_size=1)
    sub2 = rospy.Subscriber("/Bbox", Bboxes, callback_Bbox, queue_size=1)
    sub3 = rospy.Subscriber("/dualImg", Image, callbackImg, queue_size=1)
    pub1 = rospy.Publisher("/radarImg", Image, queue_size=1)
    pub2 = rospy.Publisher("/DistImg", Image, queue_size=1)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass