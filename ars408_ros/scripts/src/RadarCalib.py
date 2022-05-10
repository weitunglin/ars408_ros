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
fixACCrange = False

topic_RGB = config['topic_RGB_Calib']
topic_TRM = config['topic_TRM']
topic_Dual = config['topic_Dual']
topic_yolo = config['topic_yolo']
topic_Bbox = config['topic_Bbox']
topic_Radar = config['topic_Radar']
topic_RadarCalib = config['topic_RadarCalib']

size_RGB = config['size_RGB_Calib']
size_RGB = config['size_RGB_720p']
size_TRM = config['size_TRM']
size_Dual = config['size_Dual']

# 內部參數
img_width = config['size_RGB_Calib_output'][0]
img_height = config['size_RGB_Calib_output'][1]
pixelTime = img_width / config['size_RGB_Calib'][0]
pixelTime = 1
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

global nowImg, myPoints
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

def render_radar_on_image(pts_radar, img, calib, img_width, img_height):
    global trackID, trackInfo, lasttrackInfo
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

    WeightedColor = (0, 69, 255)
    RadarColor = (0, 36, 255)
    textColor = (255, 255, 255)
    fontSize = 0.5
    fontThickness = 1
    labelSizeOffset = 1
    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        depthV = min(255, int(820 / depth))
        circle_size = 30 / 255 * depthV + 4 * textTime
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i]) * pixelTime),
                         int(np.round(imgfov_pc_pixel[1, i]) * pixelTime)),
                   int(circle_size), color=tuple(RadarColor), thickness=-1)

        # put coordinate text
        coordinate = (int(np.round(imgfov_pc_pixel[0, i]) * pixelTime), int(np.round(imgfov_pc_pixel[1, i]) * pixelTime))
        coordinatetext = "{x}, {y}".format(x = coordinate[0], y = coordinate[1])
        labelSize = cv2.getTextSize(coordinatetext, cv2.FONT_HERSHEY_SIMPLEX, fontSize, fontThickness)[0]
        sub_img_x =  (coordinate[0], coordinate[0] + labelSize[0])
        sub_img_y =  (coordinate[1] - labelSize[1] * (1 - labelSizeOffset), coordinate[1] + labelSize[1] * labelSizeOffset)

        sub_img = img[sub_img_y[0]:sub_img_y[1], sub_img_x[0]:sub_img_x[1]]
        rect = np.ones(sub_img.shape, dtype=np.uint8) 
        rect[:][:] = WeightedColor
        res = cv2.addWeighted(sub_img, 0.2, rect, 0.8, 0)
        img[sub_img_y[0]:sub_img_y[1], sub_img_x[0]:sub_img_x[1]] = res
        cv2.putText(img, coordinatetext, (sub_img_x[0], sub_img_y[0] + labelSize[1]), cv2.FONT_HERSHEY_SIMPLEX, fontSize, textColor, fontThickness, cv2.LINE_AA)

    return img

def callbackPoint(data):
    global myPoints
    myPoints.radarPoints = data.rps

def callbackImg(data):
    global nowImg
    bridge = CvBridge()
    nowImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def listener(): 
    global nowImg, myPoints
    rospy.init_node("RadarCalib")
    rate = rospy.Rate(frameRate)
    myPoints = RadarState()
    sub1 = rospy.Subscriber(topic_Radar, RadarPoints, callbackPoint, queue_size=1)
    sub2 = rospy.Subscriber(topic_RGB, Image, callbackImg, queue_size=1)
    pub1 = rospy.Publisher(topic_RadarCalib, Image, queue_size=1)

    while not rospy.is_shutdown():
        if not ("nowImg"  in globals() and "myPoints" in globals()):
            continue
        radarList = []
        for point in np.array(myPoints.radarPoints):
            pt_x = point.distX
            pt_y = point.distY 
            pt_z = 0
            radarList.append([pt_x,pt_y,pt_z])
            
        nowImg_radar = np.array(radarList)
        if nowImg_radar.size:
            radarCalibImg = render_radar_on_image(nowImg_radar, nowImg.copy(), calib, img_width, img_height)
            bridge = CvBridge()
            pub1.publish(bridge.cv2_to_imgmsg(radarCalibImg))
        rate.sleep()
    
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass