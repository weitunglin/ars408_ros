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
# size_RGB = (640 * 2.5, 480 * 2.5)  
img_width = 1600
img_height = 1200
fov_width = 28
fov_height = 40
pixelTime = 2.5

# 外部參數
img2Radar_x = 210        # 影像到雷達的距離 (cm)
img2Radar_y = 10         # 影像到雷達的距離 (cm)
img2Radar_z = 180        # 影像到地板的距離 (cm)
img2ground = 2           # 影像照到地板的距離 (m)

global nowImg, pub1, pub2, myBB

calib = {
    # origin img on github is 1242 x 375
    # 'P_rect':{fu, 0, mid_of_img(x), 0, 0, fv, mid_of_img(y), 0, 0, 0, 1, 0}
    # 'P_rect':np.array([840,0,310,0.000000000000e+00,0.000000000000e+00,760,230,0.000000000000e+00,0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,0.000000000000e+00]),
    'P_rect':np.array([350, 0, 300.09341, 0.000000000000e+00, 0.000000000000e+00, 300, 250, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00]),
    'R0_rect':np.array([9.999239000000e-01, 9.837760000000e-03, -7.445048000000e-03, -9.869795000000e-03, 9.999421000000e-01, -4.278459000000e-03, 7.402527000000e-03, 4.351614000000e-03, 9.999631000000e-01]),
    'Tr_velo_to_cam':np.array([7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -2.717806000000e-01]),
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
    """
    Apply the perspective projection
    Args:
        pts_3d:     3D points in camera coordinate [3, npoints]
        proj_mat:   Projection matrix [3, 4]
    """
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
    img_xy = []
    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        depthV = min(255, int(820 / depth))
        color = cmap[depthV, :]
        circlr_size = 30 / 255 * depthV + 4 * pixelTime
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i]) * pixelTime),
                         int(np.round(imgfov_pc_pixel[1, i]) * pixelTime)),
                   int(circlr_size), color=tuple(color), thickness=-1)
        img_xy.append((int(np.round(imgfov_pc_pixel[0, i]) * pixelTime) , int(np.round(imgfov_pc_pixel[1, i]) * pixelTime), distTTC[i][0], distTTC[i][1]))
    return img, img_xy

def drawBbox2Img(img, bboxes, img_xy):
    for i in bboxes.bboxes:
        bboxColor = (0, 255, 0)
        textColor = (255, 255, 255)
        fontSize = 0.5
        fontThickness = 1
        leftTop = (int(i.x_min * pixelTime), int(i.y_min * pixelTime))
        rightBut = (int(i.x_max * pixelTime), int(i.y_max * pixelTime))

        minDist = 99999
        for xy in img_xy:
            if xy[0] > leftTop[0] and xy[0]< rightBut[0] and xy[1] > leftTop[1] and xy[1]< rightBut[1]:
                if xy[2] < minDist:
                    bboxColor = (0, 255, 0)
                    minDist = xy[2]
                    if xy[3] == True:
                        bboxColor = (0, 0, 255)
        
        yoloText =  "{0}".format(i.objClass)
        # yoloText =  "{0}: {1:0.2f}, ".format(i.objClass, i.score)
        disText = ": Null"
        if minDist != 99999:
            disText = ": {0:0.2f} m".format(minDist)

        labelSize = cv2.getTextSize(yoloText + disText, cv2.FONT_HERSHEY_SIMPLEX, fontSize * pixelTime, int(fontThickness * pixelTime))[0]
        cv2.rectangle(img, leftTop, rightBut, bboxColor, 2)
        cv2.rectangle(img, leftTop, (leftTop[0] + labelSize[0] - 15, leftTop[1] - 30), (255, 0, 0, 0.6), thickness=-1)
        cv2.putText(img, yoloText + disText, (leftTop[0] - 5, leftTop[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, fontSize * pixelTime, textColor, int(fontThickness * pixelTime), cv2.LINE_AA)

    return img

def callbackPoint(data):
    radarList = []
    distTTCList = []
    for point in np.array(data.rps):
            pt_x = point.distX + 2
            pt_y = point.distY 
            pt_z = -1.5
            radarList.append([pt_x,pt_y,pt_z])

            dist = math.sqrt(point.distX**2 + point.distY**2)                   # 算距離 (m)
            TTC = False
            if point.vrelX == 0:  # 直向等速
                if abs(point.distX) < 1 and point.vrelY != 0 and dist / abs(point.vrelY) < 4.0:
                    TTC = True
            elif point.vrelY == 0: # 橫向等速
                if abs(point.distY) < 1 and point.vrelX < 0 and dist / -point.vrelX < 4.0:
                    TTC = True
            elif dist / math.sqrt(point.vrelX**2 + point.vrelY**2) < 4.0 and point.vrelX < 0 and point.isDanger:
                TTC = True
            distTTCList.append((dist, TTC))

    distTTC = np.array(distTTCList)
    nowImg_radar = np.array(radarList)
    if ("nowImg" in globals()):
        radarImg, img_xy = render_lidar_on_image(nowImg_radar, nowImg.copy(), calib, img_width, img_height, distTTC)
        DistImg = drawBbox2Img(nowImg.copy(), myBB, img_xy)
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