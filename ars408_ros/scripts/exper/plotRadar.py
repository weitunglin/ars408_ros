#! /usr/bin/env python3
# coding=utf-8

import cv2
import os
import rospy
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
import time
from ars408_msg.msg import RadarPoints, RadarPoint
# from pypcd import pypcd

global nowImg
pub = rospy.Publisher("/radarImg", Image, queue_size=1)
global bridge
bridge = CvBridge()

calib = {
    # origin img on github is 1242 x 375
    # 'P_rect':{fu, 0, mid_of_img(x), 0, 0, fv, mid_of_img(y), 0, 0, 0, 1, 0}
    # 'P_rect':np.array([840,0,310,0.000000000000e+00,0.000000000000e+00,760,230,0.000000000000e+00,0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,0.000000000000e+00]),
    'P_rect':np.array([350, 0, 300.09341, 0.000000000000e+00, 0.000000000000e+00, 300, 250, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00]),
    'R0_rect':np.array([9.999239000000e-01, 9.837760000000e-03, -7.445048000000e-03, -9.869795000000e-03, 9.999421000000e-01, -4.278459000000e-03, 7.402527000000e-03, 4.351614000000e-03, 9.999631000000e-01]),
    'Tr_velo_to_cam':np.array([7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -2.717806000000e-01]),
}

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

def render_lidar_on_image(pts_velo, img, calib, img_width, img_height):
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

    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        depthV = min(255, int(820 / depth))
        color = cmap[depthV, :]
        circlr_size = 30 / 255 * depthV
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i])),
                         int(np.round(imgfov_pc_pixel[1, i]))),
                   int(circlr_size), color=tuple(color), thickness=-1)
    return img

def callbackPoint(data):
    global bridge
    radarList = []
    for point in np.array(data.rps):
            pt_x = point.distX + 2
            pt_y = point.distY 
            pt_z = -1.8
            radarList.append([pt_x,pt_y,pt_z])
    nowImg_radar = np.array(radarList)

    if ("nowImg" in globals()):
        img_height= 480
        img_width = 640

        fusion = render_lidar_on_image(nowImg_radar, nowImg, calib, img_width, img_height)
        img_message = bridge.cv2_to_imgmsg(fusion)
        pub.publish(img_message)

def callbackImg(data):
    global nowImg, bridge
    nowImg = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def listener(): 
    rospy.init_node("plotRadar")
    rospy.Subscriber("/radarPub", RadarPoints, callbackPoint, queue_size=1)
    rospy.Subscriber("/image_rect_color", Image, callbackImg, queue_size=1)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass