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

global nowImg_Lidar
is_lidar = False
pub = rospy.Publisher("/lidarImg", Image, queue_size=1)

calib = {
    'P_rect':np.array([700,0.000000000000e+00,395,0.000000000000e+00,0.000000000000e+00,700,300,0.000000000000e+00,0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,0.000000000000e+00]),
    'R0_rect':np.array([9.999239000000e-01,9.837760000000e-03,-7.445048000000e-03,-9.869795000000e-03,9.999421000000e-01,-4.278459000000e-03,7.402527000000e-03,4.351614000000e-03,9.999631000000e-01]),
    'Tr_velo_to_cam':np.array([7.533745000000e-03,-9.999714000000e-01,-6.166020000000e-04,-4.069766000000e-03,1.480249000000e-02,7.280733000000e-04,-9.998902000000e-01,-7.631618000000e-02,9.998621000000e-01,7.523790000000e-03,1.480755000000e-02,-2.717806000000e-01]),
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
                    (nowImg_Lidar[:, 0] > 0)
                    )[0]
    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from lidar
    imgfov_pc_velo = pts_velo[inds, :]
    imgfov_pc_velo = np.hstack((imgfov_pc_velo, np.ones((imgfov_pc_velo.shape[0], 1))))
    imgfov_pc_cam2 = np.dot(proj_velo2cam2, imgfov_pc_velo.transpose())

    cmap = plt.cm.get_cmap('hsv', 256)
    cmap = np.array([cmap(i) for i in range(255)])[:, :3] * 255
    
    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        color = cmap[int(820 / depth), :]
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i])),
                         int(np.round(imgfov_pc_pixel[1, i]))),
                   3, color=tuple(color), thickness=-1)
    return img


def callbackPoint(data):
    global nowImg_Lidar
    lidarList = []
    for point in point_cloud2.read_points(data, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            lidarList.append([pt_x,pt_y,pt_z])
    nowImg_Lidar = np.array(lidarList)
    global is_lidar
    is_lidar = True
    
def callbackImg(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    global is_lidar
    if is_lidar:
        lidarOnImage(img, nowImg_Lidar)

def lidarOnImage(img, lidarPoints):
    global nowImg_Lidar
    bridge = CvBridge()
    img_height= 680
    img_width = 840
    fusion = render_lidar_on_image(nowImg_Lidar, img, calib, img_width, img_height)
    img_message = bridge.cv2_to_imgmsg(fusion)
    pub.publish(img_message)
    global is_lidar
    is_lidar = False


def listener(): 
    rospy.init_node("plotLidar")
    rospy.Subscriber("/velodyne_points", PointCloud2, callbackPoint)
    rospy.Subscriber("/rgbImg", Image, callbackImg)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass