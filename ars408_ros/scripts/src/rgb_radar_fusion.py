#! /usr/bin/env python3
# coding=utf-8
import rospy

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import RadarPoints
import message_filters
from numpy.polynomial import polyutils as pu


class YOLOv8Detector:
    def __init__(self, bridge):
        self.model = YOLO('/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/weights/yolov8m.pt')
        self.bridge = bridge

    def detect(self, cv_image):
        preds = self.model.predict(cv_image)[0]

        annotated_img = preds.plot(conf=False)

        # for pred in preds:
        #     rospy.loginfo(pred)

        annotated_img_msg = self.bridge.cv2_to_imgmsg(annotated_img)

        return preds, annotated_img


class RadarPoint():
    def __init__(self, point_x=0, point_y=0, point_speed=0, point_dist=0):
        self.point_x = point_x
        self.point_y = point_y
        self.point_speed = point_speed
        self.point_dist = point_dist


def calculate_radar_points_averages(points):
    if len(points) == 1:
        return points[0]

    avg = RadarPoint()

    for p in points:
        avg.point_x += p.point_x
        avg.point_y += p.point_y
        avg.point_speed += p.point_speed
        avg.point_dist += p.point_dist
    
    avg.point_x /= len(points)
    avg.point_y /= len(points)
    avg.point_speed /= len(points)
    avg.point_dist /= len(points)

    return avg


class RadarRGBVisualizer():
    def __init__(self, use_time_synchronizer=True):
        if use_time_synchronizer:
            self.sub_image = message_filters.Subscriber("/camera/image_color", Image)
            self.sub_radar = message_filters.Subscriber("/radar/front_center/decoded_messages", RadarPoints)

            self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar], 10, 0.2, reset=True)
            self.sync.registerCallback(self.callback)
        else:
            self.sub_image = rospy.Subscriber("/camera/image_color", Image, self.image_cb, queue_size=1)
            self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_cb, queue_size=1)

        self.pub_fusion_image = rospy.Publisher("/radar_rgb/image", Image, queue_size=1)
        self.pub_yolo_image = rospy.Publisher("/yolo_detection/image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image = None
        self.radar_points = None

        self.detector = YOLOv8Detector(bridge=self.bridge)

        """
        camera matrix
        [2033,    0, 1068]
        [0   , 2056,  539]
        [0   ,    0,    1]
        """
        self.p = np.array([1068, -2033, 0, 0, 539, 0, -2056, 0, 1, 0, 0, 0]).reshape((3, 4))

        self.image_width, self.image_height = 2048, 1536
    
    def image_cb(self, image: Image):
        self.image = image
    
    def radar_cb(self, radar_points: RadarPoints):
        self.radar_points = radar_points
    
    def project_to_radar(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack([points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        return points[:3, :]

    def project_to_image(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack((points, np.ones((1, num_pts))))
        points = projection_matrix @ points
        points[:2, :] /= points[2, :]
        return points[:2, :]
    
    def callback(self, image, radar_points):
        self.image = image
        self.radar_points = radar_points
        self.loop()
    
    def loop(self):
        if not self.image or not self.radar_points:
            return

        radar_image = image = self.bridge.imgmsg_to_cv2(self.image)
        preds, annotated_image = self.detector.detect(image)
        preds = preds.cpu()

        rospy.loginfo_throttle(5, preds.boxes.shape)

        box_radar_mapping = [[] for i in range(preds.boxes.shape[0])]
        rospy.loginfo_throttle(5, box_radar_mapping)

        radar_points = self.radar_points.rps
        points_3d = np.empty(shape=(0, 3))
        for p in radar_points:
            points_3d = np.append(points_3d, [[p.distX, p.distY, 1.0]], axis=0)
        
        # points_2d = self.project_to_radar(points_3d.transpose(), self.p)
        points_2d = self.project_to_image(points_3d.transpose(), self.p)

        scale_coordinate = False
        if scale_coordinate:
            points_2d[0, :] = points_2d[0, :] / 1280 * 2048
            points_2d[1, :] = points_2d[1, :] / 720 * 1536
        
        inds = np.where((points_2d[0, :] < self.image_width*0.9) & (points_2d[0, :] >= self.image_width*0.1)
                & (points_2d[1, :] < self.image_height) & (points_2d[1, :] >= 0)
                & (points_3d[:, 0] > 0)
                )[0]

        points_2d = points_2d[:, inds]
        points_3d = points_3d[inds, :]
        radar_points = [radar_points[i] for i in inds]

        # offset_x, offset_y = 100, 0
        # radar_image = radar_image[:, offset_x:-offset_x]
        # radar_image = cv2.resize(radar_image, (self.image_width, self.image_height))
        # rospy.loginfo_once(radar_image.shape)

        # rospy.loginfo_once((radar_image.shape))

        print_radar_info = True
        print_box_radar_mapping = False
        box_ids = []

        scale_x_old_domain = (0, 2047)
        scale_x_new_domain = (0.75, 1.25)

        scale_y_old_domain = (15, 60)
        scale_y_new_domain = (0.8, 1.6)

        for p in range(points_2d.shape[1]):
            depth = (80 - points_3d[p, 0]) / 80
            length = max(int(80 * depth), 40)
            
            point_x = int(points_2d[0, p] * 1)
            point_y = int(points_2d[1, p] * 1) # 0.7

            point_speed = np.sqrt(pow(radar_points[p].vrelX, 2) + pow(radar_points[p].vrelY, 2)) # radar point speed in m/s
            point_dist = np.sqrt(pow(radar_points[p].distX, 2) + pow(radar_points[p].distY, 2))

            # 0: not post process
            # 1: Scene C (mcdo)
            # 2: Scene B (chicken)
            # 3: Scene A (elementary school)
            hard_postprocess = 1
            image_width = radar_image.shape[1]
            image_height = radar_image.shape[0]
            if hard_postprocess == 1:
                if point_y < image_height / 2:
                    old_point_y = point_y
                    scale_y = pu.mapdomain(point_dist, scale_y_old_domain, scale_y_new_domain)
                    point_y = int(image_height - point_y * scale_y)
                    scale_x = pu.mapdomain(point_x, scale_x_old_domain, scale_x_new_domain)
                    if scale_x > 1:
                        scale_x = max(scale_x, 1.1)
                    else:
                        scale_x = min(0.9, scale_x)
                    point_x = int(point_x * scale_x)

            # if p_speed > 1 or radar_points[p].distX < 10:
            # if (radar_points[p].classT != 7 and point_dist < 20) and point_speed > 0.5:
            if point_x > 500 and point_dist > 15 and point_dist < 50 and point_speed > 1:
                # check if inside bounding box
                # box_id = None
                for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
                    box_ids.append(i)
                    if point_x > (x - w / 1.7) and point_x < (x + w / 1.7) \
                        and point_y > (y - h / 1.7) and point_y < (y + h / 1.7):
                        box_radar_mapping[i].append(RadarPoint(point_x, point_y, point_speed, point_dist))

                if print_radar_info:
                    cv2.putText(radar_image, f"{point_speed * 3.6:.1f} km/h", (point_x, point_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(radar_image, f"{point_dist:.1f} m", (point_x, point_y+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                    # cv2.putText(radar_image, f"{radar_points[p].classT}", (point_x, point_y+100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)
                    cv2.line(radar_image, (point_x, point_y+length), (point_x, point_y), (0, int(255 * abs(depth)), 50), thickness=6)

                # if box_id != None:
                #     x,y,w,h = preds.boxes.xywh[box_id]
                #     cls = preds.names[int(preds.boxes.cls[box_id])]
                #     # rospy.loginfo(((x, y), (x+w, y+h)))
                #     cv2.putText(radar_image, f"{cls} {point_dist:.1f} m {point_speed * 3.6:.1f} km/h", (int(x - w / 2), int(y - h / 2)-25), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)
                #     cv2.rectangle(radar_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 3)

            # if print_radar_info:
            #     cv2.line(radar_image, (point_x, point_y+length), (point_x, point_y), (0, int(255 * abs(depth)), 50), thickness=6)

        if print_box_radar_mapping:
            for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
                cv2.rectangle(radar_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
                cls = preds.names[int(preds.boxes.cls[i])]
            
                if len(box_radar_mapping[i]):
                    avg = calculate_radar_points_averages(box_radar_mapping[i])
                    cv2.putText(radar_image, f"{cls} {avg.point_dist:.1f}m {avg.point_speed * 3.6:.1f}km/h", (int(x - w / 2), int(y - h / 2)-15), cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 255, 0), 2, cv2.LINE_AA)


        radar_image_msg = self.bridge.cv2_to_imgmsg(radar_image)
        self.pub_fusion_image.publish(radar_image_msg)

        # annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image)
        # self.pub_yolo_image.publish(annotated_image_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("Radar RGB Visualizer")

        use_time_synchronizer = False
        v = RadarRGBVisualizer(use_time_synchronizer=use_time_synchronizer)

        if use_time_synchronizer:
            rospy.spin()
        else:
            r = rospy.Rate(60)
            while not rospy.is_shutdown():
                v.loop()
                r.sleep()

    except rospy.ROSException as e:
        rospy.logerr(str(e))
