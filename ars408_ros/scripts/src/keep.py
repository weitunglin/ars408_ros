#! /usr/bin/env python3
# coding=utf-8
from email import message
from os import access
import threading
from functools import partial
from collections import defaultdict
import math
from typing import List

import rospy
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from config.config import default_config, rgb_config, radar_config
from ars408_msg.msg import RadarPoints, Objects, Bboxes, Object, Bbox, RadarPoint
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from scripts.src.yolo_torch import YOLO

def draw_object_on_radar(self, objects: Objects):
    ref_id = 0

    def rotate(_x,_y,angle):
        angle = angle * -1
        x = math.cos(angle) * _x - math.sin(angle) * _y   
        y = math.sin(angle) * _x + math.cos(angle) * _y 
        return (x,y)

    def Draw_Surface(distX, distY, transX, transY, transRadius, width, height, length):

        # Rotate points
        points = [
            Point(x=rotate(distX-length/2,distY - width/2,transRadius)[0],
        y=rotate(distX-length/2,distY - width/2,transRadius)[1], z = height),
        Point(x=rotate(distX-length/2,distY+width/2,transRadius)[0],
        y=rotate(distX - length/2,distY + width/2,transRadius)[1], z = height),
        Point(x=rotate(distX+length/2,distY+width/2,transRadius)[0],
        y=rotate(distX+length/2,distY+width/2,transRadius)[1], z = height),
        Point(x=rotate(distX+length/2,distY-width/2,transRadius)[0],
        y=rotate(distX+length/2,distY-width/2,transRadius)[1], z = height),
        Point(x=rotate(distX - length/2,distY-width/2,transRadius)[0],
        y=rotate(distX-length/2,distY-width/2,transRadius)[1], z = height)
        ]

        # Calib coordinate
        for points_counter in range(len(points)):
            points[points_counter].x = points[points_counter].x + transX
            points[points_counter].y = points[points_counter].y + transY * -1

        return points

    def Draw_Beam(distX, distY, transX, transY, transRadius, width, height, length):

        # Rotate points
        points = [
            Point(x=rotate(distX-length/2,distY - width/2,transRadius)[0],
        y=rotate(distX-length/2,distY-width/2,transRadius)[1], z = 0),
        Point(x=rotate(distX-length/2,distY-width/2,transRadius)[0],
        y=rotate(distX-length/2,distY-width/2,transRadius)[1], z = height),
        Point(x=rotate(distX-length/2,distY+width/2,transRadius)[0],
        y=rotate(distX-length/2,distY+width/2,transRadius)[1], z = 0),
        Point(x=rotate(distX-length/2,distY+width/2,transRadius)[0],
        y=rotate(distX-length/2,distY+width/2,transRadius)[1], z = height),
        Point(x=rotate(distX+length/2,distY+width/2,transRadius)[0],
        y=rotate(distX+length/2,distY+width/2,transRadius)[1], z = 0),
        Point(x=rotate(distX+length/2,distY+width/2,transRadius)[0],
        y=rotate(distX+length/2,distY+width/2,transRadius)[1], z = height),
        Point(x=rotate(distX+length/2,distY-width/2,transRadius)[0],
        y=rotate(distX+length/2,distY-width/2,transRadius)[1], z = 0),
        Point(x=rotate(distX+length/2,distY-width/2,transRadius)[0],
        y=rotate(distX+length/2,distY-width/2,transRadius)[1], z = height),
        ]
        
        # Calib coordinate
        for points_counter in range(len(points)):
            points[points_counter].x = points[points_counter].x + transX
            points[points_counter].y = points[points_counter].y + transY * -1            

        return points

    markers = MarkerArray()
    for o in objects:

        marker_bottom = Marker(
            header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
            id = ref_id,
            type = Marker.LINE_STRIP,
            action = Marker.ADD,
            pose = Pose(
                position = Point(x=0,y=0,z=0.1),
                orientation = Quaternion(x=0,y=0,z=0,w=1.0)
            ),
            points = Draw_Surface(o.radar_info.distX,o.radar_info.distY,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],radar_config[o.radar_name].transform[2],o.radar_info.width,0.0,o.radar_info.height),
            color = ColorRGBA(r=1,g=0,b=0,a=1.0),
            scale = Vector3(x=0.1,y=0.5,z=0.1)
        )

        ref_id = ref_id + 1

        marker_floor = Marker(
            header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
            id = ref_id,
            type = Marker.LINE_STRIP,
            action = Marker.ADD,
            pose = Pose(
                position = Point(x=0,y=0,z=0.1),
                orientation = Quaternion(x=0,y=0,z=0,w=1.0)
            ),
            points = Draw_Surface(o.radar_info.distX,o.radar_info.distY,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],radar_config[o.radar_name].transform[2],o.radar_info.width,1,o.radar_info.height),
            color = ColorRGBA(r=1,g=0,b=0,a=1.0),
            scale = Vector3(x=0.1,y=0.5,z=0.1)
        )

        ref_id = ref_id + 1

        marker_beam = Marker(
            header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
            id = ref_id,
            type = Marker.LINE_LIST,
            action = Marker.ADD,
            pose = Pose(
                position = Point(x=0,y=0,z=0.1),
                orientation = Quaternion(x=0,y=0,z=0,w=1.0)
            ),
            points = Draw_Beam(o.radar_info.distX,o.radar_info.distY,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],radar_config[o.radar_name].transform[2],o.radar_info.width,1,o.radar_info.height),
            color = ColorRGBA(r=1,g=0,b=0,a=1.0),
            scale = Vector3(x=0.1,y=0.5,z=0.1)
        )

        ref_id = ref_id + 1

        markers.markers.append(marker_bottom)
        markers.markers.append(marker_floor)
        markers.markers.append(marker_beam)

    self.object_marker_array_pub.publish(markers)


class FusionSubscriber():
    def __init__(self):
        self.radar_sub: message_filters.Subscriber
        self.rgb_sub: message_filters.Subscriber
    
    def to_list(self):
        """
        callback will return [radar, calib_rgb]
        """
        return [self.radar_sub, self.rgb_sub]

class SensorFusion():
    def __init__(self):
        # subscriber
        self.sub_fusion = defaultdict()
        
        # publisher
        self.pub_fusion = defaultdict()
        self.pub_fusion["radar_image"] = defaultdict()
        self.pub_fusion["fusion_image"] = defaultdict()
        
        # data
        self.fusion_data = defaultdict()
        internal_topics = ["rgb", "radar", "bounding_boxes"]
        for i in internal_topics:
            self.fusion_data[i] = defaultdict()
        
        # time sync
        self.synchronizer = defaultdict()
        
        # config
        self.sensor_names = default_config.sensor_fusion
        # yolo model
        self.yolo_model = YOLO()
        # cvbridge
        self.bridge = CvBridge()
        
        # subscribe and publish
        self.subscribe_and_publish()
    
    
    def subscribe_and_publish(self):
        
        for sensor in self.sensor_names:
            # subscribe radar and calib_rgb
            self.sub_fusion[sensor.name] = FusionSubscriber()
            self.sub_fusion[sensor.name].radar_sub = message_filters.Subscriber("/radar/" + sensor.radar_name + "/decoded_messages", RadarPoints)
            self.sub_fusion[sensor.name].rgb_sub = message_filters.Subscriber("/rgb/" + sensor.rgb_name + "/calib_image", Image)
            
            # time sync
            self.synchronizer[sensor.name] = message_filters.ApproximateTimeSynchronizer(self.sub_fusion[sensor.name].to_list(), 1, 0.5)
            self.synchronizer[sensor.name].registerCallback(partial(self.callback, sensor.name))
            
            # render radar on image
            if default_config.use_radar_image:
                name = sensor.rgb_name + "/" + sensor.radar_name
                self.pub_fusion["radar_image"][name] = rospy.Publisher("/fusion/" + name + "/radar_image", Image, queue_size=1)
                self.pub_fusion["fusion_image"][name] = rospy.Publisher("/fusion/" + name + "/fusion_image", Image, queue_size=1)
        
        # objects publisher
        self.pub_object = rospy.Publisher("/objects", Objects, queue_size=1)
        self.object_marker_array_pub = rospy.Publisher("/object_marker_array",MarkerArray,queue_size=1)
        
        
    def callback(self, sensor_name, radar_points, calib_rgb):
        rospy.loginfo("=="*5 + f"{sensor_name} CALLBACK" + "=="*5)
        
        self.fusion_data["radar"][sensor_name] = radar_points
        rgb_img = self.bridge.imgmsg_to_cv2(calib_rgb)
        bbox_array = self.yolo_model.inference(rgb_images=rgb_img, rgb_names=sensor_name)
        
        objects = Objects()
        if len(bbox_array) == 0 or len(self.fusion_data["radar"][sensor_name].rps) == 0:
            return
        
    def radar_callback(self, radar_name, radar_points):
        self.fusion_data["radar"][radar_name] = radar_points
    
    def ts_callback(self, *data):
        # synchronized data
        rgb_images: List[Image] = []
        rgb_names: List[str] = []
        radar_points_array: List[RadarPoints] = []

        # yolo model
        for i in range(len(self.config)):
            rgb_images.append(self.bridge.imgmsg_to_cv2(data[i]))
            rgb_names.append(self.config[i].rgb_name)
            radar_points_array.append(self.fusion_data["radar"][self.config[i].radar_name])
        
        bounding_boxes_array = self.yolo_model.inference(rgb_images=rgb_images, rgb_names=rgb_names)

        objects = Objects()
        for i in range(len(self.config)):
            if len(radar_points_array[i].rps) == 0 and len(bounding_boxes_array[i]) == 0:
                rospy.logwarn("no objects")
                continue

            fusion_image = rgb_images[i].copy()
            radar_image = rgb_images[i].copy()
            config = self.config[i]

            radar_points = radar_points_array[i].rps
            points_3d = np.empty(shape=(0, 3))
            for p in radar_points:
                points_3d = np.append(points_3d, [[p.distX, p.distY, 0.5]], axis=0)

            # obtain projection matrix
            proj_radar_to_rgb = self.project_radar_to_rgb(rgb_name=config.rgb_name)
            # apply projection
            points_2d = self.project_to_rgb(points_3d.transpose(), proj_radar_to_rgb)

            # filter out pixels points
            inds = np.where((points_2d[0, :] < rgb_config[config.rgb_name].size[0]) & (points_2d[0, :] >= 0) &
                (points_2d[1, :] < rgb_config[config.rgb_name].size[1]) & (points_2d[1, :] >= 0) &
                (points_3d[:, 0] > 0)
                )[0]
            points_2d = points_2d[:, inds]

            # retrieve depth from radar (x)
            points_3d = points_3d[inds, :]

            # fusion
            if len(bounding_boxes_array[i]):
                self.yolo_model.draw_yolo_image(fusion_image, bounding_boxes_array[i])
                if len(points_2d) and len(points_2d[0]):
                    for box in bounding_boxes_array[i]:
                        points_in_box = self.find_inside_points(box, radar_points, points_2d)
                        if not len(points_in_box):
                            continue
                        true_points = self.filter_points(box, points_in_box)
                        radar_info = self.aggregate_radar_info(true_points)
                    
                        o = Object()
                        o.bounding_box = box
                        o.radar_info = radar_info
                        o.radar_points = radar_points
                        o.radar_name = config.radar_name
                        o.rgb_name = config.rgb_name
                        objects.objects.append(o)
                    
                        cv2.putText(fusion_image, str(int(radar_info.distX)), (int(box.x_min), int(box.y_min-30)), cv2.FONT_HERSHEY_SIMPLEX, .75, (220, 0, 50), 3)

                        for j in range(len(true_points)):
                            depth = (80 - true_points[j][0].distX) / 80
                            length = max(int(80 * depth), 4)
                            cv2.line(fusion_image, (int(true_points[j][1]), int(true_points[j][2])+length), (int(true_points[j][1]), int(true_points[j][2])), (0, int(255 * (depth)), 50), thickness=3)


            if default_config.use_radar_image:
                for p in range(points_2d.shape[1]):
                    depth = (80 - points_3d[p, 0]) / 80
                    length = max(int(80 * depth), 4)
                    cv2.line(radar_image, (int(points_2d[0, p]), int(points_2d[1, p])+length), (int(points_2d[0, p]), int(points_2d[1, p])), (0, int(255 * (depth)), 50), thickness=3)

            self.pub_fusion["radar_image"][config.rgb_name + "/" + config.radar_name].publish(self.bridge.cv2_to_imgmsg(radar_image))
            self.pub_fusion["fusion_image"][config.rgb_name + "/" + config.radar_name].publish(self.bridge.cv2_to_imgmsg(fusion_image))
        # TODO
        # filter objects between multiple devices
        self.draw_object_on_radar(objects.objects)
        self.pub_object.publish(objects)

    def _subscribe_and_publish(self):
        sub_list = []
        for i in self.config:
            # radar topic
            self.sub_fusion["radar"][i.radar_name] = rospy.Subscriber("/radar/" + i.radar_name + "/decoded_messages", RadarPoints, partial(self.radar_callback, i.radar_name), queue_size=1)
            self.fusion_data["radar"][i.radar_name] = RadarPoints()
            
            # calib rgb topic
            self.sub_fusion["rgb"][i.rgb_name] = message_filters.Subscriber("/rgb/" + i.rgb_name + "/calib_image", Image)
            sub_list.append(self.sub_fusion["rgb"][i.rgb_name])
            
            # render radar on image
            if default_config.use_radar_image:
                name = i.rgb_name + "/" + i.radar_name
                self.pub_fusion["radar_image"][name] = rospy.Publisher("/fusion/" + name + "/radar_image", Image, queue_size=1)
                self.pub_fusion["fusion_image"][name] = rospy.Publisher("/fusion/" + name + "/fusion_image", Image, queue_size=1)

        # objects publisher
        self.pub_object = rospy.Publisher("/objects", Objects, queue_size=1)
        self.object_marker_array_pub = rospy.Publisher("/object_marker_array",MarkerArray,queue_size=1)

        # time sync
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(sub_list, 1, 0.3)
        self.synchronizer.registerCallback(self.ts_callback)

    def project_radar_to_rgb(self, rgb_name): # https://github.com/darylclimb/cvml_project/blob/master/projections/lidar_camera_projection/data/000114_calib.txt # P_radar_to_rgb = np.vstack([np.array([7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -3.717806000000e-01]).reshape((3,4)), np.array([0., 0., 0., 1.])])
        # identity transformation matrix
        P_radar_to_rgb = np.array(
            [0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1]
        ).reshape((4, 4))
        P_rgb = rgb_config[rgb_name].P
        projection_matrix = np.dot(P_rgb, np.dot(np.eye(4), P_radar_to_rgb))
        return projection_matrix

    def project_to_rgb(self, radar_points, projection_matrix):
        num_pts = radar_points.shape[1]
        points = np.vstack([radar_points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        points[:2, :] /= points[2, :]
        return points[:2, :]

    def find_inside_points(self, box: Bbox, radar_points: List[RadarPoint], points_2d):
        """
        find points inside the bounding box.
        """
        lt, lb, rt, rb = box.x_min, box.x_max, box.y_min, box.y_max
        result = []
        for i in range(len(points_2d[0])):
            if points_2d[0, i] < lb and points_2d[0, i] > lt:
                result.append([radar_points[i], points_2d[0][i], points_2d[1][i]])
        return result

    def filter_points(self, box: Bbox, radar_points):
        """
        find closest point.
        get points around the point, area depends on the object type.
        """
        distances = np.zeros((len(radar_points)))
        for i in range(len(radar_points)):
            d = math.sqrt(math.pow(radar_points[i][0].distX, 2) + math.pow(radar_points[i][0].distY, 2))
            distances[i] = d
        
        closest = np.argmin(distances)
        min_dist = distances[closest]
        result = []
        for i in range(len(distances)):
            if distances[i] - min_dist < 3:
                result.append(radar_points[i])
        return result

    def aggregate_radar_info(self, radar_points) -> RadarPoint:
        """
        aggregate radar info (use average)
        radar_points: [RadarPoint, pts_2d_x, pts_2d_y]
        """
        n = len(radar_points)
        radar_info = RadarPoint()
        for i in radar_points:
            radar_info.distX += i[0].distX / n
            radar_info.distY += i[0].distY / n
            radar_info.vrelX += i[0].vrelX / n
            radar_info.vrelY += i[0].vrelY / n
            radar_info.rcs += i[0].rcs / n
            radar_info.width += i[0].width / n
            radar_info.height += i[0].height / n
        return radar_info
    

def main():
    rospy.init_node("Sensor Fusion")

    sensor_fusion = SensorFusion()
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException:
        pass
