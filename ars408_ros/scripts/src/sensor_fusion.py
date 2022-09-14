#! /usr/bin/env python3
# coding=utf-8
import math
from functools import partial
from collections import defaultdict
from typing import List, final

import rospy
import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA

from config.config import default_config, rgb_config, radar_config
from ars408_msg.msg import RadarPoints, Objects, Object, Bbox, RadarPoint
from scripts.src.yolo_torch import YOLO


class Publisher():
    def __init__(self, name):
        self.radar_image = rospy.Publisher(f"/fusion/{name}/radar_image", Image, queue_size=1)
        self.yolo_image = rospy.Publisher(f"/fusion/{name}/yolo_image", Image, queue_size=1)
        self.fusion_image = rospy.Publisher(f"/fusion/{name}/fusion_image", Image, queue_size=1)

class SensorFusion():
    def __init__(self):
        self.config = default_config.sensor_fusion
        self.bridge = CvBridge()
        self.yolo_model = YOLO()
        # TODO
        # add night vision model

        self.setup_synchronizer()

        # objects publisher
        self.pub_fusion: dict[str, Publisher] = dict()
        for i in self.config:
            self.pub_fusion[i.name] = Publisher(i.name)
        self.pub_object = rospy.Publisher("/objects", Objects, queue_size=1)
        self.object_marker_array_pub = rospy.Publisher("/objects_marker_array", MarkerArray, queue_size=1)

    def setup_synchronizer(self):
        """
        setup synchronizer for all rgb and radar pair.
        """
        sub = list()
        for i in self.config:
            name = i.name
            # add rgb subscriber
            sub.append(message_filters.Subscriber(f"/rgb/{name}/synced_image", Image))
            # add radar subscriber
            sub.append(message_filters.Subscriber(f"/radar/{name}/synced_messages", RadarPoints))

        # synchronizer
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(sub, queue_size=5, slop=0.3)
        self.synchronizer.registerCallback(self.fusion_callback)

    def fusion_callback(self, *msgs):
        """
        fusion rgb and radar messages and publish `ars408_msg/objects`.
        """
        rgb_image_array: list[cv2.Mat] = [] # array of `cv2.Mat` for yolo inference
        rgb_name_array: list[str] = []
        radar_points_array: list[RadarPoints] = [] # array of `ars408_msg/RadarPoints`
        objects_array: dict[str, Objects] = dict()

        # preprocess msgs
        for i in range(len(self.config)):
            rgb_name_array.append(self.config[i].name)
            rgb_image_array.append(self.bridge.imgmsg_to_cv2(msgs[i * 2]))
            radar_points_array.append(msgs[i * 2 + 1])
            objects_array[self.config[i].name] = Objects()
        
        bounding_boxes_array = self.yolo_model.inference(rgb_images=rgb_image_array, rgb_names=rgb_name_array)

        for i in range(len(self.config)):
            if len(radar_points_array[i].rps) == 0 and len(bounding_boxes_array[i]) == 0:
                rospy.logwarn("no objects")
                continue

            yolo_image = rgb_image_array[i].copy()
            radar_image = rgb_image_array[i].copy()
            config = self.config[i]

            # create 3d numpy array for faster matrix manipulation
            radar_points = radar_points_array[i].rps
            points_3d = np.empty(shape=(0, 3))
            for p in radar_points:
                points_3d = np.append(points_3d, [[p.distX, p.distY, 0.5]], axis=0)

            # obtain projection matrix
            proj_radar_to_rgb = self.project_radar_to_rgb(rgb_name=config.rgb_name)

            # apply projection
            points_2d = self.project_to_rgb(points_3d.transpose(), proj_radar_to_rgb)

            # filter out pixels points
            image_width = rgb_config[config.rgb_name].size[0]
            image_height = rgb_config[config.rgb_name].size[1]
            inds = np.where((points_2d[0, :] < image_width) & (points_2d[0, :] >= 0) &
                (points_2d[1, :] < image_height) & (points_2d[1, :] >= 0) &
                (points_3d[:, 0] > 0)
                )[0]
            points_2d = points_2d[:, inds]
            points_3d = points_3d[inds, :]
            radar_points = [radar_points[i] for i in inds]

            # fusion
            if len(bounding_boxes_array[i]):
                self.yolo_model.draw_yolo_image(yolo_image, bounding_boxes_array[i])
                fusion_image = yolo_image.copy()
                if len(points_2d) and len(points_2d[0]):
                    for box in bounding_boxes_array[i]:
                        points_in_box = self.find_inside_points(box, radar_points, points_2d)
                        if len(points_in_box) == 0:
                            continue
                        true_points = self.filter_points(box, points_in_box)
                        if len(true_points) == 0:
                            continue
                        radar_info = self.aggregate_radar_info(true_points)
                        # TODO
                        # remove used radar point (map box and radar point 1 to 1)

                        # FIXME
                        # move this into config (and make it flexible)
                        radar_info.width = 4
                        radar_info.height = 2
                        if box.objClass in ["motor-peole", "bike-people", "motor", "bike"]:
                            radar_info.width = 2
                            radar_info.height = 2
                        
                        o = Object()
                        o.bounding_box = box
                        o.radar_info = radar_info
                        o.radar_points = [i[0] for i in true_points]
                        o.radar_name = config.radar_name
                        o.rgb_name = config.rgb_name
                        objects_array[config.name].objects.append(o)
                    
                        cv2.putText(fusion_image, str(int(radar_info.distX)), (int(box.x_min), int(box.y_min-30)), cv2.FONT_HERSHEY_SIMPLEX, .75, (220, 0, 50), 3)

                        for j in range(len(true_points)):
                            depth = (80 - true_points[j][0].distX) / 80
                            length = max(int(80 * depth), 4)
                            cv2.line(fusion_image, (int(true_points[j][1]), int(true_points[j][2])+length), (int(true_points[j][1]), int(true_points[j][2])), (0, int(255 * (depth)), 50), thickness=3)

                self.pub_fusion[config.name].fusion_image.publish(self.bridge.cv2_to_imgmsg(fusion_image))

            if default_config.use_radar_image:
                for p in range(points_2d.shape[1]):
                    depth = (80 - points_3d[p, 0]) / 80
                    length = max(int(80 * depth), 4)
                    cv2.line(radar_image, (int(points_2d[0, p]), int(points_2d[1, p])+length), (int(points_2d[0, p]), int(points_2d[1, p])), (0, int(255 * (depth)), 50), thickness=3)
                self.pub_fusion[config.name].radar_image.publish(self.bridge.cv2_to_imgmsg(radar_image))

            if default_config.use_yolo_image:
                self.pub_fusion[config.name].yolo_image.publish(self.bridge.cv2_to_imgmsg(yolo_image))

        # TODO
        # filter objects between multiple devices
        final_objects = Objects()
        for objects in objects_array.values():
            for object in objects.objects:
                final_objects.objects.append(object)
            
        self.pub_object.publish(final_objects)
        
        marker_array = self.draw_object_on_radar(final_objects.objects)
        self.object_marker_array_pub.publish(marker_array)

    def project_radar_to_rgb(self, rgb_name) -> np.ndarray:
        # identity transformation matrix
        P_radar_to_rgb = np.array(
            [0, -1, 0, -1,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1]
        ).reshape((4, 4))
        # https://github.com/darylclimb/cvml_project/blob/master/projections/lidar_camera_projection/data/000114_calib.txt
        # P_radar_to_rgb = np.vstack([np.array([7.533745000000e-03, -9.999714000000e-01, -6.166020000000e-04, -4.069766000000e-03, 1.480249000000e-02, 7.280733000000e-04, -9.998902000000e-01, -7.631618000000e-02, 9.998621000000e-01, 7.523790000000e-03, 1.480755000000e-02, -3.717806000000e-01]).reshape((3,4)), np.array([0., 0., 0., 1.])])
        P_rgb = rgb_config[rgb_name].P
        projection_matrix = np.dot(P_rgb, np.dot(np.eye(4), P_radar_to_rgb))
        return projection_matrix
    
    def project_rgb_to_radar(self, rgb_name) -> np.ndarray:
        P_radar_to_rgb = np.array(
            [0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, 0,
            0, 0, 0, 1]
        ).reshape((4, 4))
        
        projection_matrix = np.dot(np.eye(4), np.linalg.inv(P_radar_to_rgb))
        return projection_matrix

    def project_to_rgb(self, radar_points, projection_matrix):
        num_pts = radar_points.shape[1]
        points = np.vstack([radar_points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        points[:2, :] /= points[2, :]
        return points[:2, :]
    
    def project_to_radar(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack([points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        return points[:3, :]

    def draw_object_on_radar(self, objects: Objects):
        ref_id = 0

        def rotate(_x,_y, angle):
            # angle = angle * -1
            x = math.cos(angle) * _x - math.sin(angle) * _y   
            y = math.sin(angle) * _x + math.cos(angle) * _y 
            return (x,y)

        # Return order : LU RU RD BD (L/R:Left/Right, U/D:Up/Down)
        def calculate_point(distX,distY,transRadius,width,length):
            LU = Point(x=rotate(distX-length/2,distY-width/2,transRadius)[0],
                        y=rotate(distX-length/2,distY-width/2,transRadius)[1])
            RU = Point(x=rotate(distX-length/2,distY+width/2,transRadius)[0],
                        y=rotate(distX-length/2,distY+width/2,transRadius)[1])
            RD = Point(x=rotate(distX+length/2,distY+width/2,transRadius)[0],
                        y=rotate(distX+length/2,distY+width/2,transRadius)[1])
            LD = Point(x=rotate(distX+length/2,distY-width/2,transRadius)[0],
                        y=rotate(distX+length/2,distY-width/2,transRadius)[1])
            return [LU,RU,RD,LD]


        def draw_surface(LU:Point, RU:Point, RD:Point, LD:Point, transX, transY, height):

            LU.z = height
            RU.z = height
            RD.z = height
            LD.z = height

            # Rotate points
            points = [LU,RU,RD,LD]

            # Calib coordinate
            for points_counter in range(len(points)):
                points[points_counter].x = points[points_counter].x + transX
                points[points_counter].y = points[points_counter].y + transY * -1

            return points

        def draw_beam(LU:Point, RU:Point, RD:Point, LD:Point, transX, transY, height):

            # T/B:Top/Bottom
            TLU = Point(x=LU.x,y=LU.y,z=height)
            BLU = Point(x=LU.x,y=LU.y,z=0)
            TRU = Point(x=RU.x,y=RU.y,z=height)
            BRU = Point(x=RU.x,y=RU.y,z=0)
            TRD = Point(x=RD.x,y=RD.y,z=height)
            BRD = Point(x=RD.x,y=RD.y,z=0)
            TLD = Point(x=LD.x,y=LD.y,z=height)
            BLD = Point(x=LD.x,y=LD.y,z=0)
            # Rotate points
            points = [
               TLU,BLU,
               TRU,BRU,
               TRD,BRD,
               TLD,BLD ]
            
            # Calib coordinate
            for points_counter in range(len(points)):
                points[points_counter].x = points[points_counter].x + transX
                points[points_counter].y = points[points_counter].y + transY * -1

            return points

        # markers = MarkerArray(header=Header(frame_id = "base_link", stamp = rospy.Time.now()),)
        markers = MarkerArray()
        
        markers.markers.append(Marker(header=Header(frame_id="base_link"), action=Marker.DELETEALL))
        self.object_marker_array_pub.publish(markers)
        markers.markers.clear()
        
        for o in objects:
            # TODO
            # move this into config (and make it flexible)
            c = ColorRGBA(r=1.0,g=0,b=0,a=1.0) if o.bounding_box.objClass in ["motor", "motor-people"] else ColorRGBA(r=0,g=1,b=1,a=1)

            [LU,RU,RD,LD] = calculate_point(o.radar_info.distX,o.radar_info.distY,radar_config[o.radar_name].transform[2],
                                    o.radar_info.width,default_config.class_depth[o.bounding_box.objClass])
            marker_bottom = Marker(
                header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
                id = ref_id,
                ns=o.rgb_name,
                type = Marker.LINE_STRIP,
                action = Marker.ADD,
                pose = Pose(
                    position = Point(x=0,y=0,z=0.1),
                    orientation = Quaternion(x=0,y=0,z=0,w=1.0)
                ),
                points = draw_surface(LU,RU,RD,LD,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],0),
                color = c,
                scale = Vector3(x=0.1,y=0.5,z=0.1),
            )

            ref_id = ref_id + 1

            marker_top = Marker(
                header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
                id = ref_id,
                ns=o.rgb_name,
                type = Marker.LINE_STRIP,
                action = Marker.ADD,
                pose = Pose(
                    position = Point(x=0,y=0,z=0.1),
                    orientation = Quaternion(x=0,y=0,z=0,w=1.0)
                ),
                points = draw_surface(LU,RU,RD,LD,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],o.radar_info.height),
                color = c,
                scale = Vector3(x=0.1,y=0.5,z=0.1),
            )

            ref_id = ref_id + 1

            marker_beam = Marker(
                header = Header(frame_id = "base_link", stamp = rospy.Time.now()),
                id = ref_id,
                ns=o.rgb_name,
                type = Marker.LINE_LIST,
                action = Marker.ADD,
                pose = Pose(
                    position = Point(x=0,y=0,z=0.1),
                    orientation = Quaternion(x=0,y=0,z=0,w=1.0)
                ),
                points = draw_beam(LU,RU,RD,LD,radar_config[o.radar_name].transform[1],radar_config[o.radar_name].transform[0],o.radar_info.height),
                color = c,
                scale = Vector3(x=0.1,y=0.5,z=0.1),
            )

            ref_id = ref_id + 1
            markers.markers.append(marker_bottom)
            markers.markers.append(marker_top)
            markers.markers.append(marker_beam)
        
        return markers

    def find_inside_points(self, box: Bbox, radar_points: List[RadarPoint], points_2d):
        """
        find points inside the bounding box.
        """
        # lt, lb, rt, rb = box.x_min, box.x_max, box.y_min, box.y_max
        result = []
        for i in range(len(points_2d[0])):
            if points_2d[0, i] < box.x_max and points_2d[0, i] > box.x_min:
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
        if min_dist > 60:
            return result

        for i in range(len(distances)):
            if (distances[i] - min_dist) < default_config.class_depth[box.objClass]:
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
            # radar_info.width += i[0].width / n
            # radar_info.height += i[0].height / n
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