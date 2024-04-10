#!/usr/bin/env python3
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
import yaml

import math
import cv2
import rospy
import numpy as np
import message_filters
from numpy.polynomial import polyutils as pu
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ars408_msg.msg import RadarPoints
import matplotlib.pyplot as plt
import ultralytics
yaml.add_representer(np.ndarray, lambda dumper, array: dumper.represent_list(array.tolist()))

@dataclass
class Config():
    use_time_synchronizer: bool = True


class Optimizer():
    def __init__(self):
        rospy.init_node('rsu_radar_camera_auto_optimizer')

        self.param_path = '/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/param.yaml'

        self.config = Config()
        self.model = ultralytics.YOLO('yolov8m.pt')
        self.bridge = CvBridge()
        self.track_history = defaultdict(lambda: [])
        self.image_width, self.image_height = 2048, 1536
        """
        camera matrix
        [2033,    0, 1068]
        [0   , 2056,  539]
        [0   ,    0,    1]
        """
        self.p = np.array([1068, -2033, 0, 0, 539, 0, -2056, 0, 1, 0, 0, 0]).reshape((3, 4))
        # self.p = np.array([850, -1700, 0, 0, 539, 0, -2056, 0, 1, 0, 0, 0]).reshape((3, 4))
        self.param = yaml.load(open(self.param_path, 'r'))

        self.done = True
        self.optimize_count = defaultdict(lambda: 0)
        self.sx = defaultdict(lambda: 0)
        self.sy = defaultdict(lambda: 0)

        if 'sx' in self.param:
            self.sx.update(self.param['sx'])
        if 'sy' in self.param:
            self.sy.update(self.param['sy'])

        self.setup()

        self.sx_range = (0.01, 2, 0.01)
        self.sy_range = (0.1, 4, 0.05)

        self.sub_optimizer = rospy.Subscriber('/optimizer_start', Empty, self.optimize_start, queue_size=1)

        if self.config.use_time_synchronizer:
            self.sub_image = message_filters.Subscriber('/camera/image_color', Image)
            self.sub_radar = message_filters.Subscriber('/radar/front_center/decoded_messages', RadarPoints)
            self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar], 20, 1, reset=True)
            self.sync.registerCallback(self.callback)
        else:
            # not implemented
            return

        self.pub_image = rospy.Publisher('/optimizer', Image, queue_size=10)
    
    def setup(self):
        self.sx_array = list()
        for v in self.sx.values():
            self.sx_array.append(v)

        self.sy_array = list()
        for v in self.sy.values():
            self.sy_array.append(v)

        if self.sx_array is not None or self.sy_array is not None:
            self.dist_array = list()
            for i in range(len(self.param['checkpoints'])):
                self.dist_array.append(self.param['start_dist']+self.param['interval_dist']*(i))
        
        rospy.loginfo(self.sx_array)
        rospy.loginfo(self.sy_array)
        rospy.loginfo(self.dist_array)
        
    
    def optimize_start(self, _):
        self.done = False
        self.optimize_count = defaultdict(lambda: 0)
        self.sx = defaultdict(lambda: 0)
        self.sy = defaultdict(lambda: 0)
    
    def default_to_regular(self, d, o):
        if isinstance(o, defaultdict):
            for k, v in o.items():
                d[k] = float(v)
        return d
    
    def skip_class(self, cls):
        if cls == 4:
            return True
        return False

    def project_to_image(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack((points, np.ones((1, num_pts))))
        points = projection_matrix @ points
        points[:2, :] /= points[2, :]
        return points[:2, :]
    
    def radar_process(self, radar_points):
        radar_points = radar_points.rps
        points_3d = np.empty(shape=(0, 3))
        for p in radar_points:
            points_3d = np.append(points_3d, [[p.distX, p.distY, 1.0]], axis=0)

        points_2d = self.project_to_image(points_3d.transpose(), self.p)

        inds = np.where((points_2d[0, :] < self.image_width*0.9) & (points_2d[0, :] >= self.image_width*0.1)
                & (points_2d[1, :] < self.image_height) & (points_2d[1, :] >= 0)
                & (points_3d[:, 0] > 0)
                )[0]

        points_2d = points_2d[:, inds]
        points_3d = points_3d[inds, :]
        radar_points = [radar_points[i] for i in inds]
        return points_2d, points_3d, radar_points
    
    def map_scene_point(self, point_x, point_y, sx, sy):
        n_point_x = int(point_x * sx)
        n_point_y = int(self.image_height - point_y * sy)
        return n_point_x, n_point_y
    
    def optimize(self, range, mode, target_x, target_y, point_x, point_y):
        best_s = 0
        best_error = 1e9

        for s in np.arange(*range):
            if mode == 'fix_x':
                n_point_x, n_point_y = self.map_scene_point(point_x, point_y, 0.01, s)
                e = math.sqrt(pow(target_y-n_point_y, 2))
            elif mode == 'fix_y':
                n_point_x, n_point_y = self.map_scene_point(point_x, point_y, s, 1)
                e = math.sqrt(pow(target_x-n_point_x, 2))
            if e < best_error:
                best_s = s
                best_error = e

        return best_s
    
    def show_result(self, image_msg: Image, radar_msg: RadarPoints):
        frame = self.bridge.imgmsg_to_cv2(image_msg)
        """ Preprocess radar points """
        points_2d, points_3d, radar_points = self.radar_process(radar_msg)
        for radar_id in range(points_2d.shape[1]):
            p = radar_points[radar_id]
            # dist = math.sqrt(pow(p.distX, 2)) + math.sqrt(pow(p.distY, 2))
            dist = math.sqrt(pow(p.distX, 2))
            speed = math.sqrt(pow(p.vrelX, 2)) + math.sqrt(pow(p.vrelY, 2))
            
            if self.param['speed_threshold'] and speed < self.param['speed_threshold']:
                        # and abs(p.vrelX / (p.vrelY+1e-6)) < 0.2:
                continue
        
            if dist > self.dist_array[-1]:
                continue

            # if len(self.sx_array) and len(self.sy_array):
            #     sx = pu.mapdomain(dist, self.dist_array, self.sx_array)
            #     sy = pu.mapdomain(dist, self.dist_array, self.sy_array)

            #     rospy.loginfo(f'{dist} {sy}')
            #     n_point_x, n_point_y = self.map_scene_point(int(points_2d[0, radar_id]), int(points_2d[1, radar_id]), sx, sy)
            #     cv2.circle(frame, (n_point_x, n_point_y), 20, (255, 0, 255), 10)
        
            zone = None
            for zone_id in range(len(self.param['checkpoints']) - 1):
                lower_bound = self.param['start_dist'] + self.param['interval_dist'] * zone_id
                upper_bound = self.param['start_dist'] + self.param['interval_dist'] * (zone_id + 1)
                if dist >= lower_bound and dist <= upper_bound:
                    zone = zone_id
            
            if zone == None:
                if dist < self.param['start_dist']:
                    zone = 0
                else:
                    zone = len(self.param['checkpoints'])-1

            if zone is not None:
                sx = pu.mapdomain(dist,
                    (self.param['start_dist']+self.param['interval_dist']*zone, self.param['start_dist']+self.param['interval_dist']*(zone+1)),
                    (self.sx[zone], self.sx[zone+1]))
                sy = pu.mapdomain(dist,
                    (self.param['start_dist']+self.param['interval_dist']*zone, self.param['start_dist']+self.param['interval_dist']*(zone+1)),
                    (self.sy[zone], self.sy[zone+1]))
                # rospy.loginfo('-'*10)
                # rospy.loginfo(dist)
                # rospy.loginfo((self.param['start_dist']+self.param['interval_dist']*zone, self.param['start_dist']+self.param['interval_dist']*zone+1))
                # rospy.loginfo((self.sx[zone], self.sx[zone+1]))
                # rospy.loginfo(sx)
                # rospy.loginfo((self.sy[zone], self.sy[zone+1]))
                # rospy.loginfo(sy)
                # rospy.loginfo('-'*10)
                n_point_x, n_point_y = self.map_scene_point(int(points_2d[0, radar_id]), int(points_2d[1, radar_id]), sx, sy)
                cv2.circle(frame, (n_point_x, n_point_y), 20, (255, 0, 255), 10)

        output_image_msg = self.bridge.cv2_to_imgmsg(frame)
        self.pub_image.publish(output_image_msg)
    
    def callback(self, image_msg: Image, radar_msg: RadarPoints):
        if self.done:
            self.show_result(image_msg, radar_msg)
            return

        """ Preprocess radar points """
        points_2d, points_3d, radar_points = self.radar_process(radar_msg)

        """ Detects objects in frame """
        frame = self.bridge.imgmsg_to_cv2(image_msg)
        results = self.model.track(frame, persist=True, verbose=False)

        # Get the boxes and track IDs
        classes = results[0].boxes.cls.cpu()
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist()

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Plot the tracks
        for box, cls, track_id in zip(boxes, classes, track_ids):
            if self.skip_class(cls):
                continue
            x, y, w, h = box
            track = self.track_history[track_id]
            track.append((float(x), float(y)))  # x, y center point
            if len(track) > 30:  # retain 90 tracks for 90 frames
                track.pop(0)

            # Draw the tracking lines
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
        
        """ Split boxes into zones """
        boxes_in_zones = defaultdict(lambda: [])
        for zone_id, (lower_bound, upper_bound) in enumerate(zip(self.param['checkpoints'][:-1], self.param['checkpoints'][1:])):
            for box_id, (box, cls, track_id) in enumerate(zip(boxes, classes, track_ids)):
                if self.skip_class(cls):
                    continue
                x, y, w, h = box
                rospy.loginfo(f'{box_id} {box} {cls} {track_id}')
                center_x, center_y = int(x + int(w / 2)), int(y + int(h / 2))
                if self.param['roi_region']:
                    x0 = self.param['roi_region'][0][0]
                    x1 = self.param['roi_region'][1][0]
                    if center_x <= x0 or center_x >= x1:
                        rospy.loginfo('skip box')
                        continue

                if self.param['speed_threshold'] > 0:
                    if center_y <= lower_bound[1] and center_y >= upper_bound[1]:
                        boxes_in_zones[zone_id].append(box_id)
                else:
                    if center_y >= lower_bound[1] and center_y <= upper_bound[1]:
                        boxes_in_zones[zone_id].append(box_id)

        """ Plot zones """
        for anchor in self.param['checkpoints']:
            if self.param['roi_region']:
                x0 = self.param['roi_region'][0][0]
                x1 = self.param['roi_region'][1][0]
                cv2.line(annotated_frame, (x0, anchor[1]), (x1, anchor[1]), (0, 255, 0), 3)
            else:
                cv2.line(annotated_frame, (0, anchor[1]), (self.image_width-1, anchor[1]), (0, 255, 0), 3)
        
        # if len(boxes_in_zones) > 0:
        #     rospy.loginfo(boxes_in_zones)

        """ Split radar points into zones """
        radar_points_in_zones = defaultdict(lambda: [])
        for zone_id in range(len(self.param['checkpoints']) - 1):
            lower_bound = self.param['start_dist'] + self.param['interval_dist'] * zone_id
            upper_bound = self.param['start_dist'] + self.param['interval_dist'] * (zone_id + 1)
            for radar_id in range(len(radar_points)):
                p = radar_points[radar_id]
                # dist = math.sqrt(pow(p.distX, 2)) + math.sqrt(pow(p.distY, 2))
                dist = math.sqrt(pow(p.distX, 2))
                speed = math.sqrt(pow(p.vrelX, 2)) + math.sqrt(pow(p.vrelY, 2))
                if dist >= lower_bound and dist <= upper_bound:
                    if self.param['speed_threshold'] and speed < self.param['speed_threshold'] \
                        and abs(p.vrelX / (p.vrelY+1e-6)) < 0.2:
                        continue
                    radar_points_in_zones[zone_id].append(radar_id)
        
        # if len(radar_points_in_zones) > 0:
        #     rospy.loginfo(radar_points_in_zones)

        """ Optimize each zone if matched """
        done_count = 0
        for zone_id in range(len(self.param['checkpoints']) - 1):
            if self.optimize_count[zone_id] > self.param['optimizer_step']:
                done_count = done_count + 1
                continue

            if len(boxes_in_zones[zone_id]) == 1 and len(radar_points_in_zones[zone_id]) == 1:
                box_id = boxes_in_zones[zone_id][0]
                radar_id = radar_points_in_zones[zone_id][0]
                box = boxes[box_id]
                radar_point = radar_points[radar_id]
                dist = math.sqrt(pow(p.distX, 2)) + math.sqrt(pow(p.distY, 2))
                
                """ Optimize """
                rospy.loginfo((box, radar_point))
                target_x, target_y = box[0] + int(box[2] / 2), box[1] + int(box[3] / 2)
                point_x, point_y = int(points_2d[0, radar_id]), int(points_2d[1, radar_id])
                sx_step = self.optimize(self.sx_range, 'fix_y', target_x, target_y, point_x, point_y)
                sy_step = self.optimize(self.sy_range, 'fix_x', target_x, target_y, point_x, point_y)
                self.sx[zone_id] += sx_step
                self.sy[zone_id] += sy_step
                self.optimize_count[zone_id] += 1
        
        if done_count == (len(self.param['checkpoints']) - 1):
            for zone_id in range(len(self.param['checkpoints']) - 1):
                n = self.optimize_count[zone_id]
                self.sx[zone_id] /= n
                self.sy[zone_id] /= n
            
            self.sx[len(self.param['checkpoints'])-1] = self.sx[len(self.param['checkpoints'])-2]
            self.sy[len(self.param['checkpoints'])-1] = self.sy[len(self.param['checkpoints'])-2]+(self.sy[len(self.param['checkpoints'])-2]-self.sy[len(self.param['checkpoints'])-3])

            self.param['sx'] = dict()
            self.param['sy'] = dict()
            self.param['sx'] = self.default_to_regular(self.param['sx'], self.sx)
            self.param['sy'] = self.default_to_regular(self.param['sy'], self.sy)
            
            self.setup()

            """ Save param.yaml """
            yaml.dump(deepcopy(self.param), open(self.param_path, 'w'), default_flow_style=False)
            
            self.done = True

        output_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame)        
        self.pub_image.publish(output_image_msg)

if __name__ == "__main__":
    try:
        _ = Optimizer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr_once(e)

