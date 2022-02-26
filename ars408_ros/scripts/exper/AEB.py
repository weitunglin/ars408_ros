#! /usr/bin/env python3
# coding=utf-8
import os
import math
from pickle import FALSE
from tokenize import Double

import rospy
import numpy as np
import yaml

from ars408_msg.msg import RadarPoint, RadarPoints
from std_msgs.msg import Float32, String

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int8MultiArray, Header, ColorRGBA, Int8
from ars408_msg.msg import pathPoints
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point

# load config
with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)


RVIZ_TEXT = False
RVIZ_ARROW = False

RVIZ_RANGE = True
RADAR_PREDICT = True
COLLISION_RANGE = True
RVIZ_RADARPOINTS_TRAJECTORY = True


class AEB():
    def __init__(self, radar_channel):
        self.radar_channel = radar_channel
        self.radar_period = rospy.Time.now()

        self.Q_radar = np.zeros((4, 4))
        self.H_radar = np.zeros((4, 4))
        self.R_radar = np.zeros((4, 4))
        self.B_radar = np.zeros((4, 4))
        self.U_radar = np.zeros((4, 4))

        self.now_speed = 0.0
        self.now_zaxis = 0.0
        self.car_width = 1.5
        self.car_length = 5.0
        self.radar_width = 0.5
        self.RCS_filter = -10000.0
        self.radar_abs_speed = np.zeros(100)

        self.disappear_time = {}
        self.radar_trajectory = {}

        self.path_points: pathPoints = None
        self.collision_path: pathPoints = None
        self.radar_points: RadarPoints = None

        self.sub_radar_pub_rt = rospy.Subscriber(
            self.radar_channel + "/radarPubRT", RadarPoints, self.callbackRadarPubRT, queue_size=1)
        self.sub_path_points = rospy.Subscriber(
            self.radar_channel + "/" + config["topic_PathPoints"], pathPoints, self.callbackPathPoints, queue_size=1)

        self.pub_collision = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Collision"], Int8MultiArray, queue_size=1)
        self.pub_aeb = rospy.Publisher(
            self.radar_channel + "/" + config["topic_AEB"], Int8MultiArray, queue_size=1)

    def getHeader(self) -> Header:
        return Header(frame_id=self.radar_channel, stamp=rospy.Time.now())


    def kfPredict(self, X, P, F, Q):
        X = F * X
        P = F * P * F.T + Q


    def kfUpdate(self, X, P, Y, H, R):
        V = Y - H * X
        S = H * P * H.T + R
        K = P * H.T * S.I
        X = X + K * V
        P = P - K * H * P


    # [FIXME] fix rotate point
    def rotatePoint(self, angle, x, y, center_x, center_y) -> np.matrix:
        rotate = float(angle * math.pi / 180)

        x -= center_x
        y -= center_y

        rotate_x = float(math.cos(rotate) * x -
                         math.sin(rotate) * y + center_x)
        rotate_y = float(math.sin(rotate) * x +
                         math.cos(rotate) * y + center_y)

        if abs(rotate_x) < 0.001:
            rotate_x = 0
        else:
            rotate_x = rotate_x

        if abs(rotate_y) < 0.001:
            rotate_y = 0
        else:
            rotate_y = rotate_y

        return np.matrix(rotate_x, rotate_y)


    # [FIXME] fix intersect
    def intersect(self, a,  b) -> bool:
        for i in len(a):
            cur = np.matrix(a[i])
            next = np.matrix(a[(i + 1) % a.size()])
            edge = np.matrix(next - cur)

            axis = np.matrix(-edge(1), edge(0))

            a_max = -np.Infinity
            a_min = np.Infinity
            b_max = -np.Infinity
            b_min = np.Infinity

            for v in a:
                proj = float(axis.dot(v))
                if(proj < a_min):
                    a_min = proj
                if(proj > a_max):
                    a_max = proj

            for v in b:
                proj = float(axis.dot(v))
                if(proj < b_min):
                    b_min = proj
                if(proj > b_max):
                    b_max = proj

            if (a_min < b_max and a_max > b_min) or (b_min < a_max and b_min > a_min):
                continue
            else:
                return False

        return True

    def minMax(value1, value2, flag) -> float:
        if(flag == 0):
            return value2 if value1 > value2 else value1
        else:
            return value1 if value1 > value2 else value2


    def callbackPathPoints(self, data: pathPoints):
        self.path_points = data


    def callbackRadarPubRT(self, data: RadarPoints):
        self.radar_points = data
    

    def loop(self):
        # do AEB
        time_diff = (rospy.Time.now() - self.radar_period).to_sec()
        self.radar_period = rospy.Time.now()

        kalman_markers = MarkerArray()
        colli_arr = Int8MultiArray()
        aeb_arr = Int8MultiArray()

        last_id = -1

        for it in self.radar_points.rps:

            pov_speed = math.sqrt(pow(it.vrelX, 2.0) + pow(it.vrelY, 2.0))

            # [TODO] AEB radarTraj
            self.disappear_time[it.id] = 0
            if last_id + 1 != it.id:
                for i in range(last_id + 1, it.id):
                    self.disappear_time[i] += 1

                    if self.disappear_time[i] >= 3:
                        self.radar_trajectory[i].pathPoints.clear()

            last_id = it.id

            # ? not necessary
            id_name = it.id

            if RADAR_PREDICT:
                if len(self.path_points) >= 2:
                    init_x = self.path_points[0].X
                    init_y = self.path_points[0].Y
                    init_vx = (self.path_points[1].X - self.path_points[0].X) / time_diff
                    init_vy = (self.path_points[1].Y - self.path_points[0].Y) / time_diff

                    X_radar = np.matrix([[init_x, init_y, init_vx, init_vy]])

                    F_radar = np.matrix([[1, 0, time_diff, 0],
                                         [0, 1, 0, time_diff],
                                         [0, 0, 1, 0],
                                         [0, 0, 0, 1]])

                    P_radar = np.matrix([[0.1, 0, 0, 0], 
                                         [0, 0.1, 0, 0],
                                         [0, 0, 0.1, 0],
                                         [0, 0, 0, 0.1]])

                    cur_radar_speed = None

                    for t in range(1, len(self.path_points)):
                        velocity_x = float(self.path_points[t].X - self.path_points[t - 1].X)/time_diff
                        velocity_y = float(self.path_points[t].Y - self.path_points[t - 1].Y)/time_diff

                        if t == len((self.path_points)) - 1:
                            F_radar = np.matrix([[1, 0, 4, 0], 
                                                 [0, 1, 0, 4], 
                                                 [0, 0, 1, 0], 
                                                 [0, 0, 0, 1]])
                            cur_radar_speed = pow(pow(velocity_x, 2) + pow(velocity_y, 2), 0.5)
                    
                        Y_radar = np.matrix([[1, 0, 4, 0], 
                                             [0, 1, 0, 4], 
                                             [0, 0, 1, 0], 
                                             [0, 0, 0, 1]])
                         # kalman filter
                        self.kfPredict(X_radar, P_radar, F_radar, self.B_radar, self.U_radar, self.Q_radar)
                        self.kfUpdate(X_radar, P_radar, Y_radar, self.H_radar, self.R_radar)

                
                    kalman_marker = Marker(header=self.getHeader(), ns="kalman_markers", id=it.id, type=Marker.LINE_STRIP, action=Marker.ADD, lifetime=rospy.rostime.Duration(0.1),
                                               pose=Pose(orientation=Quaternion(w=1.0)), scale=Vector3(x=0.5, z=0.5), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

                    if COLLISION_RANGE:
                        a_points = np.array([])

                        radar_m = float(X_radar[0, 0] - it.distX) / (X_radar[1, 0] - it.distY)
                        radar_angle = float(math.atan(radar_m) / (math.pi / 180))
                        radar_rotate_angle = float(90 - abs(radar_angle))    

                        if radar_angle < 0:
                            radar_rotate_angle = -radar_rotate_angle

                        if pov_speed < 0:
                            np.append(a_points, (self.rotatePoint(
                                radar_rotate_angle, it.distX, it.distY + self.radar_width / 2, it.distX, it.distY)))
                            np.append(a_points, (self.rotatePoint(
                                radar_rotate_angle, it.distX, it.distY - self.adar_width / 2, it.distX, it.distY)))
                            np.append(a_points, (self.rotatePoint(radar_rotate_angle, X_radar(
                                0, 0), X_radar(1, 0) - self.radar_width / 2, X_radar(0, 0), X_radar(1, 0))))
                            np.append(a_points, (self.rotatePoint(radar_rotate_angle, X_radar(
                                0, 0), X_radar(1, 0) + self.radar_width / 2, X_radar(0, 0), X_radar(1, 0))))

                            path_index = 0
                            for pred in self.collision_path.pathPoints:
                                path_index += 1
                                if path_index % 4 == 1:
                                    continue
                                b_points = np.array(np.matrix())

                                np.append(b_points, (self.rotatePoint(
                                    self.now_zaxis/100*path_index, pred.X, pred.Y + (self.car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotatePoint(
                                    self.now_zaxis/100*path_index, pred.X, pred.Y - (self.car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotatePoint(
                                    self.now_zaxis/100*path_index, pred.X - self.car_length, pred.Y - (self.car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotatePoint(
                                    self.now_zaxis/100*path_index, pred.X - self.car_length, pred.Y + (self.car_width / 2), pred.X, pred.Y)))

                                if self.intersect(a_points, b_points) and self.intersect(b_points, a_points):
                                    ttr_ego = float(4 * path_index)/100
                                    ttr_target = float(
                                        pow(pow(pred.X - it.distX, 2) + pow(pred.Y - it.distY, 2), 0.5) / abs(pov_speed))
                                    ttc_threshold = float(
                                        abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5) + 1.5)

                                    if(abs(ttr_target - ttr_ego) < 1.5 and self.minMax(ttr_target, ttr_ego, 0) < ttc_threshold):
                                        colli = Int8
                                        colli.data = id_name
                                        np.append(
                                            colli_arr.data.push_back(it.id))

                                    ttc_threshold = (
                                        abs(pov_speed) / (2 * 9.8 * 0.3)) + 1
                                    if(abs(ttr_target - ttr_ego) < 1 and self.minMax(ttr_target, ttr_ego, 0) < ttc_threshold):
                                        aeb = Int8
                                        aeb.data = it.id
                                        aeb_arr.data.push_back(it.id)
                                        print(
                                            "AEB->\nRadar ID : %d  Radar Speed : %f  Vehicle Speed : %f\n", it.id, pov_speed, self.now_speed)

                                    break
                    p = Point
                    p.z = 1
                    p.x = it.distX
                    p.y = it.distY
                    np.append(kalman_marker.points, p)
                    p.x = X_radar(0, 0)
                    p.y = X_radar(1, 0)
                    np.append(kalman_marker.points, p)

                    if(abs(X_radar(0, 0) - it.distX) < 100 and abs(X_radar(1, 0) - it.distY) < 100):
                        np.append(kalman_markers.markers, kalman_marker)   

                self.pub_collision.publish(colli_arr)
                self.pub_aeb.publish(aeb_arr)         
        
        # [TODO] AEB radarTraj
        for i in range(last_id + 1, 100):
            self.disappear_time[i] += 1
            # std::cout << "(" << i << ":" << disappear_time[i] << ") ";
            if(self.disappear_time[i] >= 3):
                self.path_points[i].pathPoints.clear()
        pass


def listener(radar_channel):
    rospy.init_node("AEB")
    rosrate = rospy.Rate(config["frameRate"])

    aeb = AEB(radar_channel)

    while not rospy.is_shutdown():
        # do AEB
        aeb.loop()
        rosrate.sleep()


if __name__ == "__main__":
    try:
        radar_channel = rospy.get_param('AEB/radarChannel')

        if radar_channel != None:
            pass
            # listener(radar_channel)
    except rospy.ROSInternalException:
        pass


        # # [FIXME] RVIZ RANGE
        # # if RVIZ_RANGE:
        #     #
        #     # RVIZ RADAR RANGE
        #     #
        #     # rviz_range()

        # #
        # # [1] Show Radar
        # # [2] Kalman predict Radar Path
        # # [3] AEB
        # #
        # collision_markers = MarkerArray()
        # marker_array = MarkerArray()
        # radar_trajectory_markers = MarkerArray()
        # kalman_markers = MarkerArray()
        # collison_arr = Int8MultiArray()

        # marker = Marker(header=self.getHeader())

        # # Clear all the markers
        # marker.action = Marker.DELETEALL
        # marker_array.markers.append(marker)
        # self.marker_array_pub.publish(marker_array)

        # path_points = pathPoints()

        # last_id = -1
        # aeb_array = Int8MultiArray()

        # for it in data.rps:
        #     # Radar Speed calculation
        #     pov_speed = math.sqrt(pow(it.vrelX, 2.0) + pow(it.vrelY, 2.0))

        #     if it.vrelX < 0:
        #         pov_speed = -pov_speed

        #     acc = pov_speed - self.radar_abs_speed[it.id]

        #     self.radar_abs_speed[it.id] = pov_speed

        #     # [TODO] AEB radarTraj
        #     self.disappear_time[it.id] = 0
        #     if last_id + 1 != it.id:
        #         for i in range(last_id + 1, it.id):
        #             self.disappear_time[i] += 1

        #             if self.disappear_time[i] >= 3:
        #                 self.radar_trajectory[i].pathPoints.clear()

        #     last_id = it.id

        #     # [FIXME] is this necessary ?
        #     id_name = it.id

        #     # [TODO]: AEB
        #     if RADAR_PREDICT:

        #         if(len(self.radar_trajectory[id_name].pathPoints) >= 2):

        #             init_x = self.radar_trajectory[id_name].pathPoints[0].X
        #             init_y = self.radar_trajectory[id_name].pathPoints[0].Y
        #             init_vx = (self.radar_trajectory[id_name].pathPoints[1].X -
        #                        self.radar_trajectory[id_name].pathPoints[0].X) / time_diff
        #             init_vy = (self.radar_trajectory[id_name].pathPoints[1].Y -
        #                        self.radar_trajectory[id_name].pathPoints[0].Y) / time_diff

        #             X_radar = np.matrix([init_x, init_y, init_vx, init_vy])

        #             F_radar = np.matrix(
        #                 [[1, 0, time_diff, 0], [0, 1, 0, time_diff], [0, 0, 1, 0], [0, 0, 0, 1]])

        #             P_radar = np.matrix([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [
        #                                 0, 0, 0.1, 0], [0, 0, 0, 0.1]])

        #             cur_radar_speed = None

        #             for t in range(1, len(self.radar_trajectory[id_name].pathPoints)):
        #                 velocity_x = float(
        #                     self.radar_trajectory[id_name].pathPoints[t].X - self.radar_trajectory[id_name].pathPoints[t - 1].X)/time_diff
        #                 velocity_y = float(
        #                     self.radar_trajectory[id_name].pathPoints[t].Y - self.radar_trajectory[id_name].pathPoints[t - 1].Y)/time_diff

        #                 if t == len((self.radar_trajectory[id_name].pathPoints)) - 1:
        #                     F_radar = np.matrix(
        #                         [[1, 0, 4, 0], [0, 1, 0, 4], [0, 0, 1, 0], [0, 0, 0, 1]])

        #                     cur_radar_speed = pow(
        #                         pow(velocity_x, 2) + pow(velocity_y, 2), 0.5)

        #                 Y_radar = np.matrix(
        #                     [[1, 0, 4, 0], [0, 1, 0, 4], [0, 0, 1, 0], [0, 0, 0, 1]])

        #                 # kalman filter
        #                 self.kf_predict(X_radar, P_radar, F_radar,
        #                                 self.B_radar, self.U_radar, self.Q_radar)
        #                 self.kf_update(X_radar, P_radar,
        #                                Y_radar, self.H_radar, self.R_radar)

        #             kalman_marker = Marker(header=self.getHeader(), ns="kalman_markers", id=it.id, type=Marker.LINE_STRIP, action=Marker.ADD, lifetime=rospy.rostime.Duration(0.1),
        #                                    pose=Pose(orientation=Quaternion(w=1.0)), scale=Vector3(x=0.5, z=0.5), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

        #             if COLLISION_RANGE:
        #                 a_points = np.array(np.matrix())

        #                 radar_m = float(X_radar(0, 0) - it.distX) / \
        #                     (X_radar(1, 0) - it.distY)
        #                 radar_angle = float(math.atan(radar_m) / (math.pi / 180))
        #                 radar_rotate_angle = float(90 - abs(radar_angle))

        #                 if radar_angle < 0:
        #                     radar_rotate_angle = -radar_rotate_angle

        #                 if pov_speed < 0:
        #                     np.append(a_points, (self.rotate_point(
        #                         radar_rotate_angle, it.distX, it.distY + self.radar_width / 2, it.distX, it.distY)))
        #                     np.append(a_points, (self.rotate_point(
        #                         radar_rotate_angle, it.distX, it.distY - self.adar_width / 2, it.distX, it.distY)))
        #                     np.append(a_points, (self.rotate_point(radar_rotate_angle, X_radar(
        #                         0, 0), X_radar(1, 0) - self.radar_width / 2, X_radar(0, 0), X_radar(1, 0))))
        #                     np.append(a_points, (self.rotate_point(radar_rotate_angle, X_radar(
        #                         0, 0), X_radar(1, 0) + self.radar_width / 2, X_radar(0, 0), X_radar(1, 0))))

        #                     path_index = 0
        #                     for pred in self.collision_path.pathPoints:
        #                         path_index += 1
        #                         if path_index % 4 == 1:
        #                             continue
        #                         b_points = np.array(np.matrix())

        #                         np.append(b_points, (self.rotate_point(
        #                             nowZaxis/100*path_index, pred.X, pred.Y + (car_width / 2), pred.X, pred.Y)))
        #                         np.append(b_points, (self.rotate_point(
        #                             nowZaxis/100*path_index, pred.X, pred.Y - (car_width / 2), pred.X, pred.Y)))
        #                         np.append(b_points, (self.rotate_point(
        #                             nowZaxis/100*path_index, pred.X - car_length, pred.Y - (car_width / 2), pred.X, pred.Y)))
        #                         np.append(b_points, (self.rotate_point(
        #                             nowZaxis/100*path_index, pred.X - car_length, pred.Y + (car_width / 2), pred.X, pred.Y)))

        #                         if self.intersect(a_points, b_points) and self.intersect(b_points, a_points):
        #                             ttr_ego = float(4 * path_index)/100
        #                             ttr_target = float(
        #                                 pow(pow(pred.X - it.distX, 2) + pow(pred.Y - it.distY, 2), 0.5) / abs(pov_speed))
        #                             ttc_threshold = float(
        #                                 abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5) + 1.5)

        #                             if(abs(ttr_target - ttr_ego) < 1.5 and self.min_max(ttr_target, ttr_ego, 0) < ttc_threshold):
        #                                 colli = Int8
        #                                 colli.data = id_name
        #                                 np.append(
        #                                     colli_arr.data.push_back(it.id))

        #                             ttc_threshold = (
        #                                 abs(pov_speed) / (2 * 9.8 * 0.3)) + 1
        #                             if(abs(ttr_target - ttr_ego) < 1 and self.min_max(ttr_target, ttr_ego, 0) < ttc_threshold):
        #                                 aeb = Int8
        #                                 aeb.data = it.id
        #                                 aeb_arr.data.push_back(it.id)
        #                                 print(
        #                                     "AEB->\nRadar ID : %d  Radar Speed : %f  Vehicle Speed : %f\n", it.id, pov_speed, nowSpeed)

        #                             break

        #             p = Point
        #             p.z = 1
        #             p.x = it.distX
        #             p.y = it.distY
        #             np.append(kalman_marker.points, p)
        #             p.x = X_radar(0, 0)
        #             p.y = X_radar(1, 0)
        #             np.append(kalman_marker.points, p)

        #             if(abs(X_radar(0, 0) - it.distX) < 100 and abs(X_radar(1, 0) - it.distY) < 100):
        #                 np.append(kalman_markers.markers, kalman_marker)

        #         self.collision_pub.publish(colli_arr)
        #         self.aeb_pub.publish(aeb_arr)

        #     if (it.isDanger):
        #         # gray: Danger
        #         marker_sphere.ns = "Danger"
        #         marker_sphere.color.r = float(0.7)
        #         marker_sphere.color.g = float(0.7)
        #         marker_sphere.color.b = float(0.7)
        #         marker_sphere.color.a = float(1.0)

        #     if RVIZ_TEXT:
        #         marker_text = Marker(header=self.getHeader(
        #         ), ns="text", id=it.id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)

        #         vrel = float(pow(pow(it.vrelX, 2)+pow(it.vrelY, 2), 0.5))
        #         if(it.vrelX < 0):
        #             vrel = -vrel

        #         ss = "ID: " + str(it.id) + "\n"
        #         ss += "DistX: " + str(it.distX) + "\n"
        #         ss += "DistY: " + str(it.distY) + "\n"
        #         ss += "VrelLong: " + str(it.vrelX) + "\n"
        #         ss += "VrelLat: " + str(it.vrelY) + "\n"
        #         ss += "Distance: " + \
        #             str(math.sqrt(pow(it.distX, 2) + pow(it.distY, 2))) + "\n"
        #         # ss += "Abs_speed: " + str(radar_abs_speed[it.id]) + "\n"
        #         ss += "Vrel: " + str(vrel) + "\n"
        #         ss += "car_speed: " + str(nowSpeed) + "\n"
        #         ss += "Angle: " + \
        #             str(math.atan2(it.distY, it.distX) * 180 / math.pi) + "\n"

        #         marker_text.text = ss

        #         marker_text.pose.position.x = it.distX
        #         marker_text.pose.position.y = it.distY
        #         marker_text.pose.position.z = 0.4
        #         marker_text.pose.orientation.x = 0.0
        #         marker_text.pose.orientation.y = 0.0
        #         marker_text.pose.orientation.z = 0.0
        #         marker_text.pose.orientation.w = 1.0
        #         marker_text.scale.z = 0.5

        #         marker_text.color.r = float(1.0)
        #         marker_text.color.g = float(1.0)
        #         marker_text.color.b = float(1.0)
        #         marker_text.color.a = 1.0

        #     if RVIZ_ARROW:
        #         marker_arrow = Marker()

        #         if (it.dynProp != 1):  # not stationary
        #             rpSpeed = float(math.hypot(nowSpeed + it.vrelX, it.vrelY))
        #             marker_arrow = Marker(header=self.getHeader(), ns="arrow", id=it.id, type=Marker.ARROW, action=Marker.ADD,
        #                                   pose=Pose(x=it.distX, y=it.distY, z=0.2), scale=Vector3(x=abs(rpSpeed), y=0.2, z=0.1), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

        #             arrowDir = Double(math.atan2(
        #                 it.vrelY, nowSpeed + it.vrelX))

        #             marker_arrow.pose.orientation.x = 0.0 * \
        #                 math.sin(arrowDir/2.0)
        #             marker_arrow.pose.orientation.y = 0.0 * \
        #                 math.sin(arrowDir/2.0)
        #             marker_arrow.pose.orientation.z = 1.0 * \
        #                 math.sin(arrowDir/2.0)
        #             marker_arrow.pose.orientation.w = math.cos(arrowDir/2.0)

        #             if (it.rcs > RCS_filter and rpSpeed > 0.1):
        #                 np.append(marArr.markers, marker_arrow)

        #     if (it.rcs > RCS_filter):
        #         marArr.markers.push_back(marker_sphere)
        #         if RVIZ_TEXT:
        #             np.append(marArr.markers, marker_text)

        # for i in range(last_id + 1, 100):
        #     disappear_time[i] += 1
        #     # std::cout << "(" << i << ":" << disappear_time[i] << ") ";
        #     if(disappear_time[i] >= 3):
        #         radarTraj[i].pathPoints[:] = []

        # if len((RadarPoints().rps) > 0):
        #     self.radar_predict_pub.publish(kalman_markers)
        #     self.markerArr_pub.publish(marArr)
        #     if RVIZ_RADARPOINTS_TRAJECTORY:
        #         self.radar_trajectory_pub.publish(radarTraj_markers)

        # rospy.spin()
