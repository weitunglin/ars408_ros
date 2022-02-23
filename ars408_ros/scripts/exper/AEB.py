#! /usr/bin/env python3
# coding=utf-8
import os
import math
from pickle import FALSE
from tokenize import Double

import rospy
import numpy as np
import yaml

from ars408_msg.msg import RadarPoint, RadarPoints, spinOnce
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

frameRate = config['frameRate']


radar_period = rospy.Time.now()

M_PI = 3.14159265358979323846

RVIZ_TEXT = False
RVIZ_ARROW = False

RVIZ_RANGE = True
RADAR_PREDICT = True
COLLISION_RANGE = True
RVIZ_RADARPOINTS_TRAJECTORY = True

Q_radar = np.matrix(4, 4)
R_radar = np.matrix(4, 4)
H_radar = np.matrix(4, 4)
B_radar = np.matrix(4, 1)
U_radar = np.matrix(1, 1)

nowSpeed = 0.0
nowZaxis = 0.0
car_width = 1.5
car_length = 5.0
radar_width = 0.5

RCS_filter = -10000.0
radar_abs_speed = np.zeros(100)
disappear_time = dict(int, int)
radarTraj = dict(int, pathPoints)

collision_path = pathPoints()

# Custom Class


class AEB():
    def __init__(self, radarChl):
        self.radarPoints = []
        self.radarChannel = radarChl

        self.markerArr_pub = rospy.Publisher(
            self.radar_channel + "/" + config["topic_MarkersArr"], MarkerArray, queue_size=1)

        self.collision_pub = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Collision"], Int8MultiArray, queue_size=1)

        self.aeb_pub = rospy.Publisher(
            self.radar_channel + "/" + config["topic_AEB"], Int8MultiArray, queue_size=1)

        self.radar_predict_pub = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Radar_Predict_pub"], MarkerArray, queue_size=1)

        self.radar_trajectory_pub = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Radar_Trajectory_pub"], MarkerArray, queue_size=1)

    def getHeader(self) -> Header:
        return Header(frame_id=self.radar_channel, stamp=rospy.Time.now())

    def kf_predict(self, X, P, F, B, U, Q):
        X = F * X
        P = F * P * F.getT() + Q

    def kf_update(self, X, P, Y, H, R):
        V = Y - H * X
        S = H * P * H.getT() + R
        K = P * H.getT() * S.getI()
        X = X + K * V
        P = P - K * H * P

    def rotate_point(self, angle, x, y, center_x, center_y) -> np.matrix:
        rotate = float(angle * M_PI / 180)

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

    def intersect(self, a,  b) -> bool:
        for i in np.size(a):
            cur = np.matrix(a[i])
            next = np.matrix(a[(i + 1) % a.size()])
            edge = np.matrix(next - cur)

            axis = np.matrix(-edge(1), edge(0))

            aMax = -np.Infinity
            aMin = np.Infinity
            bMax = -np.Infinity
            bMin = np.Infinity

            for v in a:
                proj = float(axis.dot(v))
                if(proj < aMin):
                    aMin = proj
                if(proj > aMax):
                    aMax = proj

            for v in b:
                proj = float(axis.dot(v))
                if(proj < bMin):
                    bMin = proj
                if(proj > bMax):
                    bMax = proj

            if (aMin < bMax and aMax > bMin) or (bMin < aMax and bMin > aMin):
                continue
            else:
                return False

        return True

    def min_max(value1, value2, flag) -> float:
        # min
        if(flag == 0):
            if(value1 > value2):
                return value2
            else:
                return value1

        # max
        else:
            if(value1 > value2):
                return value1
            else:
                return value2

    def callbackAEB(self, data: RadarPoints):

        time_diff = (rospy.Time.now() - radar_period).to_sec()
        radar_period = rospy.Time.now()

        if RVIZ_RANGE:
            #
            # RVIZ RADAR RANGE
            #
            rviz_range()

        #
        # [1] Show Radar
        # [2] Kalman predict Radar Path
        # [3] AEB
        #
        collision_markers = MarkerArray()
        marArr = MarkerArray()
        radarTraj_markers = MarkerArray()
        kalman_markers = MarkerArray()
        colli_arr = Int8MultiArray()

        marker = Marker(header=self.getHeader())
        marker.header.stamp = rospy.Time.now()

        # Clear all the markers
        marker.action = Marker.DELETEALL
        marArr.markers.append(marker)
        self.markerArr_pub.publish(marArr)

        pathPs = pathPoints()

        last_id = int(-1)
        aeb_arr = Int8MultiArray()

        for it in RadarPoints().rps:
            # Radar Speed calculation
            pov_speed = float(math.sqrt(pow(it.vrelX, 2) + pow(it.vrelY, 2)))

            if it.vrelX < 0:
                pov_speed = -pov_speed

            acc = pov_speed - radar_abs_speed[it.id]

            radar_abs_speed[it.id] = pov_speed

            # [TODO] AEB radarTraj
            disappear_time[it.id] = 0
            if last_id + 1 != it.id:
                for i in range(last_id + 1, it.id):
                    disappear_time[i] += 1

                    if disappear_time[i] >= 3:
                        radarTraj[i].pathPoints()[:] = []

            last_id = it.id

            id_name = int(it.id)

            # RVIZ RADAR SHOW
            marker_sphere = Marker(rviz_radar(radarTraj_markers, it))

            # [TODO]: AEB
            if RADAR_PREDICT:

                if(np.size(radarTraj[id_name].pathPoints) >= 2):

                    init_x = radarTraj[id_name].pathPoints[0].X
                    init_y = radarTraj[id_name].pathPoints[0].Y
                    init_vx = (radarTraj[id_name].pathPoints[1].X -
                               radarTraj[id_name].pathPoints[0].X) / time_diff
                    init_vy = (radarTraj[id_name].pathPoints[1].Y -
                               radarTraj[id_name].pathPoints[0].Y) / time_diff

                    X_radar = np.matrix([init_x, init_y, init_vx, init_vy])

                    F_radar = np.matrix(
                        [[1, 0, time_diff, 0], [0, 1, 0, time_diff], [0, 0, 1, 0], [0, 0, 0, 1]])

                    P_radar = np.matrix([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [
                                        0, 0, 0.1, 0], [0, 0, 0, 0.1]])

                    cur_radar_speed = float()

                    for t in range(1, np.size(radarTraj[id_name].pathPoints)):
                        velocity_x = float(
                            radarTraj[id_name].pathPoints[t].X - radarTraj[id_name].pathPoints[t - 1].X)/time_diff
                        velocity_y = float(
                            radarTraj[id_name].pathPoints[t].Y - radarTraj[id_name].pathPoints[t - 1].Y)/time_diff

                        if t == np.size(radarTraj[id_name].pathPoints) - 1:
                            F_radar = np.matrix(
                                [[1, 0, 4, 0], [0, 1, 0, 4], [0, 0, 1, 0], [0, 0, 0, 1]])

                            cur_radar_speed = pow(
                                pow(velocity_x, 2) + pow(velocity_y, 2), 0.5)

                        Y_radar = np.matrix(
                            [[1, 0, 4, 0], [0, 1, 0, 4], [0, 0, 1, 0], [0, 0, 0, 1]])

                        # kalman filter
                        self.kf_predict(X_radar, P_radar, F_radar,
                                        B_radar, U_radar, Q_radar)
                        self.kf_update(X_radar, P_radar,
                                       Y_radar, H_radar, R_radar)

                    kalman_marker = Marker(header=self.getHeader(), ns="kalman_markers", id=it.id, type=Marker.LINE_STRIP, action=Marker.ADD, lifetime=rospy.rostime.Duration(0.1),
                                           pose=Pose(orientation=Quaternion(w=1.0)), scale=Vector3(x=0.5, z=0.5), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

                    if COLLISION_RANGE:
                        a_points = np.array(np.matrix())

                        radar_m = float(X_radar(0, 0) - it.distX) / \
                            (X_radar(1, 0) - it.distY)
                        radar_angle = float(math.atan(radar_m) / (M_PI / 180))
                        radar_rotate_angle = float(90 - abs(radar_angle))

                        if radar_angle < 0:
                            radar_rotate_angle = -radar_rotate_angle

                        if pov_speed < 0:
                            np.append(a_points, (self.rotate_point(
                                radar_rotate_angle, it.distX, it.distY + radar_width / 2, it.distX, it.distY)))
                            np.append(a_points, (self.rotate_point(
                                radar_rotate_angle, it.distX, it.distY - radar_width / 2, it.distX, it.distY)))
                            np.append(a_points, (self.rotate_point(radar_rotate_angle, X_radar(
                                0, 0), X_radar(1, 0) - radar_width / 2, X_radar(0, 0), X_radar(1, 0))))
                            np.append(a_points, (self.rotate_point(radar_rotate_angle, X_radar(
                                0, 0), X_radar(1, 0) + radar_width / 2, X_radar(0, 0), X_radar(1, 0))))

                            path_index = 0
                            for pred in collision_path.pathPoints():
                                path_index += 1
                                if path_index % 4 == 1:
                                    continue
                                b_points = np.array(np.matrix())

                                np.append(b_points, (self.rotate_point(
                                    nowZaxis/100*path_index, pred.X, pred.Y + (car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotate_point(
                                    nowZaxis/100*path_index, pred.X, pred.Y - (car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotate_point(
                                    nowZaxis/100*path_index, pred.X - car_length, pred.Y - (car_width / 2), pred.X, pred.Y)))
                                np.append(b_points, (self.rotate_point(
                                    nowZaxis/100*path_index, pred.X - car_length, pred.Y + (car_width / 2), pred.X, pred.Y)))

                                if self.intersect(a_points, b_points) and self.intersect(b_points, a_points):
                                    ttr_ego = float(4 * path_index)/100
                                    ttr_target = float(
                                        pow(pow(pred.X - it.distX, 2) + pow(pred.Y - it.distY, 2), 0.5) / abs(pov_speed))
                                    ttc_threshold = float(
                                        abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5) + 1.5)

                                    if(abs(ttr_target - ttr_ego) < 1.5 and self.min_max(ttr_target, ttr_ego, 0) < ttc_threshold):
                                        colli = Int8
                                        colli.data = id_name
                                        np.append(
                                            colli_arr.data.push_back(it.id))

                                    ttc_threshold = (
                                        abs(pov_speed) / (2 * 9.8 * 0.3)) + 1
                                    if(abs(ttr_target - ttr_ego) < 1 and self.min_max(ttr_target, ttr_ego, 0) < ttc_threshold):
                                        aeb = Int8
                                        aeb.data = it.id
                                        aeb_arr.data.push_back(it.id)
                                        print(
                                            "AEB->\nRadar ID : %d  Radar Speed : %f  Vehicle Speed : %f\n", it.id, pov_speed, nowSpeed)

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

                self.collision_pub.publish(colli_arr)
                self.aeb_pub.publish(aeb_arr)

            if (it.classT == 0x00):
                # White: point
                marker_sphere.ns = "point"
                marker_sphere.color.r = float(1.0)
                marker_sphere.color.g = float(1.0)
                marker_sphere.color.b = float(1.0)
                marker_sphere.color.a = float(1.0)
            elif (it.classT == 0x01):
                # Red: car
                marker_sphere.ns = "car"
                marker_sphere.color.r = float(1.0)
                marker_sphere.color.g = float(0.0)
                marker_sphere.color.b = float(0.0)
                marker_sphere.color.a = float(1.0)

            elif (it.classT == 0x02):
                # Purple： truck
                marker_sphere.ns = "truck"
                marker_sphere.color.r = float(1.0)
                marker_sphere.color.g = float(0.0)
                marker_sphere.color.b = float(1.0)
                marker_sphere.color.a = float(1.0)

            elif (it.classT == 0x03 or it.classT == 0x07):
                # Blue: reserved
                marker_sphere.ns = "reserved"
                marker_sphere.color.r = float(0.0)
                marker_sphere.color.g = float(0.0)
                marker_sphere.color.b = float(1.0)
                marker_sphere.color.a = float(1.0)

            elif (it.classT == 0x04):
                # Yellow: motorcycle
                marker_sphere.ns = "motorcycle"
                marker_sphere.color.r = float(1.0)
                marker_sphere.color.g = float(1.0)
                marker_sphere.color.b = float(0.0)
                marker_sphere.color.a = 1.0

            elif (it.classT == 0x05):
                # Green: bicycle
                marker_sphere.ns = "bicycle"
                marker_sphere.color.r = float(0.0)
                marker_sphere.color.g = float(1.0)
                marker_sphere.color.b = float(0.0)
                marker_sphere.color.a = float(1.0)

            elif (it.classT == 0x06):
                # Cyan: wide
                marker_sphere.ns = "wide"
                marker_sphere.color.r = float(0.0)
                marker_sphere.color.g = float(1.0)
                marker_sphere.color.b = float(1.0)
                marker_sphere.color.a = float(1.0)

            else:
                # Orange: others
                marker_sphere.ns = "others"
                marker_sphere.color.r = float(1.0)
                marker_sphere.color.g = float(0.5)
                marker_sphere.color.b = float(0.0)
                marker_sphere.color.a = float(1.0)

            if (it.isDanger):
                # gray: Danger
                marker_sphere.ns = "Danger"
                marker_sphere.color.r = float(0.7)
                marker_sphere.color.g = float(0.7)
                marker_sphere.color.b = float(0.7)
                marker_sphere.color.a = float(1.0)

            if RVIZ_TEXT:
                marker_text = Marker(header=self.getHeader(
                ), ns="text", id=it.id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)

                vrel = float(pow(pow(it.vrelX, 2)+pow(it.vrelY, 2), 0.5))
                if(it.vrelX < 0):
                    vrel = -vrel

                ss = "ID: " + str(it.id) + "\n"
                ss += "DistX: " + str(it.distX) + "\n"
                ss += "DistY: " + str(it.distY) + "\n"
                ss += "VrelLong: " + str(it.vrelX) + "\n"
                ss += "VrelLat: " + str(it.vrelY) + "\n"
                ss += "Distance: " + \
                    str(math.sqrt(pow(it.distX, 2) + pow(it.distY, 2))) + "\n"
                # ss += "Abs_speed: " + str(radar_abs_speed[it.id]) + "\n"
                ss += "Vrel: " + str(vrel) + "\n"
                ss += "car_speed: " + str(nowSpeed) + "\n"
                ss += "Angle: " + \
                    str(math.atan2(it.distY, it.distX) * 180 / M_PI) + "\n"

                marker_text.text = ss

                marker_text.pose.position.x = it.distX
                marker_text.pose.position.y = it.distY
                marker_text.pose.position.z = 0.4
                marker_text.pose.orientation.x = 0.0
                marker_text.pose.orientation.y = 0.0
                marker_text.pose.orientation.z = 0.0
                marker_text.pose.orientation.w = 1.0
                marker_text.scale.z = 0.5

                marker_text.color.r = float(1.0)
                marker_text.color.g = float(1.0)
                marker_text.color.b = float(1.0)
                marker_text.color.a = 1.0

            if RVIZ_ARROW:
                marker_arrow = Marker()

                if (it.dynProp != 1):  # not stationary
                    rpSpeed = float(math.hypot(nowSpeed + it.vrelX, it.vrelY))
                    marker_arrow = Marker(header=self.getHeader(), ns="arrow", id=it.id, type=Marker.ARROW, action=Marker.ADD,
                                          pose=Pose(x=it.distX, y=it.distY, z=0.2), scale=Vector3(x=abs(rpSpeed), y=0.2, z=0.1), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))

                    arrowDir = Double(math.atan2(
                        it.vrelY, nowSpeed + it.vrelX))

                    marker_arrow.pose.orientation.x = 0.0 * \
                        math.sin(arrowDir/2.0)
                    marker_arrow.pose.orientation.y = 0.0 * \
                        math.sin(arrowDir/2.0)
                    marker_arrow.pose.orientation.z = 1.0 * \
                        math.sin(arrowDir/2.0)
                    marker_arrow.pose.orientation.w = math.cos(arrowDir/2.0)

                    if (it.rcs > RCS_filter and rpSpeed > 0.1):
                        np.append(marArr.markers, marker_arrow)

            if (it.rcs > RCS_filter):
                marArr.markers.push_back(marker_sphere)
                if RVIZ_TEXT:
                    np.append(marArr.markers, marker_text)

        for i in range(last_id + 1, 100):
            disappear_time[i] += 1
            # std::cout << "(" << i << ":" << disappear_time[i] << ") ";
            if(disappear_time[i] >= 3):
                radarTraj[i].pathPoints[:] = []

        if (np.size(RadarPoints().rps) > 0):
            self.radar_predict_pub.publish(kalman_markers)
            self.markerArr_pub.publish(marArr)
            if RVIZ_RADARPOINTS_TRAJECTORY:
                self.radar_trajectory_pub.publish(radarTraj_markers)

        rospy.spin()


# Subscriber callback
def callbackPoint1(data):
    global radarPt

    radarPt.radarPoints = data.rps


def listener(radarChl):

    rospy.init_node("AEB")
    rosrate = rospy.Rate(frameRate)

    global radarPt
    radarPt = AEB(radarChl)

    # Subscribe RadarPubRT
    print("AEB Subscribe: {}".format(radarChl))
    sub1 = rospy.Subscriber(radarChl + "/radarPubRT", RadarPoints,
                            callbackPoint1, queue_size=1)

    # Publish
    pub1 = rospy.Publisher("/FUNC/AEB", RadarPoints, queue_size=1)

    while not rospy.is_shutdown():
        if not ("radarPt" in globals()):
            continue

        rosrate.sleep()


if __name__ == "__main__":

    try:
        # Get Param
        radarChannel = rospy.get_param('AEB/radarChannel')

        if radarChannel != None:
            listener(radarChannel)

    except rospy.ROSInternalException:
        pass
