#! /usr/bin/env python3
# coding=utf-8
import os
import math
import rospy
import numpy as np
import yaml

from ars408_msg.msg import RadarPoints, GPSinfo, pathPoints, pathPoint
from std_msgs.msg import Int8MultiArray

# load config
with open(os.path.expanduser("~") + "/code/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)


RADAR_PREDICT = True
COLLISION_RANGE = True

# util function
def kfPredict(X, P, F, B, U, Q):
    X = F * X
    P = F * P * F.T + Q
    return X, P

def kfUpdate(X, P, Y, H, R):
    V = Y - H * X
    S = H * P * H.T + R
    K = P * H.T * S.I
    X = X + K * V
    P = P - K * H * P
    return X, P

def rotatePoint(angle, x, y, center_x, center_y):
    rotate = angle * math.pi / 180.0

    x -= center_x
    y -= center_y

    rotate_x = math.cos(rotate) * x - math.sin(rotate) * y + center_x
    rotate_y = math.sin(rotate) * x + math.cos(rotate) * y + center_y

    if abs(rotate_x) < 0.001:
        rotate_x = 0

    if abs(rotate_y) < 0.001:
        rotate_y = 0

    return np.array([rotate_x, rotate_y])

def intersect(a,  b) -> bool:
    for i in range(a.shape[0]):
        cur = np.array(a[i])
        next = np.array(a[(i + 1) % a.shape[0]])
        axis = next - cur

        a_max = -np.Infinity
        a_min = np.Infinity
        b_max = -np.Infinity
        b_min = np.Infinity

        for v in a:
            proj = axis.dot(v)
            if(proj < a_min):
                a_min = proj
            if(proj > a_max):
                a_max = proj

        for v in b:
            proj = axis.dot(v)
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


class AEB():
    def __init__(self, radar_channel):
        self.radar_channel = radar_channel
        self.radar_period = rospy.Time.now()

        self.Q_radar = np.matrix(np.identity(4) * 4e-4)
        self.R_radar = np.matrix(np.identity(4) * 1e-2)
        self.H_radar = np.matrix(np.identity(4))
        self.B_radar = np.matrix(np.zeros(4)).reshape(4, 1)
        self.U_radar = np.matrix([0]).reshape(1, 1)

        self.now_speed = 0.0
        self.now_zaxis = 0.0
        self.car_width = 1.5
        self.car_length = 5.0
        self.radar_width = 0.5

        self.disappear_time = { i: 0 for i in range(200) }
        self.radar_trajectory = { i: pathPoints() for i in range(200) }

        self.path_points: pathPoints = None
        self.radar_points: RadarPoints = None

        self.sub_radar_pub_rt = rospy.Subscriber(
            self.radar_channel + "/radarPubRT", RadarPoints, self.callbackRadarPubRT, queue_size=1)
        self.sub_path_points = rospy.Subscriber(
            self.radar_channel + "/" + config["topic_PathPoints"], pathPoints, self.callbackPathPoints, queue_size=1)
        self.sub_gps_info = rospy.Subscriber(
            config["topic_GPS"], GPSinfo, self.callbackGPS, queue_size=1)

        self.pub_collision = rospy.Publisher(
            self.radar_channel + "/" + config["topic_Collision"], Int8MultiArray, queue_size=1)
        self.pub_aeb = rospy.Publisher(
            self.radar_channel + "/" + config["topic_AEB"], Int8MultiArray, queue_size=1)

    def callbackGPS(self, data: GPSinfo):
        self.now_speed = data.speed
        self.now_zaxis = data.zaxis

    def callbackPathPoints(self, data: pathPoints):
        self.path_points = data

    def callbackRadarPubRT(self, data: RadarPoints):
        self.radar_points = data

    def loop(self):
        time_diff = (rospy.Time.now() - self.radar_period).to_sec()
        self.radar_period = rospy.Time.now()

        if self.radar_points is None or self.path_points is None:
            return

        colli_arr = Int8MultiArray()
        aeb_arr = Int8MultiArray()

        last_id = -1

        for it in self.radar_points.rps:
            if len(self.radar_trajectory[it.id].pathPoints) == 0:
                self.radar_trajectory[it.id].pathPoints.append(pathPoint(X=it.distX, Y=it.distY))
            elif self.radar_trajectory[it.id].pathPoints[-1].X != it.distX or \
                self.radar_trajectory[it.id].pathPoints[-1].Y != it.distY:
                if pow(pow(it.distX - self.radar_trajectory[it.id].pathPoints[-1].X, 2) +
                    pow(it.distY - self.radar_trajectory[it.id].pathPoints[-1].Y, 2), 0.5) > 3:
                    self.radar_trajectory[it.id].pathPoints.clear()
                if len(self.radar_trajectory[it.id].pathPoints) >= 5:
                    self.radar_trajectory[it.id].pathPoints.pop(0)
                self.radar_trajectory[it.id].pathPoints.append(pathPoint(X=it.distX, Y=it.distY))

            pov_speed = math.sqrt(pow(it.vrelX, 2.0) + pow(it.vrelY, 2.0))

            if it.vrelX < 0:
                pov_speed = -pov_speed

            self.disappear_time[it.id] = 0
            if last_id + 1 != it.id:
                for i in range(last_id + 1, it.id):
                    self.disappear_time[i] += 1

                    if self.disappear_time[i] >= 3:
                        self.radar_trajectory[i].pathPoints.clear()

            last_id = it.id

            if RADAR_PREDICT:
                if len(self.radar_trajectory[it.id].pathPoints) >= 2:
                    path_points = self.radar_trajectory[it.id].pathPoints
                    init_x = path_points[0].X
                    init_y = path_points[0].Y
                    init_vx = (path_points[1].X - path_points[0].X) / time_diff
                    init_vy = (path_points[1].Y - path_points[0].Y) / time_diff

                    X_radar = np.matrix([init_x, init_y, init_vx, init_vy]).reshape(4, 1)

                    F_radar = np.matrix([1, 0, time_diff, 0,
                                         0, 1, 0, time_diff,
                                         0, 0, 1, 0,
                                         0, 0, 0, 1]).reshape(4, 4)

                    P_radar = np.matrix([0.1, 0, 0, 0,
                                         0, 0.1, 0, 0,
                                         0, 0, 0.1, 0,
                                         0, 0, 0, 0.1]).reshape(4, 4)

                    for t in range(1, len(path_points)):
                        velocity_x = (path_points[t].X - path_points[t - 1].X) / time_diff
                        velocity_y = (path_points[t].Y - path_points[t - 1].Y) / time_diff

                        if t == len(path_points) - 1:
                            F_radar = np.matrix([1, 0, 4, 0,
                                                 0, 1, 0, 4,
                                                 0, 0, 1, 0,
                                                 0, 0, 0, 1]).reshape(4, 4)

                        Y_radar = np.matrix([path_points[t].X, path_points[t].Y, velocity_x, velocity_y]).reshape(4, 1)

                        X_radar, P_radar = kfPredict(X_radar, P_radar, F_radar,
                                       self.B_radar, self.U_radar, self.Q_radar)
                        X_radar, P_radar = kfUpdate(X_radar, P_radar, Y_radar,
                                      self.H_radar, self.R_radar)

                    if COLLISION_RANGE:
                        a_points = np.array([])

                        radar_m = (X_radar[0, 0] - it.distX) / (X_radar[1, 0] - it.distY)
                        radar_angle = (math.atan(radar_m) / (math.pi / 180.0))
                        radar_rotate_angle = (90.0 - abs(radar_angle))

                        if radar_angle < 0:
                            radar_rotate_angle = -radar_rotate_angle

                        if pov_speed < 0:
                            a_points = np.append(a_points, rotatePoint(radar_rotate_angle, it.distX,
                                it.distY + self.radar_width / 2.0, it.distX, it.distY))
                            a_points = np.append(a_points, rotatePoint(radar_rotate_angle, it.distX,
                                it.distY - self.radar_width / 2.0, it.distX, it.distY))
                            a_points = np.append(a_points, rotatePoint(radar_rotate_angle, X_radar[0, 0],
                                X_radar[1, 0] - self.radar_width / 2.0, X_radar[0, 0], X_radar[1, 0]))
                            a_points = np.append(a_points, rotatePoint(radar_rotate_angle, X_radar[0, 0],
                                X_radar[1, 0] + self.radar_width / 2.0, X_radar[0, 0], X_radar[1, 0]))
                            a_points = a_points.reshape(4, 2)

                            path_index = 0
                            for pred in self.path_points.pathPoints:
                                path_index += 1
                                if path_index % 4 == 1:
                                    continue

                                b_points = np.array([])

                                b_points = np.append(b_points, rotatePoint(self.now_zaxis / 100.0 * path_index,
                                    pred.X, pred.Y + (self.car_width / 2), pred.X, pred.Y))
                                b_points = np.append(b_points, rotatePoint(self.now_zaxis / 100.0 * path_index,
                                    pred.X, pred.Y - (self.car_width / 2), pred.X, pred.Y))
                                b_points = np.append(b_points, rotatePoint(self.now_zaxis / 100.0 * path_index,
                                    pred.X - self.car_length, pred.Y - (self.car_width / 2), pred.X, pred.Y))
                                b_points = np.append(b_points, rotatePoint(self.now_zaxis / 100.0 * path_index,
                                    pred.X - self.car_length, pred.Y + (self.car_width / 2), pred.X, pred.Y))
                                b_points = b_points.reshape(4, 2)

                                if intersect(a_points, b_points) and intersect(b_points, a_points):
                                    print("intersect")
                                    ttr_ego = 4 * path_index / 100.0
                                    ttr_target = pow(pow(pred.X - it.distX, 2) + pow(pred.Y - it.distY, 2), 0.5) / abs(pov_speed)
                                    ttc_threshold = abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5) + 1.5

                                    if abs(ttr_target - ttr_ego) < 1.5 and minMax(ttr_target, ttr_ego, 0) < ttc_threshold:
                                        colli_arr.data.append(it.id)

                                    ttc_threshold = abs(pov_speed) / (2 * 9.8 * 0.3) + 1
                                    if (abs(ttr_target - ttr_ego) < 1 and minMax(ttr_target, ttr_ego, 0) < ttc_threshold):
                                        aeb_arr.data.append(it.id)
                                        print(f"AEB->\nRadar ID: {it.id}\tRadar Speed: {pov_speed}\tVehicle Speed:{self.now_speed}")

                                    break

                self.pub_collision.publish(colli_arr)
                self.pub_aeb.publish(aeb_arr)

        for i in range(last_id + 1, 100):
            self.disappear_time[i] += 1

            if self.disappear_time[i] >= 3:
                self.radar_trajectory[i].pathPoints.clear()


def listener(radar_channel):
    rospy.init_node("AEB")
    rosrate = rospy.Rate(config["frameRate"])

    aeb = AEB(radar_channel)

    while not rospy.is_shutdown():
        aeb.loop()
        rosrate.sleep()


if __name__ == "__main__":
    try:
        radar_channel = rospy.get_param('AEB/radarChannel')

        if radar_channel != None:
            listener(radar_channel)
    except rospy.ROSInternalException:
        pass
