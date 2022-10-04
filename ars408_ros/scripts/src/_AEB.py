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
with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
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
        
        if self.radar_points is None:
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
                x_plus, y_plus = 0, it.distY + it.distX * math.tan(math.radians(it.angle))
                ttr_target = math.sqrt(math.pow(x_plus - it.distX, 2) + math.pow(y_plus - it.distY, 2)) \
                    / (math.pow(math.pow(it.vrelX, 2) + math.pow(it.vrelY, 2), 0.5) + 1e-6)
                ttr_ego = math.sqrt(math.pow(x_plus, 2) + math.pow(y_plus, 2)) / (self.now_speed + 0.3 + 1e-6)
                
                ttc = min(ttr_target, ttr_ego)

                ttc_threshold = abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5) + 1.5
                if abs(ttr_target - ttr_ego) < 1.5 and ttc < ttc_threshold:
                    colli_arr.data.append(it.id)
                    rospy.loginfo("FCTA Warning")
                    rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))

                ttc_threshold = abs(pov_speed) / (2 * 9.8 * 0.3) + 1
                if (abs(ttr_target - ttr_ego) < 1 and ttc < ttc_threshold):
                    aeb_arr.data.append(it.id)
                    rospy.loginfo("FCTA Danger")
                    rospy.loginfo("ttr: {} , ttc: {}".format(abs(ttr_target - ttr_ego), ttc))

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