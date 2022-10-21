#! /usr/bin/env python3
# coding=utf-8
import sys
import subprocess
import time

import rospy
import roslaunch
import rospkg

from config.config import rgb_config, radar_config, default_config, CameraType


def main():
    try:
        roscore_popen_file = open("roscore_popen.log", "w+")
        roscore_popen_err_file = open("roscore_popen_err.log", "w+")
        roscore = subprocess.Popen('roscore', stdout=roscore_popen_file, stderr=roscore_popen_err_file)     
        time.sleep(1)  # wait a bit to be sure the roscore is really launched
        
        rospy.loginfo("starting roslaunch")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        config = roslaunch.config.ROSLaunchConfig()

        rospack = rospkg.RosPack()

        radar_names = radar_config.names
        for radar_name in radar_names:
            namespace = "/radar/" + radar_name + "/"
            # TODO
            # setup radar can (ip link up)
            
            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "decode_radar",
                    name="decode_radar_" + radar_name,
                    output="screen",
                    args="{}".format(radar_name),
                    namespace=namespace
                )
            )
        
        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "transform_radar.py",
                output="screen",
                namespace="/radar",
                name="transform_radar"
            )
        )

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "_ACC.py",
                output="screen",
                namespace="acc",
                name="ACC"
            )
        )

        if default_config.use_gui:
            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "visual_radar.py",
                    output="screen",
                    namespace="/radar",
                    name="visual_radar"
                )
            )
        
        if default_config.use_gui:
            print(rospack.get_path("ars408_ros") + "/rviz/record.rviz")
            config.add_node(
                roslaunch.core.Node(
                    "rviz",
                    "rviz",
                    name="rviz",
                    args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/record.rviz")
                )
            )

            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "visual_motion.py",
                    name="visual_motion",
                    namespace="/motion"
                )
            )
        
        
        launch = roslaunch.scriptapi.ROSLaunch()

        config.assign_machines()
        launch.parent.config = config
        launch.parent.start()

        rospy.init_node("root")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # dummy loop
            rate.sleep()

    except Exception as e:
        print(str(e))
        sys.exit()
    finally:
        launch.stop()
        # roscore.terminate()


if __name__ == "__main__":
    main()
