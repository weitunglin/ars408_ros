#! /usr/bin/env python3
# coding=utf-8
from ast import arg
import sys
import subprocess
import time

import rospy
import roslaunch
import rospkg

from ars408_ros import rgb_config, radar_config, default_config


def main():
    try:
        roscore_popen_file = open("roscore_popen.log", "w+")
        roscore_popen_err_file = open("roscore_popen_err.log", "w+")
        roscore = subprocess.Popen('roscore', stdout=roscore_popen_file, stderr=roscore_popen_err_file)     
        time.sleep(2)  # wait a bit to be sure the roscore is really launched
        
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
            
            if default_config.recording:
                node_name = "socketbridge_" + radar_name
                config.add_param(
                    roslaunch.core.Param(namespace + node_name + "/can_device",
                    radar_config[radar_name].can_device)
                )
                config.add_node(
                    roslaunch.core.Node(
                        "socketcan_bridge",
                        "socketcan_to_topic_node",
                        name=node_name,
                        namespace=namespace
                    )
                )
            
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
        
        rgb_names = rgb_config.names
        for rgb_name in rgb_names:
            namespace = "/rgb/" + rgb_name + "/"
            if default_config.recording:
                config.add_node(
                    roslaunch.core.Node(
                        "ars408_ros",
                        "rgb_bridge.py",
                        name="rgb_bridge_" + rgb_name,
                        output="screen",
                        args="{}".format(rgb_name),
                        namespace=namespace
                    )
                )
            
            # calib node
            # config.add_node(
            #     roslaunch.core.Node(
            #         "ars408_ros",
            #         "calib.py",
            #     )
            # )

        # yolo node (torch)
        # config.add_node(
        #     roslaunch.core.Node(
        #         "ars408_ros",
        #         "yolo.py",
        #     )
        # )
        
        if default_config.recording:
            # motion to ros node
            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "motion_bridge.py",
                    name="motion_bridge",
                    namespace="/motion"
                )
            )
        
        if default_config.use_gui:
            print(rospack.get_path("ars408_ros") + "/rviz/default.rviz")
            config.add_node(
                roslaunch.core.Node(
                    "rviz",
                    "rviz",
                    name="rviz",
                    args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/default.rviz")
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
        roscore.terminate()


if __name__ == "__main__":
    main()
