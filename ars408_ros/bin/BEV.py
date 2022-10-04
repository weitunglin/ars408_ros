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
        
        rgb_names = rgb_config.names
        for rgb_name in rgb_names:
            namespace = "/rgb/" + rgb_name + "/"

            # calib node
            if rgb_config[rgb_name].camera_type == CameraType.RGB and default_config.use_calib:
                config.add_node(
                    roslaunch.core.Node(
                        "ars408_ros",
                        "calib_rgb.py",
                        name="rgb_calib_" + rgb_name,
                        output="screen",
                        args="{}".format(rgb_name),
                        namespace=namespace
                    )
                )

            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "BEV_rgb.py",
                    name="BEV_rgb_" + rgb_name,
                    output="screen",
                    args="{}".format(rgb_name),
                    namespace=namespace
                )
            )

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "BEV_fusion_rgb.py",
                output="screen",
                namespace="/rgb",
                name="BEV_fusion_rgb"
            )
        )

        if default_config.use_gui:
            print(rospack.get_path("ars408_ros") + "/rviz/BEV.rviz")
            config.add_node(
                roslaunch.core.Node(
                    "rviz",
                    "rviz",
                    name="rviz",
                    args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/BEV.rviz")
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
