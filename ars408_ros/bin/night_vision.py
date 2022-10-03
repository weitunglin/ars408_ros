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
            
            if rgb_config[rgb_name].camera_type == CameraType.RGB and default_config.use_calib:
                # calib node
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

        # if default_config.use_yolo:
            # # yolo node (torch)
            # config.add_node(
            #     roslaunch.core.Node(
            #         "ars408_ros",
            #         "yolo_torch.py",
            #         output="screen",
            #         namespace="model"
            #     )
            # )
            # # dual
            # config.add_node(
            #     roslaunch.core.Node(
            #         "ars408_ros",
            #         "dual_vision_torch.py",
            #         output="screen",
            #         namespace="model"
            #     )
            # )

        # if default_config.use_dual_vision:
        #     config.add_node(
        #         roslaunch.core.Node(
        #             "ars408_ros",
        #             "dual_vision.py",
        #             name="dual_vision",
        #             output="screen"
        #         )
        #     )

        # TODO
        # (sensor_sync + dual_vision_torch + sensor_fusion)
        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "sensor_fusion.py",
                name="sensor_fusion",
                output="screen"
            )
        )
        
        if default_config.use_gui:
            print(rospack.get_path("ars408_ros") + "/rviz/night_vision.rviz")
            config.add_node(
                roslaunch.core.Node(
                    "rviz",
                    "rviz",
                    name="rviz",
                    args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/night_vision.rviz")
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