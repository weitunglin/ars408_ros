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
        # roscore_popen_file = open("roscore_popen.log", "w+")
        # roscore_popen_err_file = open("roscore_popen_err.log", "w+")
        # roscore = subprocess.Popen('roscore', stdout=roscore_popen_file, stderr=roscore_popen_err_file)     
        # time.sleep(1)  # wait a bit to be sure the roscore is really launched
        
        rospy.loginfo("starting roslaunch")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        config = roslaunch.config.ROSLaunchConfig()
        rospack = rospkg.RosPack()

        # rviz car model
        config.add_param(
            roslaunch.core.Param(
                "/car_model/robot_description",
                rospack.get_path("ars408_ros") + "/rviz/car_model/default.urdf"
            )
        )
        config.add_node(
            roslaunch.core.Node(
                "tf",
                "static_transform_publisher",
                namespace="/car_model",
                name="base_frame_broadcaster",
                args="-3 0 0 0 0 0 1 base_link ground_link 10"
            )
        )

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

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "visual_radar_points",
                output="screen",
                namespace="/radar",
                name="visual_radar_points"
            )
        )

        # if default_config.use_radar_polygon:
        #     config.add_node(
        #         roslaunch.core.Node(
        #             "ars408_ros",
        #             "radar_polygon.py",
        #             output="screen",
        #             name="radar_polygon"
        #         )
        #     )

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
            
            # if rgb_config[rgb_name].camera_type == CameraType.RGB and default_config.use_calib:
                # calib node
                # config.add_node(
                #     roslaunch.core.Node(
                #         "ars408_ros",
                #         "decompress_rgb.py",
                #         name="rgb_decompress_" + rgb_name,
                #         output="screen",
                #         args="{}".format(rgb_name),
                #         namespace=namespace
                #     )
                # )
                # config.add_node(
                #     roslaunch.core.Node(
                #         "ars408_ros",
                #         "calib_rgb.py",
                #         name="rgb_calib_" + rgb_name,
                #         output="screen",
                #         args="{}".format(rgb_name),
                #         namespace=namespace
                #     )
                # )

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

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "synchronization.py",
                name="synchronization",
                output="screen"
            )
        )

        # config.add_node(
        #     roslaunch.core.Node(
        #         "ars408_ros",
        #         "_ACC.py",
        #         output="screen",
        #         namespace="acc",
        #         name="ACC"
        #     )
        # )

        # if default_config.use_lta:
        #     config.add_node(
        #         roslaunch.core.Node(
        #             "ars408_ros",
        #             "lane_trace.py",
        #             name="lane_trace",
        #             output="screen"
        #         )
        #     )
        
        if default_config.use_aeb:
            config.add_node(
                roslaunch.core.Node(
                    "ars408_ros",
                    "AEB.py",
                    name="AEB",
                    output="screen"
                )
            )

        # TODO
        # (sensor_sync + yolo_torch + sensor_fusion)
        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "sensor_fusion.py",
                name="sensor_fusion",
                output="screen"
            )
        )
        
        # if default_config.recording:
        #     #motion to ros node
        #     config.add_node(
        #         roslaunch.core.Node(
        #             "ars408_ros",
        #             "motion_bridge.py",
        #             name="motion_bridge",
        #             namespace="/motion"
        #         )
        #     )
        
        if default_config.use_gui:
            print(rospack.get_path("ars408_ros") + "/rviz/default.rviz")
        config.add_node(
            roslaunch.core.Node(
                "rviz",
                "rviz",
                name="rviz",
                args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/rps.rviz")
            )
        )

        #     config.add_node(
        #         roslaunch.core.Node(
        #             "ars408_ros",
        #             "visual_motion.py",
        #             name="visual_motion",
        #             namespace="/motion"
        #         )
        #     )

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