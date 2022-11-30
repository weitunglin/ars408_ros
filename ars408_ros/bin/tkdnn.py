#! /usr/bin/env python3
# coding=utf-8
import sys

import rospy
import roslaunch
import rospkg

from config.config import rgb_config, default_config, CameraType


def main():
    try:
        rospy.loginfo("starting roslaunch")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        config = roslaunch.config.ROSLaunchConfig()
        rospack = rospkg.RosPack()

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

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "test_sync.py",
                name="test_sync",
                output="screen"
            )
        )

        config.add_node(
            roslaunch.core.Node(
                "ars408_ros",
                "tkdnn_batch",
                name="tkdnn_batch",
                output="screen"
            )
        )

        config.add_node(
            roslaunch.core.Node(
                "rviz",
                "rviz",
                name="rviz",
                args="-d {}".format(rospack.get_path("ars408_ros") + "/rviz/tkdnn.rviz")
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


if __name__ == "__main__":
    main()
