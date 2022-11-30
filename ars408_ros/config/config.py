from enum import Enum
from typing import List
import math
import os
import numpy as np


class FusionConfig(object):
    def __init__(self, name: str, rgb_name: str, radar_name: str):
        """
        `rgb_name` and `radar_name` is for capability for fusing different sensors.
        but not implemented yet.
        """
        self.rgb_name = rgb_name
        self.radar_name = radar_name
        self.name = name


class DefaultConfig(object):
    frame_rate = 30

    # FIXME
    # fix flag structure
    use_gui = True
    use_cuda = True
    recording = False
    use_calib = True
    use_yolo = False
    use_yolo_half = True
    use_yolo_image = True
    use_radar_image = True
    use_dual_vision = True
    use_lta = True
    use_radar_polygon = True
    use_aeb = True

    sensor_list = [
        "front_left",
        "front_center",
        "front_right",
        "rear_right",
        "rear_center",
        "rear_left"
    ]

    class_depth = {
        "person": 1,
        "motor": 3,
        "bike": 3,
        "car": 5,
        "truck": 10,
        "bus": 10,
        "motor-people": 3,
        "bike-people": 3,
    }
    sensor_fusion = [
        FusionConfig(rgb_name=name, radar_name=name, name=name) for name in sensor_list
    ]


class Radar(object):
    def __init__(self, c):
        self.transform: List[float] = c["transform"]
        self.can_device: str = c["can_device"]
        self.projection_translate = c["projection_translate"]


class RadarConfig():
    def __init__(self):
        """
        "radar_name": {
            "transform": float[3] [-1, 0, -1.57] [x_trans (m), y_trans (m), rotate (rad)]
            "can_device": str     "can*"    
        }
        """
        self.radar_config = {
            "front_left": {
                "transform": [0, 1, 1.57],
                "can_device": "can3",
                "projection_translate": [0, 0, 0],
            },
            "front_center": {
                "transform": [0, 0, 0],
                "can_device": "can4",
                "projection_translate": [0, 0, 0],
            },
            "front_right": {
                "transform": [0, -1, -1.57],
                "can_device": "can5",
                "projection_translate": [0, 0, 0],
            },
            "rear_right": {
                "transform": [-5, -1, -1.57],
                "can_device": "can0",
                "projection_translate": [0, 0, 0],
            },
            "rear_center": {
                "transform": [-5, 0, math.pi],
                "can_device": "can1",
                "projection_translate": [0, 0, 0],
            },
            "rear_left": {
                "transform": [-5, 1, 1.57],
                "can_device": "can2",
                "projection_translate": [0, 0, 0],
            }
        }
        self.names = list(self.radar_config.keys())
        self.radars = {i: Radar(self.radar_config[i]) for i in self.names}

    def __getitem__(self, name):
        return self.radars[name]


class CameraType(Enum):
    RGB = 0
    THERMAL = 1


class RGB(object):
    def __init__(self, c):
        self.size = tuple(c["size"])
        self.port = c["port"]
        self.frame_rate = c["frame_rate"]
        self.camera_type = c["camera_type"]
        if self.camera_type == CameraType.RGB:
            self.K = c["K"]
            self.D = c["D"]
            self.intrinsic_matrix = np.array(self.K).reshape((3, 3))
            self.distortion_matrix = np.array(self.D).reshape((1, 4))
            self.R = np.eye(3, dtype=np.float64)
            self.P = np.zeros((3, 4), dtype=np.float64)
            self.P[:3, :3] = self.intrinsic_matrix[:3, :3]


class RGBConfig(object):
    def __init__(self):
        """
        "rgb_name": {
            "size": int[2]                [1280, 712] [width (pixel), height (pixel)]
            "port": str                   "/dev/video*" or "symlink to /dev/video*"
            "frame_rate": int             30
            "camera_type": CameraType     CameraType.RGB

            (if is RGB)
            "K": float[9]                 intrinsic matrix
            "D": float[4]                 distortion matrxi
        }
        """
        self.rgb_config = {
            "front_left": {
                "size": [1280, 712],
                "port": "/dev/camera_front_left",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "front_center": {
                "size": [1280, 712],
                "port": "/dev/camera_front_center",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "front_right": {
                "size": [1280, 712],
                "port": "/dev/camera_front_right",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_right": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_right",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_center": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_center",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_left": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_left",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "thermal": {
                "size": [640, 512],
                "port": "/dev/camera_thermal",
                "frame_rate": 30,
                "camera_type": CameraType.THERMAL,
            }
        }

        """
        dual_vision:
            dual image object detection. (rgb + thermal)

        rgb:
            single rgb image object detection. (batch of rgbs)
        """
        self.model = {
            "dual_vision": ["front_center", "thermal"],
            "rgb": ["front_left", "front_center", "front_right", "rear_right", "rear_center", "rear_left"],

            "cfg": os.path.expanduser("~") + "/micromax/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/cfg/bsw.cfg",
            "names": os.path.expanduser("~") + "/micromax/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/data/bsw.names",
            "weights": os.path.expanduser("~") + "/micromax/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/weights/best.pt",

            "dual_weights": os.path.expanduser("~") + "/micromax/catkin_ws/src/ARS408_ros/ars408_package/NVS/inference/weights/best.pt",
            "dual_image_size": (640, 640),
            "dual_conf_thres": 0.4,
            "dual_iou_thres": 0.45,

            "image_size": tuple((416, 416)),
            "conf_thres": 0.6,
            "iou_thres": 0.5
        }

        self.names = self.rgb_config.keys()
        self.rgbs = {i: RGB(self.rgb_config[i]) for i in self.names}

    def __getitem__(self, name):
        return self.rgbs[name]


class TopicConfig(object):
    pass


default_config = DefaultConfig()
radar_config = RadarConfig()
rgb_config = RGBConfig()
topic_config = TopicConfig()
