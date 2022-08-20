from enum import Enum

import numpy as np

class CameraType(Enum):
    RGB = 0
    THERMAL = 1


class RGB(object):
    def __init__(self, c):
        self.size = tuple(c["size"])
        self.port = c["port"]
        self.codec = c["codec"]
        self.frame_rate = c["frame_rate"]
        self.camera_type = c["camera_type"]
        if self.camera_type == CameraType.RGB:
            self.K = c["K"]
            self.D = c["D"]
            self.intrinsic_matrix = np.array(self.K).reshape((3, 3))
            self.distortion_matrix = np.array(self.D).reshape((1, 4))
            self.R = np.eye(3, dtype=np.float64)
            self.P = np.zeros((3, 4), dtype=np.float64)
            self.P[:3,:3] = self.intrinsic_matrix[:3,:3]


class RGBConfig(object):
    def __init__(self):
        self.rgb_config = {
            # "rgb_name": {
            #   "size": [width (px), height (px)]
            #   "port": "/dev/video0"
            #   "codec": 
            #   "K": [size of 9 float array]
            #   "D": [size of 5 float array]
            #   "r2c": [size of 9 float array]
            # }
            "front_left": {
                "size": [1280, 712],
                "port": "/dev/camera_front_left",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "front_center": {
                "size": [1280, 712],
                "port": "/dev/camera_front_center",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "front_right": {
                "size": [1280, 712],
                "port": "/dev/camera_front_right",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_right": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_right",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_center": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_center",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "rear_left": {
                "size": [1280, 712],
                "port": "/dev/camera_rear_left",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.RGB,
                "K": [468.016638, -3.619860, 639.087418, 0.000000, 467.911725, 349.678282, 0.000000, 0.000000, 1.000000],
                "D": [-0.005040, 0.077882, -0.010632, 0.038149]
            },
            "thermal": {
                "size": [640, 512],
                "port": "/dev/camera_thermal",
                "codec": "MJPG",
                "frame_rate": 30,
                "camera_type": CameraType.THERMAL,
            }
        }
    
        self.model = {
            """
            dual_vision:
                dual image object detection. (rgb + thermal)

            rgb:
                single rgb image object detection. (batch of rgbs)
            """
            "dual_vision": [["front_center", "thermal"]],
            "rgb": ["front_left", "front_right", "rear_right", "rear_center", "rear_left"],

            "cfg": "/home/allen/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/cfg/bsw.cfg",
            "names": "/home/allen/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/data/bsw.names",
            "weights": "/home/allen/catkin_ws/src/ARS408_ros/ars408_package/PyTorch_YOLOv4/weights/best.pt",

            "image_size": tuple((608, 608)),
            "conf_thres": 0.6,
            "iou_thres": 0.4
        }

        self.names = self.rgb_config.keys()
        # for i in self.names:
        #     setattr(self, i, RGB(self.rgb_config[i]))
        self.rgbs = { i: RGB(self.rgb_config[i]) for i in self.names }
    
    def __getitem__(self, name):
        return self.rgbs[name]

rgb_config = RGBConfig()
