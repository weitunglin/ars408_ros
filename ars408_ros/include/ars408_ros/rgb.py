from enum import Enum


class CameraType(Enum):
    RGB = 0
    THERMAL = 1


class RGB(object):
    def __init__(self, c):
        self.size = c["size"]
        self.port = c["port"]
        self.codec = c["codec"]
        self.frame_rate = c["frame_rate"]
        self.camera_type = c["camera_type"]
        if self.camera_type == CameraType.RGB:
            self.K = c["K"]
            self.D = c["D"]


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
    
        self.names = self.rgb_config.keys()
        # for i in self.names:
        #     setattr(self, i, RGB(self.rgb_config[i]))
        self.rgbs = { i: RGB(self.rgb_config[i]) for i in self.names }
    
    def __getitem__(self, name):
        return self.rgbs[name]

rgb_config = RGBConfig()
