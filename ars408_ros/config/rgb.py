class RGB(object):
    def __init__(self, c):
        self.size = c["size"]
        self.port = c["port"]
        self.codec = c["codec"]
        self.frame_rate = c["frame_rate"]
        # self.K = c["K"]
        # self.D = c["D"]


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
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "front_center": {
                "size": [1280, 712],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "front_right": {
                "size": [1280, 712],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "rear_right": {
                "size": [1280, 712],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "rear_center": {
                "size": [1280, 712],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "rear_left": {
                "size": [1280, 712],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            },
            "thermal": {
                "size": [640, 512],
                "port": "/dev/video0",
                "codec": "MJPG",
                "frame_rate": 30
            }
        }
    
        self.names = self.rgb_config.keys()
        # for i in self.names:
        #     setattr(self, i, RGB(self.rgb_config[i]))
        self.rgbs = { i: RGB(self.rgb_config[i]) for i in self.names }
    
    def __getitem__(self, name):
        return self.rgbs[name]

rgb_config = RGBConfig()
