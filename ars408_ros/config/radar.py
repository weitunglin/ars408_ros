class Radar(object):
    def __init__(self, c):
        self.transform = c["transform"]
        self.can_device = c["can_device"]


class RadarConfig():
    def __init__(self):
        self.radar_config = {
            # "radar_name": {
            #   "transform": [x_trans (m), y_trans (m), rotate (rad)]
            #   "can_device": "can_name"
            # }
            "front_left": {
                "transform": [-1, 0, -0.77],
                "can_device": "can0",
            },
            "front_center": {
                "transform": [0, 0, 0],
                "can_device": "can1",
            },
            "front_right": {
                "transform": [1, 0, 0.77],
                "can_device": "can2",
            },
            "rear_right": {
                "transform": [1, 5, 1.22],
                "can_device": "can3",
            },
            "rear_center": {
                "transform": [0, 5, 0],
                "can_device": "can4",
            },
            "rear_left": {
                "transform": [-1, 5, -1.22],
                "can_device": "can5",
            }
        }
        self.names = self.radar_config.keys()
        self.radars = { i: Radar(self.radar_config[i]) for i in self.names}

    def __getitem__(self, name):
        return self.radars[name]    

radar_config = RadarConfig()

