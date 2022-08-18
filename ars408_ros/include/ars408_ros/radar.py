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
                "transform": [-1, 0, -1.57],
                "can_device": "can3",
            },
            "front_center": {
                "transform": [0, 0, 0],
                "can_device": "can5",
            },
            "front_right": {
                "transform": [1, 0, 1.57],
                "can_device": "can4",
            },
            "rear_right": {
                "transform": [1, -5, 1.57],
                "can_device": "can0",
            },
            "rear_center": {
                "transform": [0, -5, 3.14],
                "can_device": "can1",
            },
            "rear_left": {
                "transform": [-1, -5, -1.57],
                "can_device": "can2",
            }
        }
        self.names = self.radar_config.keys()
        self.radars = { i: Radar(self.radar_config[i]) for i in self.names}

    def __getitem__(self, name):
        return self.radars[name]    

radar_config = RadarConfig()

# /radar/front_left/received_messages /radar/front_center/received_messages /radar/front_right/received_messages /radar/rear_left/received_messages /radar/rear_center/received_messages /radar/rear_right/received_messages /rgb/front_left/original_image /rgb/front_center/original_image /rgb/front_right/original_image /rgb/rear_left/original_image /rgb/rear_center/original_image /rgb/rear_right/original_image /rgb/thermal/original_image
