class FusionConfig(object):
    def __init__(self, rgb_name, radar_name):
        self.rgb_name = rgb_name
        self.radar_name = radar_name

class DefaultConfig(object):
    frame_rate = 30

    use_gui = True
    use_cuda = True
    recording = False
    use_calib = True
    use_yolo = True
    use_yolo_half = True
    use_yolo_image = True
    use_radar_image = True

    sensor_fusion = [FusionConfig(rgb_name=i, radar_name=i) for i in ["front_left", "front_center", "front_right", "rear_right", "rear_center", "rear_left"]]

default_config = DefaultConfig()

# config.radar["radar_name"]
# config.rgb["radar_name"]
# config.default.use_gui
