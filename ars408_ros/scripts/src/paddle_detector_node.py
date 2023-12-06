#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from deploy.pipeline.pipeline import Pipeline

class PaddleDetector:
    def __init__(self):
        rospy.init_node('paddle_detector', anonymous=True)

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('/camera/image_color', Image, self.image_callback)

        self.pipeline = Pipeline({}, {})

    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        """
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.float() / 255.0
        img = img.permute(2, 0, 1).unsqueeze(0)
        """

        

if __name__ == "__main__":
    try:
        PaddleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
