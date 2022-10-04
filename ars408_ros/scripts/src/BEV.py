#! /usr/bin/env python3
# coding=utf-8
import cv2
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BEV():
    """
    Creates bird-eye-view of front and rear cameras

    Subscribe:
        /rgb/front_center/calib_image
        /rgb/rear_center/calib_image
    
    Publish:
        /rgb/bev_image
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.sub_list = [
            message_filters.Subscriber("/rgb/front_center/calib_image", Image),
            message_filters.Subscriber("/rgb/rear_center/calib_image", Image)]
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(self.sub_list,
            queue_size=1, slop=0.2)
        self.synchronizer.registerCallback(self.callback)

        self.bev_roi = (slice(450, 712), slice(20, 1260))
        self.image_width, self.image_height = 1260-20, 712-450
        self.offset_x = 65

    def msg_to_image(self, msg: Image) -> cv2.Mat:
        return self.bridge.imgmsg_to_cv2(msg)

    def get_bev_image(self, img: cv2.Mat) -> cv2.Mat:
        src = np.array([[0, self.image_height], [self.image_width, self.image_height]
            , [0, 0], [self.image_width, 0]], dtype=np.float32)
        dst = np.float32([[self.image_width / 2 - self.offset_x, self.image_height],
            [self.image_width / 2 + self.offset_x, self.image_height],
            [0, 0], [self.image_width, 0]])
        M = cv2.getPerspectiveTransform(src, dst)

        img = img[self.bev_roi]
        warped_img = cv2.warpPerspective(img, M, (self.image_width, self.image_height))
        return warped_img

    def callback(self, front_center, rear_center):
        source = [front_center, rear_center]
        rotate = [0, 180]
        result = cv2.Mat((1280, 712))

        c = self.msg_to_image(front_center)
        cv2.imwrite("c.png", c)

        rc = self.msg_to_image(rear_center)
        cv2.imwrite("rc.png", rc)
        # for i in source:
        #     image = self.msg_to_image(i)
        #     bev_image = self.get_bev_image(image)
        pass

def main():
    rospy.init_node("BEV")

    bev = BEV()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInitException:
        pass
