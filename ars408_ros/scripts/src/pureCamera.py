#! /usr/bin/env python3
# coding=utf-8
import cv2


cam = cv2.VideoCapture(2)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
# cam.set(cv2.CAP_PROP_XI_IMAGE_DATA_BIT_DEPTH, False)

while True:
    ret, cv_image = cam.read()
    cv2.imshow("Img", cv_image)
    cv2.waitKey(1)
