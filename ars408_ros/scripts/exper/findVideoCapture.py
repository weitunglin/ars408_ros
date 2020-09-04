#! /usr/bin/env python3
# coding=utf-8
import cv2


i = 0
while True:
    cam = cv2.VideoCapture(i)
    ret, cv_image = cam.read()
    if ret:
        cv2.imshow("Img", cv_image)
        print(i)
        cv2.waitKey(0)
    i+=1
