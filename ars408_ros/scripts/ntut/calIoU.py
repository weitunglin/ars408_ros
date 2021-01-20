#! /usr/bin/env python3
# coding=utf-8

import os, sys
import cv2
import argparse
import math
import yaml

with open(os.path.expanduser("~") + "/catkin_ws/src/ARS408_ros/ars408_ros/config/config.yaml", 'r') as stream:
    try:
        config = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

size_Dual = config['size_Dual']

def bb_intersection_over_union(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    
    iou = interArea / float(boxAArea + boxBArea - interArea)

    return iou

if __name__ == "__main__":
    count = 1
    module = 10
    total = total2 = 0
    correctcount = correctcount2 = 0
    fpcount = fpcount2 = 0
    filename = "costco3"
    while count < 2400:
        if count % module == 0:
            f = open(os.path.join('/home/balin/Downloads/北科/costco_label', "Thermal_" + filename + "_{0:06}.txt".format(count)), 'r')
            f2 = open(os.path.join('/home/balin/Downloads/北科/costco_output_label', filename + "_{0:07}.txt".format(count)), 'r')
            f3 = open(os.path.join('/home/balin/Downloads/北科/costco_output_radar', filename + "_{0:07}.txt".format(count)), 'r')
            line = f.readline()
            line2 = f2.readline()
            line3 = f3.readline()
            
            boxAs = []
            while line:
                c, x, y, w, h = line.split(" ")
                x, y, w, h = float(x), float(y), float(w), float(h)
                boxAs.append([int((x - w / 2) * size_Dual[0]), int((y - h / 2) * size_Dual[1]), int((x + w / 2) * size_Dual[0]), int((y + float(h) / 2) * size_Dual[1])])
                line = f.readline()

            boxBs = []
            while line2:
                c, x, y, w, h, s = line2.split(" ")
                x, y, w, h, s = float(x), float(y), float(w), float(h), float(s)
                boxBs.append([int((x - w / 2) * size_Dual[0]), int((y - h / 2) * size_Dual[1]), int((x + w / 2) * size_Dual[0]), int((y + float(h) / 2) * size_Dual[1]), s])
                line2 = f2.readline()

            radar = []
            while line3:
                x, y = line3.split(" ")
                x, y = int(x), int(y)
                radar.append([x, y])
                line3 = f3.readline()

            boxCs = []
            scoreScale = 800
            for i in boxBs:
                leftTop = (i[0], i[1])
                rightBut = (i[2], i[3])

                scoreDist = 99999
                for radarpoint in radar:
                    x = ((leftTop[0] + rightBut[0]) / 2 - radarpoint[0]) ** 2
                    y = ((leftTop[1] + rightBut[1]) / 2 - radarpoint[1]) ** 2
                    scoreDist = min(scoreDist, math.sqrt(x+y))

                i[4] = (i[4] - scoreDist / scoreScale) if scoreDist != 99999 and (leftTop[1] + rightBut[1]) > 640 else i[4]
                if i[4] >= 0.25:
                    boxCs.append(i)

            for i in boxBs:
                total += 1
                FP = 1
                for j in boxAs:
                    if bb_intersection_over_union(i,j) >= 0.5:
                        correctcount += 1
                        FP = 0
                        break
                if FP:
                    fpcount += 1
            print("total:", total, "corr:", correctcount, "fpcount:", fpcount, "FP:", (fpcount / total) * 100)

            for i in boxCs:
                total2 += 1
                FP = 1
                for j in boxAs:
                    if bb_intersection_over_union(i,j) >= 0.5:
                        correctcount2 += 1
                        FP = 0
                        break
                if FP:
                    fpcount2 += 1
            print("total2:", total2, "corr2:", correctcount2, "fpcount2:", fpcount2, "FP:", (fpcount2 / total) * 100)

        count += 1