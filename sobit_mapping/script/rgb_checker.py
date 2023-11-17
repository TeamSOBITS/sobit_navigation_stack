#!/usr/bin/env python3
import rospy
import yaml
import csv
import sys
import cv2
import rospkg
import numpy as np
from PIL import Image


def checker():
    rospy.init_node("rgb_checker")
    rospack = rospkg.RosPack()
    with open(rospack.get_path("sobit_navigation") + "/param/floor_color/rgb_base.yaml", "r") as yml:
        rgb_base = yaml.safe_load(yml)
    R = rgb_base["R"]
    G = rgb_base["G"]
    B = rgb_base["B"]
    if ((R == None) or (G == None) or (B == None)):
        print("NO SETTING")
        return
    rgb_array = np.zeros((500,500,3))
    for i in range(500):
        for j in range(500):
            rgb_array[i][j] = [R, G, B]
    out_put = Image.fromarray(rgb_array.astype(np.uint8))
    out_put.save(rospack.get_path("sobit_navigation") + "/param/floor_color/rgb_base.jpg")
    image = cv2.imread(rospack.get_path("sobit_navigation") + "/param/floor_color/rgb_base.jpg")
    cv2.imshow("rgb_base", image)
    cv2.waitKey(0)
    rospy.spin()
    return


if __name__ == '__main__':
    checker()
    # main()