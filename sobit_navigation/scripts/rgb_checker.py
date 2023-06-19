#!/usr/bin/env python3
import rospy
import csv
import sys
import cv2
# from cv_bridge import CvBridge
import rospkg
import numpy as np
from PIL import Image


def checker(R, G, B):
    if ((R == None) or (G == None) or (B == None)):
        print("NO SETTING")
        return
    rospack = rospkg.RosPack()
    rospack.list()
    package_path = rospack.get_path("sobit_navigation")
    rgb_array = np.zeros((500,500,3))
    for i in range(500):
        for j in range(500):
            rgb_array[i][j] = [R, G, B] #########
    out_put = Image.fromarray(rgb_array.astype(np.uint8))
    out_put.save(package_path + "/param/rgb_base.jpg")  #####
    image = cv2.imread(package_path + "/param/rgb_base.jpg")#####
    cv2.imshow("rgb_base", image)
    cv2.waitKey(0)
    return

def main():
    rospy.init_node("rgb_checker")
    R = rospy.get_param("/rgb_checker/rgb/R", 200)
    G = rospy.get_param("/rgb_checker/rgb/G", 200)
    B = rospy.get_param("/rgb_checker/rgb/B", 200)
    rospy.loginfo("R = {0}, G = {1}, B = {2}".format(R,G,B))
    checker(R, G, B)
    return


if __name__ == '__main__':
    main()