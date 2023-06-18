#!/usr/bin/env python3
import rospy
import csv
import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt


def main():
    # filename = '/home/sobits/catkin_ws/src/pcl_color.csv'
    # with open(filename, encoding='utf8', newline='') as f:
    #     csvreader = f.read()
    # print(csvreader)
    # print(type(csvreader))
    # dd = np.loadtxt('/home/sobits/catkin_ws/src/sobit_navigation_stack/pcl_color.csv', delimiter=',')

    # ddd = np.reshape(dd,(-1,1280,3))
    ddd = np.zeros((500,500,3))
    # print(ddd)
    # ddd = np.reshape(ddd, (100,100,3))
    for i in range(500):
        for j in range(500):
            ddd[i][j] = [150, 150, 160]
    # ddd = np.array([[[121, 118, 137]]])
    # ddd
    print(type(ddd))
    # print(ddd.shape)

    out_put = Image.fromarray(ddd.astype(np.uint8))
    # out_put = out_put[:,:,::-1]
    print(out_put.mode)


    out_put.save('/home/sobits/catkin_ws/src/sobit_navigation_stack/pic_tem.jpg')
    srcBGR = cv2.imread("/home/sobits/catkin_ws/src/sobit_navigation_stack/pic_tem.jpg")
    # destRGB = cv2.cvtColor(srcBGR, cv2.COLOR_BGR2RGB)

    cv2.imshow("im",srcBGR)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()

    
    
    
    return



if __name__ == '__main__':
    main()