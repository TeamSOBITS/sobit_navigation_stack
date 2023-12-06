#! /usr/bin/env python3
#coding:utf-8

import rospy
from sobit_pro_module import SobitProJointController
from sobit_pro_module import Joint
import sys
import math

def pantilt_controll():
    args = sys.argv
    pro_joint_ctr = SobitProJointController(args[0]) # args[0] : C++上でros::init()を行うための引数
    pan_ang = rospy.get_param( rospy.get_name() + '/pan_angle_deg', 0.0 ) * math.pi / 180.0
    tilt_ang = rospy.get_param( rospy.get_name() + '/tilt_angle_deg', 0.0 ) * math.pi / 180.0
    # カメラパンチルトを動かす
    rospy.loginfo('move PanTilt -> pan angle = %.3f [deg]\ttilt angle = %.3f [deg]\n', pan_ang * 180/math.pi, tilt_ang * 180/math.pi)
    pro_joint_ctr.moveHeadPanTilt( pan_ang, tilt_ang, 2.0 )

if __name__ == '__main__':
    try:
        rospy.init_node('sobit_pro_pantilt_controll')
        pantilt_controll()
    except rospy.ROSInterruptException: pass
