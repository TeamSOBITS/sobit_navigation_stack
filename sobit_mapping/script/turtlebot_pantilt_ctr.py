#!/usr/bin/env python3
import rospy
from sobit_education_module import SobitEducationController
from sobit_education_module import Joint
import sys
def pro_pantilt_ctr():
    rospy.init_node('pro_pantilt_ctr')
    r = rospy.Rate(1) # 10hz
    ang = rospy.get_param( rospy.get_name() + '/tilt_rad', 0.0 )
    args = sys.argv
    edu_pantilt_ctr = SobitEducationController(args[0]) # args[0] : C++上でros::init()を行うための引数
    # カメラパンチルトを動かす
    edu_pantilt_ctr.moveXtionPanTilt( 0.0, ang, 0.5, False )
    r.sleep()
if __name__ == '__main__':
    try:
        pro_pantilt_ctr()
    except rospy.ROSInterruptException: pass
