#!/usr/bin/env python3
import rospy
from sobit_navigation_module import SOBITNavigationLibraryPython
import sys


def test():
    rospy.init_node("navigation_test")
    args = sys.argv
    r = rospy.Rate(10) # 10hz
    nav_lib = SOBITNavigationLibraryPython(args[0])

    ####### move2PositionPy( point_x, point_y, point_z, quater_x, quater_y, quater_z, quater_w, frame_name      , is_wait ) #######
    nav_lib.move2PositionPy( 3.0    , -2.0   , 0.0    , 0.0     , 0.0     , 0.0     ,  1.0    , "base_footprint", False   )
    # nav_lib.move2PositionPy( 0.0    ,  0.0   , 0.0    , 0.0     , 0.0     , 0.0     , -1.0    , "map"           , False   )
    while not rospy.is_shutdown():
        if not nav_lib.exist_goal_:
            break
        r.sleep()


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException: pass