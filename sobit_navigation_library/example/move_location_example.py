#!/usr/bin/env python3
import rospy
from sobit_navigation_module import SOBITNavigationLibraryPython
import sys


def test():
    rospy.init_node("navigation_test")
    args = sys.argv
    nav_lib = SOBITNavigationLibraryPython(args[0])

    nav_lib.move2Location( "table", False )
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if not nav_lib.exist_goal_:
            break
        r.sleep()


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException: pass