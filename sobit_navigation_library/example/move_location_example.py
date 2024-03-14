#!/usr/bin/env python3
import rospy
from sobit_navigation_module import SOBITNavigationLibraryPython
import sys


def test():
    rospy.init_node("navigation_test")
    args = sys.argv
    r = rospy.Rate(10) # 10hz
    nav_lib = SOBITNavigationLibraryPython(args[0])

    nav_lib.move2Location( "table", False )
    while not rospy.is_shutdown():
        if not nav_lib.exist_goal_:
            break
        r.sleep()
        # print("==================================")
        # print("status_id : ", nav_lib.status_id_)
        # print("result : ", nav_lib.result_)
        # print("==================================")
    # print("---------------------------------")
    # print("status_id : ", nav_lib.status_id_)
    # print("result : ", nav_lib.result_)
    # print("exist_goal : ", nav_lib.exist_goal_)
    # print("---------------------------------")


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException: pass