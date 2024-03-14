#!/usr/bin/env python3
import rospy
from sobit_navigation_module import SOBITNavigationLibraryPython
import sys


def test():
    rospy.init_node("navigation_test")
    args = sys.argv
    nav_lib = SOBITNavigationLibraryPython(args[0])

    nav_lib.cancelMoving()
    nav_lib.clearCostmaps()

    #####   estimatePoseFromLocation( location_name )
    nav_lib.estimatePoseFromLocation( "table"       )

    #####   addLocationPosePy( location_name, frame_name, point_x, point_y, point_z, quater_x, quater_y, quater_z, quater_w)
    nav_lib.addLocationPosePy( "add_point"  , "map"     , 1.0    , 0.5    , 0.0    , 0.0     , 0.0     , 0.0     , 1.0     )


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException: pass