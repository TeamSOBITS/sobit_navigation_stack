#include <ros/ros.h>
#include <sobit_navigation_library/sobit_navigation_library.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    SOBITNavigationStack::SOBITNavigationLibrary nav_lib;

    /*      move2Position( point_x, point_y, point_z, quater_x, quater_y, quater_z, quater_w, frame_name      , is_wait ) */
    nav_lib.move2Position( 3.0    , -2.0   , 0.0    , 0.0     , 0.0     , 0.0     ,  1.0    , "base_footprint", false   );
    // nav_lib.move2Position( 0.0    ,  0.0   , 0.0    , 0.0     , 0.0     , 0.0     , -1.0    , "map"           , false   );
    while (ros::ok()) {
        if( !nav_lib.exist_goal_ ) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}