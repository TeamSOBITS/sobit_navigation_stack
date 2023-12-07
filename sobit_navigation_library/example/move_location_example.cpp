#include <ros/ros.h>
#include <sobit_navigation_library/sobit_navigation_library.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_test");
    ros::NodeHandle nh;

    SOBITNavigationStack::SOBITNavigationLibrary nav_lib;
    nav_lib.move2Location( "table", false );

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if( !nav_lib.exist_goal_ ) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}