#include <ros/ros.h>
#include <sobit_navigation_library/sobit_navigation_library.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    SOBITNavigationStack::SOBITNavigationLibrary nav_lib;

    nav_lib.move2Location( "table", false );
    while (ros::ok()) {
        if( !nav_lib.exist_goal_ ) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
        // std::cout << "==================================" << std::endl;
        // std::cout << "status_id : " << nav_lib.status_id_ << std::endl;
        // std::cout << "result : " << nav_lib.result_ << std::endl;
        // std::cout << "==================================" << std::endl;
    }
    // std::cout << "---------------------------------" << std::endl;
    // std::cout << "status_id : " << nav_lib.status_id_ << std::endl;
    // std::cout << "result : " << nav_lib.result_ << std::endl;
    // std::cout << "exist_goal : " << nav_lib.exist_goal_ << std::endl;
    // std::cout << "---------------------------------" << std::endl;
    ros::spin();
}