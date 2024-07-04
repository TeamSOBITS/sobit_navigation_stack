#include <ros/ros.h>
#include <sobit_navigation_library/sobit_navigation_library.hpp>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    SOBITNavigationStack::SOBITNavigationLibrary nav_lib;
    geometry_msgs::Pose target_position;


    target_position.position.x    =  3.0; /* point_x  */
    target_position.position.y    = -2.0; /* point_y  */
    target_position.position.z    =  0.0; /* point_z  */
    target_position.orientation.x =  0.0; /* quater_x */
    target_position.orientation.y =  0.0; /* quater_y */
    target_position.orientation.z =  0.0; /* quater_z */
    target_position.orientation.w =  1.0; /* quater_w */

    /*      move2Position( geometry_msgs::Pose target_position,  frame_name     , is_wait ); */
    nav_lib.move2Position(                     target_position, "base_footprint", false   );
    // nav_lib.move2Position(                     target_position, "map"           , false   );

    while (ros::ok()) {
        if( !nav_lib.exist_goal_ ) {
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}