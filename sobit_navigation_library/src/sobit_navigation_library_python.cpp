#include <sobit_navigation_library/sobit_navigation_library_python.hpp>

using namespace SOBITNavigationStack;

SOBITNavigationLibraryPython::SOBITNavigationLibraryPython( const std::string &name ) : SOBITNavigationLibrary( name ) {

}

// 移動したい位置に移動する(Pybind用)
bool SOBITNavigationLibraryPython:: move2PositionPy(
    const double x,
    const double y,
    const double z,
    const double qx,
    const double qy,
    const double qz,
    const double qw,
    const std::string& frame_id,
    const bool is_wait ) {
    geometry_msgs::Pose target_position;
    target_position.position.x = x;
    target_position.position.y = y;
    target_position.position.z = z;
    target_position.orientation.x = qx; 
    target_position.orientation.y = qy;
    target_position.orientation.z = qz;
    target_position.orientation.w = qw;
    move2Position( target_position, frame_id, is_wait );
    return true;
}

// ロケーションポジションの追加(Pybind用)
void SOBITNavigationLibraryPython::addLocationPosePy(
    const std::string& name,
    const std::string& frame_id,
    const double x,
    const double y,
    const double z,
    const double qx,
    const double qy,
    const double qz,
    const double qw ) {
        if ( is_output_ ) ROS_INFO( "[ SOBITNavigationLibrary::addLocationPosePy ] Add a location position\n" );
        LocationPose pose;
        geometry_msgs::Pose target_position;
        pose.name = name;
        pose.frame_id = frame_id;    
        target_position.position.x = x;
        target_position.position.y = y;
        target_position.position.z = z;
        target_position.orientation.x = qx; 
        target_position.orientation.y = qy;
        target_position.orientation.z = qz;
        target_position.orientation.w = qw;
        pose.pose = target_position;
        location_poses_.push_back( pose );
        return;
    }


