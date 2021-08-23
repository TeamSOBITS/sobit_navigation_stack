#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

class LocationPose {
    public :
        LocationPose ( std::string location_name, geometry_msgs::Pose pose ) 
            :  location_name_(location_name), pose_(pose) {}
        std::string location_name_;
        geometry_msgs::Pose pose_;
};

class DisplayLocationMarker {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_msg_;
        ros::Publisher pub_location_marker_;
        std::vector<LocationPose> location_poses_;

        void callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg );
        visualization_msgs::Marker makeMakerArrow( const geometry_msgs::Pose& pose, int marker_id, const float size );
        visualization_msgs::Marker makeMakerString( const std::string string, const geometry_msgs::Pose& pose, int marker_id, const float size );

    public:
        DisplayLocationMarker( );
        void displayMarker();
};


void DisplayLocationMarker::callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg ) {
    location_poses_.push_back( LocationPose( msg->header.frame_id, msg->pose ) );
    return;
}

visualization_msgs::Marker DisplayLocationMarker::makeMakerArrow( const geometry_msgs::Pose& pose, int marker_id, const float size ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "location_pose";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = size;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(1.0);
    return marker;
}
visualization_msgs::Marker DisplayLocationMarker::makeMakerString( const std::string string, const geometry_msgs::Pose& pose, int marker_id, const float size ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "location_pose";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;
    marker.pose.position.z = marker.pose.position.z + 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.text = string;

    marker.lifetime = ros::Duration(1.0);
    return marker;
}

DisplayLocationMarker::DisplayLocationMarker( ) : nh_(), pnh_("~") {
    sub_msg_ = nh_.subscribe( "/location_pose", 1, &DisplayLocationMarker::callbackMessage, this );
    pub_location_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/location_pose_marker", 1);
}

void DisplayLocationMarker::displayMarker() {
    ros::Rate loop_rate(30);
    
	while(ros::ok()) {
        int marker_id = 0;
        visualization_msgs::MarkerArray marker_array;
        for ( const auto lp : location_poses_ ) {
            marker_array.markers.push_back( makeMakerArrow( lp.pose_, marker_id, 0.5 ) );
            marker_id++;
            marker_array.markers.push_back( makeMakerString( lp.location_name_, lp.pose_, marker_id, 0.5 ) );
            marker_id++;
        }
        pub_location_marker_.publish(marker_array);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return;

}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "display_location_marker");
    DisplayLocationMarker display_location_marker;
    display_location_marker.displayMarker();
    ros::spin();
    return 0;
}
