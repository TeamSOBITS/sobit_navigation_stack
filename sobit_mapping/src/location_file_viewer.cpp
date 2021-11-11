#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <stdlib.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;


class LocationPose {
    public :
        std::string name;
        std::string frame_id;
        geometry_msgs::Pose pose;
};

class LocationFileViewer {
    private :
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber sub_msg_;
        ros::Publisher pub_location_marker_;
        std::vector<LocationPose> location_poses_;
        std::string file_name_;

        visualization_msgs::MarkerArray marker_array_;
        int marker_id_;

        void loadLocationFile();
        void callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg );
        visualization_msgs::Marker makeMakerArrow( const geometry_msgs::Pose& pose, int marker_id, const float size, const bool is_add = false );
        visualization_msgs::Marker makeMakerString( const std::string string, const geometry_msgs::Pose& pose, int marker_id, const float size );

    public :
        LocationFileViewer();
        void viewer();
        void displayMarker();
};

// ロケーションファイルを読み込む関数
void LocationFileViewer::loadLocationFile() {
    XmlRpc::XmlRpcValue pose_val;
    if ( nh_.hasParam("/location_pose") ) ROS_INFO( "[ LocationFileViewer ] Load the location file\n" );
    else {
        ROS_ERROR( "[ LocationFileViewer ] The location file path does not exist.\n" );
        return;
    }

    nh_.getParam("/location_pose", pose_val);
    int pose_num = pose_val.size();
    location_poses_.clear();
    for ( int i = 0; i < pose_num; i++ ) {
        LocationPose pose;
        pose.name = static_cast<std::string>(pose_val[i]["location_name"]); 
        pose.frame_id = static_cast<std::string>(pose_val[i]["frame_id"]); 
        pose.pose.position.x = static_cast<double>(pose_val[i]["translation_x"]); 
        pose.pose.position.y = static_cast<double>(pose_val[i]["translation_y"]); 
        pose.pose.position.z = static_cast<double>(pose_val[i]["translation_z"]); 
        pose.pose.orientation.x = static_cast<double>(pose_val[i]["rotation_x"]); 
        pose.pose.orientation.y = static_cast<double>(pose_val[i]["rotation_y"]); 
        pose.pose.orientation.z = static_cast<double>(pose_val[i]["rotation_z"]); 
        pose.pose.orientation.w = static_cast<double>(pose_val[i]["rotation_w"]); 
        location_poses_.push_back( pose );
        marker_array_.markers.push_back( makeMakerArrow( pose.pose, marker_id_, 0.5 ) );
        marker_id_++;
        marker_array_.markers.push_back( makeMakerString( pose.name, pose.pose, marker_id_, 0.5 ) );
        marker_id_++;
    }
    return;
}

void LocationFileViewer::callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg ) {
    std::cout <<  "\n[ 地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : ";
    std::string location_name;
    std::getline(std::cin, location_name);
    if(location_name =="q") {
        std::cout <<  "\nOK,I'll end...."  << std::endl;					
        ros::Duration(2).sleep();
        exit(EXIT_SUCCESS);
        return;
    }
	std::cout << "\n現在地点を「" << location_name << "」で保存します。" << std::endl;

    std::cout << std::endl;
	std::cout << "transform.getOrigine().x(): "<< fixed << std::setprecision(7) << msg->pose.position.x << std::endl;
	std::cout << "transform.getOrigine().y(): "<< fixed << std::setprecision(7) << msg->pose.position.y << std::endl;
	std::cout << "transform.getOrigine().z(): "<< fixed << std::setprecision(7) << msg->pose.position.z << std::endl;

	std::cout << "transform.getRotation().x(): "<< fixed << std::setprecision(7) << msg->pose.orientation.x << std::endl;
	std::cout << "transform.getRotation().y(): "<< fixed << std::setprecision(7) << msg->pose.orientation.y << std::endl;
	std::cout << "transform.getRotation().z(): "<< fixed << std::setprecision(7) << msg->pose.orientation.z << std::endl;
	std::cout << "transform.getRotation().w(): "<< fixed << std::setprecision(7) << msg->pose.orientation.w << std::endl;

	ofstream ofs(file_name_, ios::app);
	if(ofs) {
		ofs << "    - {" <<std::endl;
		ofs << "        location_name: \"" << location_name << "\","<<std::endl;
		ofs << "        frame_id: \"map\","<<std::endl;
		ofs << fixed << std::setprecision(7) << "        translation_x: " << msg->pose.position.x << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        translation_y: " << msg->pose.position.y << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        translation_z: " << msg->pose.position.z << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        rotation_x: " << msg->pose.orientation.x << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        rotation_y: " << msg->pose.orientation.y << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        rotation_z: " << msg->pose.orientation.z << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "        rotation_w: " << msg->pose.orientation.w << "," << std::endl;
		ofs << fixed << std::setprecision(7) << "    }" <<std::endl;
		ofs << fixed << std::setprecision(7) << std::endl;
		ofs.close();
	
        std::cout <<  "「" << file_name_ << "　」に追記完了。"  << std::endl;
	} else {
		ofs.close();
		std::cout <<  file_name_ << "　は作成できませんでした。ｍ(_ _;)ｍ"  << std::endl;
		std::cout << "ファイルのパスを確認して下さい。" << std::endl;
	}	
    LocationPose pose;
    pose.name = location_name;
    pose.frame_id = msg->header.frame_id;
    pose.pose = msg->pose; 
    location_poses_.push_back( pose );
    marker_array_.markers.push_back( makeMakerArrow( pose.pose, marker_id_, 0.5, true ) );
    marker_id_++;
    marker_array_.markers.push_back( makeMakerString( pose.name, pose.pose, marker_id_, 0.5 ) );
    marker_id_++;
    std::cout <<"========================================" << std::endl;
    std::cout <<"[ 登録地点一覧 ]" << std::endl;
    int i = 1;
    for ( const auto& pose : location_poses_ ) std::cout << "    [ " << i++ << " ] : " << pose.name << std::endl; 
	return ;
}

visualization_msgs::Marker LocationFileViewer::makeMakerArrow( const geometry_msgs::Pose& pose, int marker_id, const float size, const bool is_add ) {
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

    if ( is_add )  {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }

    //marker.lifetime = ros::Duration(1.0);
    return marker;
}

visualization_msgs::Marker LocationFileViewer::makeMakerString( const std::string string, const geometry_msgs::Pose& pose, int marker_id, const float size ) {
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

    //marker.lifetime = ros::Duration(1.0);
    return marker;
}

LocationFileViewer::LocationFileViewer() : nh_(), pnh_("~") {
    loadLocationFile();
    sub_msg_ = nh_.subscribe( "/move_base_simple/goal", 1, &LocationFileViewer::callbackMessage, this );
    pub_location_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/location_pose_marker", 1);
    file_name_ = pnh_.param<std::string>("location_file_path", "/map");
    marker_id_ = 0;
}

void LocationFileViewer::viewer() {
    ros::Rate loop_rate(30);
    std::cout <<"========================================" << std::endl;
    std::cout <<"[ 登録地点一覧 ]" << std::endl;
    int i = 1;
    for ( const auto& pose : location_poses_ ) std::cout << "    [ " << i++ << " ] : " << pose.name << std::endl; 
    while(true) {
        ros::spinOnce();
		loop_rate.sleep();
        pub_location_marker_.publish(marker_array_);
	}
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "location_file_viewer");
    LocationFileViewer location_file_viewer;
    location_file_viewer.viewer();
    
    ros::spin();
    return 0;
}