#include "ros/ros.h"
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <stdlib.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

using namespace std;

class CreateLocationFile {
	private : 
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Publisher pub_pose_;
		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformListener tf_listener_;
		std::string file_name_;
		bool saveLocation( const std::string location_name );

	public : 
		CreateLocationFile();
		void createLocationFile();
};

bool CreateLocationFile::saveLocation( const std::string location_name ) {
	try{
		geometry_msgs::TransformStamped transformStamped;
		tfBuffer_.canTransform("map","base_footprint", ros::Time(0), ros::Duration(0.5));
		transformStamped = tfBuffer_.lookupTransform ("map","base_footprint", ros::Time(0));

		geometry_msgs::Transform transform = transformStamped.transform;
		std::cout << std::endl;
		std::cout << "point_x   : " << fixed << std::setprecision(7) << transform.translation.x << std::endl;
		std::cout << "point_y   : " << fixed << std::setprecision(7) << transform.translation.y << std::endl;
		std::cout << "point_z   : " << fixed << std::setprecision(7) << transform.translation.z << std::endl;
		std::cout << "rotation_x: " << fixed << std::setprecision(7) << transform.rotation.x << std::endl;
		std::cout << "rotation_y: " << fixed << std::setprecision(7) << transform.rotation.y << std::endl;
		std::cout << "rotation_z: " << fixed << std::setprecision(7) << transform.rotation.z << std::endl;
		std::cout << "rotation_w: " << fixed << std::setprecision(7) << transform.rotation.w << std::endl;

		ofstream ofs(file_name_, ios::app);
		if(ofs) {
			ofs << "    - {" <<std::endl;
			ofs << "        location_name: \"" << location_name << "\","<<std::endl;
			ofs << "        frame_id: \"map\","<<std::endl;
			ofs << "        translation_x: " << fixed << std::setprecision(7) << transform.translation.x << "," << std::endl;
			ofs << "        translation_y: " << fixed << std::setprecision(7) << transform.translation.y << "," << std::endl;
			ofs << "        translation_z: " << fixed << std::setprecision(7) << transform.translation.z << "," << std::endl;
			ofs << "        rotation_x: " << fixed << std::setprecision(7) << transform.rotation.x << "," << std::endl;
			ofs << "        rotation_y: " << fixed << std::setprecision(7) << transform.rotation.y << "," << std::endl;
			ofs << "        rotation_z: " << fixed << std::setprecision(7) << transform.rotation.z << "," << std::endl;
			ofs << "        rotation_w: " << fixed << std::setprecision(7) << transform.rotation.w << "," << std::endl;
			ofs << "    }" <<std::endl;
			ofs << std::endl;
			ofs.close();
			std::cout <<  "「" << file_name_ << "　」として保存完了。"  << std::endl;

			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = transform.translation.x;
			pose.pose.position.y = transform.translation.y;
			pose.pose.position.z = transform.translation.z;
			pose.pose.orientation.x = transform.rotation.x;
			pose.pose.orientation.y = transform.rotation.y;
			pose.pose.orientation.z = transform.rotation.z;
			pose.pose.orientation.w = transform.rotation.w;
			pose.header.frame_id = location_name;
			pub_pose_.publish(pose);
		} else {
			ofs.close();
			std::cout <<  file_name_ << "　は作成できませんでした。ｍ(_ _;)ｍ"  << std::endl;
			std::cout << "ファイルのパスを確認して下さい。" << std::endl;
		}	
		return true;
    } catch( tf2::TransformException &ex ){
		ROS_ERROR("%s", ex.what());
		return false;
	}
}

CreateLocationFile::CreateLocationFile( ) : nh_(), pnh_("~"), tfBuffer_(), tf_listener_(tfBuffer_) {
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/location_pose", 1);
}

void CreateLocationFile::createLocationFile() {
	std::string save_location_folder_path = pnh_.param<std::string>("save_location_folder_path", "/map");
	time_t now = time(NULL);
	struct tm *pnow = localtime(&now);
	file_name_ = save_location_folder_path 
				+ "/map_location_" 
				+  std::to_string(pnow->tm_mon + 1) 
				+ "_" + std::to_string(pnow->tm_mday) 
				+ "_" + std::to_string(pnow->tm_hour) 
				+ "_" + std::to_string(pnow->tm_min) 
				+ ".yaml";
	std::cout << "Location File Path : " << file_name_ << std::endl;
	bool is_saved = false;
	std::string location_name;
	std::vector<std::string> location_names;
	ofstream ofs(file_name_, ios::app);
	if(ofs) {
		ofs << "location_pose:" <<std::endl;
	} else {
		ofs.close();
		std::cout <<  file_name_ << "　は作成できませんでした。ｍ(_ _;)ｍ"  << std::endl;
		std::cout << "ファイルのパスを確認して下さい。" << std::endl;
	}
	while(ros::ok()) {
		std::cout <<"========================================" << std::endl;
		std::cout <<"[ 登録地点一覧 ]" << std::endl;
		int i = 1;
		for ( const auto& name : location_names ) std::cout << "    [ " << i++ << " ] : " << name << std::endl; 
		std::cout <<  "\n[ ロボットの現在地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : ";

		std::getline(std::cin, location_name);
		if(location_name =="q") {
			std::cout <<  "\nOK,I'll end...."  << std::endl;					
			ros::Duration(2).sleep();
			exit(EXIT_SUCCESS);
		}else {
			std::cout << "\n現在地点を「" << location_name << "」で保存します。" << std::endl;
			saveLocation( location_name );
			location_names.push_back(location_name);
		}
	}
	return;
}

int main(int argc, char **argv) {
	std::cout <<"========================================" << std::endl;
	ros::init(argc, argv, "create_location_file");
	std::cout << "場所名を入力すると位置座標を保存" << std::endl;
	CreateLocationFile clf;
	clf.createLocationFile();
	ros::spin();
	return 0;
}
