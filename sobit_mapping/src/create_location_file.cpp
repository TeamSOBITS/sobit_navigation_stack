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
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class CreateLocationFile {
	private : 
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Publisher pub_pose_;
		tf::TransformListener tf_listener_;
		bool first_flag_;
		std::string file_name_;
		bool saveLocation( const std::string location_name );

	public : 
		CreateLocationFile();
		void createLocationFile();
};

bool CreateLocationFile::saveLocation( const std::string location_name ) {
	try{ 
	//tfで変換
	tf::StampedTransform transform;
	if(!tf_listener_.waitForTransform("/map","base_footprint",ros::Time(0),ros::Duration(3.0)) ) {
		std::cout << "位置取得失敗。	tfは出てますか？　/mapと/base_footprintのフレームは繋がっていますか？" << std::endl;
		while(1){ros::Duration(2).sleep();}
	}
	tf_listener_.lookupTransform("/map","base_footprint",ros::Time(0),transform);			

	//現在位置をファイルに出力する．
	std::cout << std::endl;
	std::cout << "transform.getOrigine().x(): "<<transform.getOrigin().x() << std::endl;
	std::cout << "transform.getOrigine().y(): "<<transform.getOrigin().y() << std::endl;
	std::cout << "transform.getOrigine().z(): "<<transform.getOrigin().z() << std::endl;

	std::cout << "transform.getRotation().x(): "<<transform.getRotation().x() << std::endl;
	std::cout << "transform.getRotation().y(): "<<transform.getRotation().y() << std::endl;
	std::cout << "transform.getRotation().z(): "<<transform.getRotation().z() << std::endl;
	std::cout << "transform.getRotation().w(): "<<transform.getRotation().w() << std::endl;

	ofstream ofs(file_name_, ios::app);
	if(ofs) {
		ofs << "    - {" <<std::endl;
		ofs << "        location_name: " << location_name <<std::endl;
		ofs << "        translation_x: " << transform.getOrigin().x() << std::endl;
		ofs << "        translation_y: " << transform.getOrigin().y() << std::endl;
		ofs << "        translation_z: " << transform.getOrigin().z() << std::endl;
		ofs << std::endl;
		ofs << "        rotation_x: " << transform.getRotation().x() << std::endl;
		ofs << "        rotation_y: " << transform.getRotation().y() << std::endl;
		ofs << "        rotation_z: " << transform.getRotation().z() << std::endl;
		ofs << "        rotation_w: " << transform.getRotation().w() << std::endl;
		ofs << "    }" <<std::endl;
		ofs << std::endl;
		ofs.close();
		if(first_flag_ == true)
			std::cout <<  "「" << file_name_ << "　」として保存完了。"  << std::endl;
		else
			std::cout <<  "「" << file_name_ << "　」に追記完了。"  << std::endl;

		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = transform.getOrigin().x();
		pose.pose.position.y = transform.getOrigin().y();
		pose.pose.position.z = transform.getOrigin().z();
		pose.pose.orientation.x = transform.getRotation().x();
		pose.pose.orientation.y = transform.getRotation().y();
		pose.pose.orientation.z = transform.getRotation().z();
		pose.pose.orientation.w = transform.getRotation().w();
		pose.header.frame_id = location_name;
		pub_pose_.publish(pose);
	} else {
		ofs.close();
		std::cout <<  file_name_ << "　は作成できませんでした。ｍ(_ _;)ｍ"  << std::endl;
		std::cout << "ファイルのパスを確認して下さい。" << std::endl;
	}	
	first_flag_ = false;
	return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

CreateLocationFile::CreateLocationFile( ) : nh_(), pnh_("~") {
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/location_pose", 1);
}

void CreateLocationFile::createLocationFile() {
	first_flag_ = true;
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
	while(true) {
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
