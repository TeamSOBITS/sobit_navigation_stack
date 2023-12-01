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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

using namespace std;

class CreateLocationFile {
	private : 
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Publisher pub_pose_;
        ros::Subscriber sub_msg_;
		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformListener tf_listener_;
		std::vector<std::string> location_names;
		std::string file_name_;
		bool use_robot_;
		geometry_msgs::Pose nav_goal_msg_;
		bool saveLocation( const std::string location_name );
		void callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg );
	public : 
		CreateLocationFile();
		void createLocationFile();
};

bool CreateLocationFile::saveLocation( const std::string location_name ) {
	geometry_msgs::Transform transform;
	try{
		if (use_robot_) {
			geometry_msgs::TransformStamped transformStamped;
			tfBuffer_.canTransform("map","base_footprint", ros::Time(0), ros::Duration(0.5));
			transformStamped = tfBuffer_.lookupTransform ("map","base_footprint", ros::Time(0));
			transform = transformStamped.transform;
		} else {
			transform.translation.x = nav_goal_msg_.position.x;
			transform.translation.y = nav_goal_msg_.position.y;
			transform.translation.z = nav_goal_msg_.position.z;
			transform.rotation.x = nav_goal_msg_.orientation.x;
			transform.rotation.y = nav_goal_msg_.orientation.y;
			transform.rotation.z = nav_goal_msg_.orientation.z;
			transform.rotation.w = nav_goal_msg_.orientation.w;
		}
	} catch( tf2::TransformException &ex ) {
		ROS_ERROR("%s", ex.what());
		std::cout << "位置取得失敗。tfは出てますか？mapとbase_footprintのフレームが繋がっていません。" << std::endl;
		std::cout << "しっかりとロボットを接続してからやり直してください。" << std::endl;
		return false;
	}
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
		std::cout <<  "「" << file_name_ << "」として保存完了。"  << std::endl;

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
		std::cout <<  file_name_ << "は作成できませんでした。"  << std::endl;
		std::cout << "ファイルのパスを確認して下さい。" << std::endl;
		return false;
	}
	return true;
}

void CreateLocationFile::callbackMessage( const geometry_msgs::PoseStampedConstPtr &msg ) {
	nav_goal_msg_ = msg->pose;
	std::cout <<"========================================" << std::endl;
	std::cout <<"[ 登録地点一覧 ]" << std::endl;
	int i = 1;
	for ( const auto& name : location_names ) std::cout << "    [ " << i++ << " ] : " << name << std::endl;
	std::cout <<  "\n[ クリックした地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : ";

	std::string location_name;
	std::getline(std::cin, location_name);
	if(location_name =="q") {
		std::cout <<  "\nOK,I'll end...."  << std::endl;					
		ros::Duration(2).sleep();
		exit(EXIT_SUCCESS);
	}else {
		std::cout << "\nクリックした地点を「" << location_name << "」で保存します。" << std::endl;
		if (saveLocation( location_name )) location_names.push_back(location_name);
	}
	std::cout << "地点登録したいところを、2D Nav Goalでクリックしてください。" << std::endl;
}

CreateLocationFile::CreateLocationFile( ) : nh_(), pnh_("~"), tfBuffer_(), tf_listener_(tfBuffer_) {
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/location_pose", 1);
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
	ofstream ofs(file_name_, ios::app);
	if(ofs) {
		ofs << "location_pose:" <<std::endl;
	} else {
		ofs.close();
		std::cout <<  file_name_ << "は作成できませんでした。"  << std::endl;
		std::cout << "ファイルのパスを確認して下さい。" << std::endl;
	}
	use_robot_ = pnh_.param<bool>("use_robot", true);
	if (use_robot_) {
		createLocationFile();
	}
	else {
		sub_msg_ = nh_.subscribe( "/move_base_simple/goal", 1, &CreateLocationFile::callbackMessage, this );
		std::cout << "地点登録したいところを、2D Nav Goalでクリックしてください。" << std::endl;
	}
}

void CreateLocationFile::createLocationFile() {
	while(ros::ok()) {
		std::cout <<"========================================" << std::endl;
		std::cout <<"[ 登録地点一覧 ]" << std::endl;
		int i = 1;
		for ( const auto& name : location_names ) std::cout << "    [ " << i++ << " ] : " << name << std::endl;
		std::cout <<  "\n[ ロボットの現在地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : ";

		std::string location_name;
		std::getline(std::cin, location_name);
		if(location_name =="q") {
			std::cout <<  "\nOK,I'll end...."  << std::endl;					
			ros::Duration(2).sleep();
			exit(EXIT_SUCCESS);
		}else {
			std::cout << "\n現在地点を「" << location_name << "」で保存します。" << std::endl;
			if (saveLocation( location_name )) location_names.push_back(location_name);
		}
	}
	return;
}

int main(int argc, char **argv) {
	std::cout <<"========================================" << std::endl;
	ros::init(argc, argv, "create_location_file");
	std::cout << "場所名を入力すると位置座標を保存" << std::endl;
	CreateLocationFile clf;
	ros::spin();
	return 0;
}
