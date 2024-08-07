#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <vector>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>


using namespace std;
using namespace std::chrono_literals;

class CreateLocationFile : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_msg_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::string> location_names;
    std::string file_name_;
    bool use_robot_;
    geometry_msgs::msg::Pose nav_goal_msg_;
    bool saveLocation(const std::string &location_name);
    void callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

public:
    CreateLocationFile();
    void createLocationFile();
};

bool CreateLocationFile::saveLocation(const std::string &location_name) {
    geometry_msgs::msg::Transform transform;
    try {
        if (use_robot_) {
            geometry_msgs::msg::TransformStamped transformStamped;
            if (tfBuffer_->canTransform("map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5))) {
                transformStamped = tfBuffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                transform = transformStamped.transform;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Transform failed");
                std::cout << "位置取得失敗。tfは出ていますか？mapとbase_footprintのフレームが繋がっていません。" << std::endl;
                std::cout << "しっかりとロボットを接続してからやり直してください。" << std::endl;
                return false;
            }
        } else {
            transform.translation.x = nav_goal_msg_.position.x;
            transform.translation.y = nav_goal_msg_.position.y;
            transform.translation.z = nav_goal_msg_.position.z;
            transform.rotation.x = nav_goal_msg_.orientation.x;
            transform.rotation.y = nav_goal_msg_.orientation.y;
            transform.rotation.z = nav_goal_msg_.orientation.z;
            transform.rotation.w = nav_goal_msg_.orientation.w;
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
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
    if (ofs) {
        ofs << "    - {" << std::endl;
        ofs << "        location_name: \"" << location_name << "\"," << std::endl;
        ofs << "        frame_id: \"map\"," << std::endl;
        ofs << "        translation_x: " << fixed << std::setprecision(7) << transform.translation.x << "," << std::endl;
        ofs << "        translation_y: " << fixed << std::setprecision(7) << transform.translation.y << "," << std::endl;
        ofs << "        translation_z: " << fixed << std::setprecision(7) << transform.translation.z << "," << std::endl;
        ofs << "        rotation_x: " << fixed << std::setprecision(7) << transform.rotation.x << "," << std::endl;
        ofs << "        rotation_y: " << fixed << std::setprecision(7) << transform.rotation.y << "," << std::endl;
        ofs << "        rotation_z: " << fixed << std::setprecision(7) << transform.rotation.z << "," << std::endl;
        ofs << "        rotation_w: " << fixed << std::setprecision(7) << transform.rotation.w << "," << std::endl;
        ofs << "    }" << std::endl;
        ofs << std::endl;
        ofs.close();
        std::cout << "「" << file_name_ << "」として保存完了。" << std::endl;

        auto pose = geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = transform.translation.x;
        pose.pose.position.y = transform.translation.y;
        pose.pose.position.z = transform.translation.z;
        pose.pose.orientation.x = transform.rotation.x;
        pose.pose.orientation.y = transform.rotation.y;
        pose.pose.orientation.z = transform.rotation.z;
        pose.pose.orientation.w = transform.rotation.w;
        pose.header.frame_id = location_name;
        pub_pose_->publish(pose);
    } else {
        ofs.close();
        std::cout << file_name_ << "は作成できませんでした。" << std::endl;
        std::cout << "ファイルのパスを確認して下さい。" << std::endl;
        return false;
    }
    return true;
}

void CreateLocationFile::callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    nav_goal_msg_ = msg->pose;
    std::cout << "========================================" << std::endl;
    std::cout << "[ 登録地点一覧 ]" << std::endl;
    int i = 1;
    for (const auto &name : location_names)std::cout << "    [ " << i++ << " ] : " << name << std::endl;
    std::cout << "\n[ クリックした地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : "<< std::endl;

    std::string location_name;
    std::getline(std::cin, location_name);

    if (location_name == "q") {
        std::cout << "\nOK, I'll end...." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(2));
        exit(EXIT_SUCCESS);
    } else {
        std::cout << "\nクリックした地点を「" << location_name << "」で保存します。" << std::endl;
        if (saveLocation(location_name))
            location_names.push_back(location_name);
    }
    std::cout << "地点登録したいところを、2D Nav Goalでクリックしてください。" << std::endl;
}

CreateLocationFile::CreateLocationFile() : Node("create_location_file"), tfBuffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_)) {
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/location_pose", 10);
    std::string save_location_folder_path = this->declare_parameter<std::string>("save_location_folder_path", "/home/sobits/colcon_ws/src/sobit_navigation_stack/location/");
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm *parts = std::localtime(&now_c);
    file_name_ = save_location_folder_path + "/map_location_" + std::to_string(parts->tm_mon + 1) + "_" + std::to_string(parts->tm_mday) + "_" + std::to_string(parts->tm_hour) + "_" + std::to_string(parts->tm_min) + ".yaml";
    std::cout << "Location File Path : " << file_name_ << std::endl;
    ofstream ofs(file_name_, ios::app);
    if (ofs) {
        ofs << "location_pose:" << std::endl;
    } else {
        ofs.close();
        std::cout << file_name_ << "は作成できませんでした。" << std::endl;
        std::cout << "ファイルのパスを確認して下さい。" << std::endl;
    }
    use_robot_ = this->declare_parameter<bool>("use_robot", false);
    if (use_robot_) {
        createLocationFile();
    } else {
        sub_msg_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose2", 10, std::bind(&CreateLocationFile::callbackMessage, this, std::placeholders::_1));
        std::cout << "地点登録したいところを、2D Nav Goalでクリックしてください。" << std::endl;
    }
}

void CreateLocationFile::createLocationFile() {
    while (rclcpp::ok()) {
        std::cout << "========================================" << std::endl;
        std::cout << "[ 登録地点一覧 ]" << std::endl;
        int i = 1;
        for (const auto &name : location_names)
            std::cout << "    [ " << i++ << " ] : " << name << std::endl;
        std::cout << "\n[ ロボットの現在地点を登録 ] \n場所名を入力してください。「q」で終了。\n Location Name : "<< std::endl;

        std::string location_name;
        std::getline(std::cin, location_name);
        if (location_name == "q") {
            std::cout << "\nOK, I'll end...." << std::endl;
            rclcpp::sleep_for(std::chrono::seconds(2));
            exit(EXIT_SUCCESS);
        } else {
            std::cout << "\n現在地点を「" << location_name << "」で保存します。" << std::endl;
            if (saveLocation(location_name))
                location_names.push_back(location_name);
        }
    }
}

int main(int argc, char **argv) {
    std::cout << "========================================" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "場所名を入力すると位置座標を保存" << std::endl;
    auto node = std::make_shared<CreateLocationFile>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}