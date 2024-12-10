#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;


class LocationFileViewer : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_location_file_path_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        std::vector<geometry_msgs::msg::TransformStamped> location_poses_;

        std::string file_name_;

        void loadLocationFile();
        void callbackMessage(const std_msgs::msg::String::SharedPtr msg);
    public:
        LocationFileViewer();
        void viewer();
        void displayMarker();
};

// ロケーションファイルを読み込む関数
void LocationFileViewer::loadLocationFile() {
    try {
        YAML::Node config = YAML::LoadFile(file_name_);
        location_poses_.clear();
        auto location_poses = config["location_pose"];

        for (const auto location_pose: location_poses) {
            std::cout << location_pose.first.as<std::string>() << std::endl;
            geometry_msgs::msg::TransformStamped pose;
            pose.child_frame_id = location_pose.first.as<std::string>();
            pose.header.frame_id = location_poses[pose.child_frame_id]["frame_id"].as<std::string>();
            pose.header.stamp = this->now();
            pose.transform.translation.x = location_poses[pose.child_frame_id]["translation"]["x"].as<float>();
            pose.transform.translation.y = location_poses[pose.child_frame_id]["translation"]["y"].as<float>();
            pose.transform.translation.z = location_poses[pose.child_frame_id]["translation"]["z"].as<float>();
            pose.transform.rotation.x = location_poses[pose.child_frame_id]["rotation"]["x"].as<float>();
            pose.transform.rotation.y = location_poses[pose.child_frame_id]["rotation"]["y"].as<float>();
            pose.transform.rotation.z = location_poses[pose.child_frame_id]["rotation"]["z"].as<float>();
            pose.transform.rotation.w = location_poses[pose.child_frame_id]["rotation"]["w"].as<float>();
            location_poses_.push_back(pose);
        }
    } catch (const YAML::Exception& e) {
        std::cout << "Faild" << std::endl;
    }
}

void LocationFileViewer::callbackMessage(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << msg->data << std::endl;
    file_name_ = msg->data;
    loadLocationFile();
}


LocationFileViewer::LocationFileViewer() : Node("location_file_viewer"), tfBroadcaster_(this){
    this->declare_parameter<std::string>("location_file_path", "");
    if (this->get_parameter("location_file_path", file_name_)) loadLocationFile();
    sub_location_file_path_ = this->create_subscription<std_msgs::msg::String>("/location_file_path", 1, std::bind(&LocationFileViewer::callbackMessage, this, std::placeholders::_1));
    viewer();
}

void LocationFileViewer::viewer() {
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        for (auto& pose : location_poses_) {
            pose.header.stamp = this->now();
            tfBroadcaster_.sendTransform(pose);
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationFileViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}