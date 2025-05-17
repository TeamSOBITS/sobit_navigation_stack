#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace std::chrono_literals;


class LocationFileViewer : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_location_file_path_;
        rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr client;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        std::vector<geometry_msgs::msg::TransformStamped> location_poses_;

        double initial_x_;
        double initial_y_;
        double initial_yaw_;

        std::string file_name_;

        bool initial_command_;
        bool create_location_file_;

        void loadLocationFile();
        void initialPoseSet();
        void callbackMessage(const std_msgs::msg::String::SharedPtr msg);
    public:
        LocationFileViewer();
        // void viewer();
        // void displayMarker();
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
        std::cout << "Faild Open the Yaml File..." << std::endl;
    }
}

void LocationFileViewer::initialPoseSet() {
    auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
    request->pose.header.frame_id = "map";
    request->pose.pose.pose.position.x = initial_x_;
    request->pose.pose.pose.position.y = initial_y_;
    geometry_msgs::msg::Quaternion qur;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = 0.0;
    rpy.z = initial_yaw_;
    tf2::Quaternion quat_tf;

    quat_tf.setRPY(rpy.x, rpy.y, rpy.z);
    qur = tf2::toMsg(quat_tf);
    request->pose.pose.pose.orientation.w = qur.w;
    request->pose.pose.pose.orientation.x = qur.x;
    request->pose.pose.pose.orientation.y = qur.y;
    request->pose.pose.pose.orientation.z = qur.z;

    while (!client->wait_for_service(10s)) {
        if (!rclcpp::ok()) return;
    }
    auto result = client->async_send_request(request);
}

void LocationFileViewer::callbackMessage(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << msg->data << std::endl;
    file_name_ = msg->data;
    loadLocationFile();
}


LocationFileViewer::LocationFileViewer() : Node("location_file_viewer"), tfBroadcaster_(this){
    // sub_location_file_path_ = this->create_subscription<std_msgs::msg::String>("/location_file_path", 1, std::bind(&LocationFileViewer::callbackMessage, this, std::placeholders::_1));
    client = this->create_client<nav2_msgs::srv::SetInitialPose>("/set_initial_pose");
    this->declare_parameter<double>("initial_x", 0.0);
    this->declare_parameter<double>("initial_y", 0.0);
    this->declare_parameter<double>("initial_yaw", 0.0);
    this->declare_parameter<std::string>("location_file_path", "");
    this->declare_parameter<bool>("initial_command", true);
    this->declare_parameter<bool>("create_location_file", false);
    this->get_parameter("initial_x", initial_x_);
    this->get_parameter("initial_y", initial_y_);
    this->get_parameter("initial_yaw", initial_yaw_);
    this->get_parameter("location_file_path", file_name_);
    this->get_parameter("initial_command", initial_command_);
    this->get_parameter("create_location_file", create_location_file_);

    if (initial_command_) initialPoseSet();
    if (create_location_file_) sub_location_file_path_ = this->create_subscription<std_msgs::msg::String>("/location_file_path", 1, std::bind(&LocationFileViewer::callbackMessage, this, std::placeholders::_1));
    else loadLocationFile();

    while (rclcpp::ok()) {
        // if (create_location_file_) rclcpp::spin_some(this->get_node_base_interface());
        rclcpp::spin_some(this->get_node_base_interface());
        for (auto& pose : location_poses_) {
            pose.header.stamp = this->now();
            tfBroadcaster_.sendTransform(pose);
        }
    }
    // if (!create_location_file_) rclcpp::spin_some(this->get_node_base_interface());
    // for (auto& pose : location_poses_) {
    //     pose.header.stamp = this->now();
    //     tfBroadcaster_.sendTransform(pose);
    // }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationFileViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






























// #include "rclcpp/rclcpp.hpp"
// #include "example_interfaces/srv/add_two_ints.hpp"

// #include <chrono>
// #include <cstdlib>
// #include <memory>

// using namespace std::chrono_literals;

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   if (argc != 3) {
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
//       return 1;
//   }

//   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
//   rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
//     node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

//   auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
//   request->a = atoll(argv[1]);
//   request->b = atoll(argv[2]);

//   while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       return 0;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   auto result = client->async_send_request(request);
//   // Wait for the result.
//   if (rclcpp::spin_until_future_complete(node, result) ==
//     rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//   }

//   rclcpp::shutdown();
//   return 0;
// }