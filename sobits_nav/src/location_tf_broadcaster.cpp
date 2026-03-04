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

#include <chrono>
using namespace std;
using namespace std::chrono_literals;


class LocationFileViewer : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_location_file_path_;
        rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr server_add_location_;
        rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr server_delete_location_;
        tf2_ros::TransformBroadcaster       dynamic_tfBroadcaster_;
        tf2_ros::StaticTransformBroadcaster static_tfBroadcaster_;
        std::vector<geometry_msgs::msg::TransformStamped> location_poses_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string file_name_;
        bool dynamic_tf = false;

        bool loadLocationFile();
        void callback_file_path(const std_msgs::msg::String::SharedPtr msg);
    public:
        LocationFileViewer();
        bool output_file(geometry_msgs::msg::TransformStamped location_pose, bool reset_flag);
        void callback_add_location(const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request, std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);
        void callback_delete_location(const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request, std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);
        void publishTF();
        void timer_callback();
    };

// ロケーションファイルを読み込む関数
bool LocationFileViewer::loadLocationFile() {
    if (file_name_ == "") return false;
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
        publishTF();
        return true;
    } catch (const YAML::Exception& e) {
        std::cout << "Faild Open the Yaml File..." << std::endl;
        return false;
    }
}

bool LocationFileViewer::output_file(geometry_msgs::msg::TransformStamped location_pose, bool reset_flag) {
    if (file_name_ == "") return false;

    ofstream ofs(file_name_, (reset_flag) ? ios::trunc : ios::app);
    if (ofs) {
        std::cout << std::endl;
        std::cout << "point_x   : " << fixed << std::setprecision(7) << location_pose.transform.translation.x << std::endl;
        std::cout << "point_y   : " << fixed << std::setprecision(7) << location_pose.transform.translation.y << std::endl;
        std::cout << "point_z   : " << fixed << std::setprecision(7) << location_pose.transform.translation.z << std::endl;
        std::cout << "rotation_x: " << fixed << std::setprecision(7) << location_pose.transform.rotation.x << std::endl;
        std::cout << "rotation_y: " << fixed << std::setprecision(7) << location_pose.transform.rotation.y << std::endl;
        std::cout << "rotation_z: " << fixed << std::setprecision(7) << location_pose.transform.rotation.z << std::endl;
        std::cout << "rotation_w: " << fixed << std::setprecision(7) << location_pose.transform.rotation.w << std::endl;

        if (reset_flag) ofs << "location_pose:" << std::endl;
        ofs << "  \"" << location_pose.child_frame_id << "\": " << std::endl;
        ofs << "    frame_id: \"map\"" << std::endl;
        ofs << "    translation: " << std::endl;
        ofs << "      x: " << fixed << std::setprecision(7) << location_pose.transform.translation.x << std::endl;
        ofs << "      y: " << fixed << std::setprecision(7) << location_pose.transform.translation.y << std::endl;
        ofs << "      z: " << fixed << std::setprecision(7) << location_pose.transform.translation.z << std::endl;
        ofs << "    rotation: " << std::endl;
        ofs << "      x: " << fixed << std::setprecision(7) << location_pose.transform.rotation.x << std::endl;
        ofs << "      y: " << fixed << std::setprecision(7) << location_pose.transform.rotation.y << std::endl;
        ofs << "      z: " << fixed << std::setprecision(7) << location_pose.transform.rotation.z << std::endl;
        ofs << "      w: " << fixed << std::setprecision(7) << location_pose.transform.rotation.w << std::endl;
        ofs << "" << std::endl;
        ofs.close();
        std::cout << "Saved in \"" << file_name_ << "\"." << std::endl;
    } else {
        ofs.close();
        std::cout << file_name_ << " could not be created. Check the path of the file." << std::endl;
        return false;
    }
    return true;
}

void LocationFileViewer::callback_file_path(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << msg->data << std::endl;
    file_name_ = msg->data;

    std::ifstream ifs(file_name_);
    bool is_empty = true;
    if (ifs) {
        is_empty = ifs.peek() == std::ifstream::traits_type::eof();
        ifs.close();
    }
    ofstream ofs(file_name_, ios::app);
    if (ofs && is_empty) ofs << "location_pose:" << std::endl;

    loadLocationFile();
}

void LocationFileViewer::callback_add_location(
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response) {
    
    bool there_is_location_ = false;
    for (const auto &location_pose : location_poses_) {
        if (location_pose.child_frame_id == request->pose.header.frame_id) there_is_location_ = true;
    }
    if (!there_is_location_) {
        std::cout << "Save the clicked location with \"" << request->pose.header.frame_id << "\"." << std::endl;

        geometry_msgs::msg::TransformStamped add_location_pose;
        add_location_pose.header.stamp = this->now();
        add_location_pose.header.frame_id = "map";
        add_location_pose.child_frame_id = request->pose.header.frame_id;
        add_location_pose.transform.translation.x = request->pose.pose.pose.position.x;
        add_location_pose.transform.translation.y = request->pose.pose.pose.position.y;
        add_location_pose.transform.translation.z = request->pose.pose.pose.position.z;
        add_location_pose.transform.rotation.x = request->pose.pose.pose.orientation.x;
        add_location_pose.transform.rotation.y = request->pose.pose.pose.orientation.y;
        add_location_pose.transform.rotation.z = request->pose.pose.pose.orientation.z;
        add_location_pose.transform.rotation.w = request->pose.pose.pose.orientation.w;

        output_file(add_location_pose, false);
        location_poses_.push_back(add_location_pose);
        publishTF();
    }
    else std::cout << "The \""  << request->pose.header.frame_id << "\" already exists." << std::endl;
    (void) response;
}

void LocationFileViewer::callback_delete_location(
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response) {
    
    size_t sel = location_poses_.size();
    for (size_t i=0; i<location_poses_.size(); i++) {
        if (location_poses_[i].child_frame_id == request->pose.header.frame_id) sel = i;
    }
    if (sel != location_poses_.size()) {
        std::cout << "Delete the location with \"" << request->pose.header.frame_id << "\"." << std::endl;

        location_poses_.erase(location_poses_.begin() + sel);

        for (size_t i=0; i<location_poses_.size(); i++) output_file(location_poses_[i], i==0);
        publishTF();

        if (location_poses_.size() == 0) {
            ofstream ofs(file_name_, ios::trunc);
            ofs << "location_pose:" << std::endl;
            ofs.close();
        }
    }
    else std::cout << "The \""  << request->pose.header.frame_id << "\" is not exists." << std::endl;
    (void) response;
}


void LocationFileViewer::publishTF() {
    if (dynamic_tf) return;
    for (auto& pose : location_poses_) {
        pose.header.stamp = this->now();
        static_tfBroadcaster_.sendTransform(pose);
    }
}


void LocationFileViewer::timer_callback() {
    for (auto& pose : location_poses_) {
        pose.header.stamp = this->now();
        dynamic_tfBroadcaster_.sendTransform(pose);
    }
}


LocationFileViewer::LocationFileViewer() : Node("location_file_viewer"), dynamic_tfBroadcaster_(this), static_tfBroadcaster_(this) {
    sub_location_file_path_ = this->create_subscription<std_msgs::msg::String>("/location_file_path", 1, std::bind(&LocationFileViewer::callback_file_path, this, std::placeholders::_1));
    server_add_location_ = this->create_service<nav2_msgs::srv::SetInitialPose>("/add_location", std::bind(&LocationFileViewer::callback_add_location, this, std::placeholders::_1, std::placeholders::_2));
    server_delete_location_ = this->create_service<nav2_msgs::srv::SetInitialPose>("/delete_location", std::bind(&LocationFileViewer::callback_delete_location, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter<std::string>("location_file_path", "");
    this->get_parameter("location_file_path", file_name_);

    if (file_name_ == "") dynamic_tf = true;
    else loadLocationFile();

    if (dynamic_tf) timer_ = this->create_wall_timer(100ms, std::bind(&LocationFileViewer::timer_callback, this));
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationFileViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}