#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
using namespace std;
using namespace std::chrono_literals;


class LocationFileViewer : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_location_file_path_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
        rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr server_add_location_;
        rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr server_delete_location_;
        tf2_ros::TransformBroadcaster       dynamic_tfBroadcaster_;
        tf2_ros::StaticTransformBroadcaster static_tfBroadcaster_;
        std::vector<geometry_msgs::msg::TransformStamped> location_poses_;
        visualization_msgs::msg::MarkerArray marker_array_;
        visualization_msgs::msg::Marker template_marker_, template_text_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string file_name_;
        bool dynamic_tf = false;
        geometry_msgs::msg::Pose goal_pose_;

        bool loadLocationFile();
        void callback_file_path(const std_msgs::msg::String::SharedPtr msg);
    public:
        LocationFileViewer();
        bool output_file(geometry_msgs::msg::TransformStamped location_pose, bool reset_flag);
        void callback_add_location(const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request, std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);
        void callback_delete_location(const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request, std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);
        void publishTF();
        void callback_global_path(const nav_msgs::msg::Path::SharedPtr msg);
        void timer_callback();
    };

// Read location file
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


void LocationFileViewer::callback_global_path(const nav_msgs::msg::Path::SharedPtr msg) {
    // for (int i=0; i < msg->poses.size(); i++) {}
    if (msg->poses.size() == 0) return;
    goal_pose_ = msg->poses[msg->poses.size()-1].pose;
}


void LocationFileViewer::timer_callback() {
    marker_array_.markers.resize(location_poses_.size()*2);

    for (size_t i = 0; i < location_poses_.size(); ++i) {
        auto& pose = location_poses_[i];
        auto& marker = marker_array_.markers[i] = template_marker_;
        auto& text = marker_array_.markers[location_poses_.size() + i] = template_text_;


        if (dynamic_tf) {
            pose.header.stamp = this->now();
            dynamic_tfBroadcaster_.sendTransform(pose);
        }

        marker.header = pose.header;
        marker.header.stamp = this->now();
        marker.ns = pose.child_frame_id;
        marker.id = i;

        marker.pose.position.x = pose.transform.translation.x;
        marker.pose.position.y = pose.transform.translation.y;
        marker.pose.position.z = pose.transform.translation.z;
        marker.pose.orientation = pose.transform.rotation;

        text.header = pose.header;
        text.header.stamp = this->now();
        text.ns = pose.child_frame_id + "_label";
        text.id = i;
        text.pose = marker.pose;
        text.pose.position.z += 0.3;
        text.text = pose.child_frame_id;

        // color addition config
        if ((goal_pose_.position.x != NAN) && (goal_pose_.position.y != NAN) && (goal_pose_.position.z != NAN)) {
            // TODO : judge of orientation too...
            if (std::sqrt(std::pow(goal_pose_.position.x - pose.transform.translation.x, 2.) + std::pow(goal_pose_.position.y - pose.transform.translation.y, 2.)) < 0.1) {
                marker.color.r = 1.0;
                marker.color.g = 0.18;
                marker.color.b = 1.0;
                text.color.r = 0.0;
                text.color.g = 0.0;
                text.color.b = 1.0;
            }
        }
    }
    pub_marker_->publish(marker_array_);
}


LocationFileViewer::LocationFileViewer() : Node("location_file_viewer"), dynamic_tfBroadcaster_(this), static_tfBroadcaster_(this) {
    sub_location_file_path_ = this->create_subscription<std_msgs::msg::String>("/location_file_path", 1, std::bind(&LocationFileViewer::callback_file_path, this, std::placeholders::_1));
    sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 1, std::bind(&LocationFileViewer::callback_global_path, this, std::placeholders::_1));
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("location_arrows", 1);
    server_add_location_ = this->create_service<nav2_msgs::srv::SetInitialPose>("/add_location", std::bind(&LocationFileViewer::callback_add_location, this, std::placeholders::_1, std::placeholders::_2));
    server_delete_location_ = this->create_service<nav2_msgs::srv::SetInitialPose>("/delete_location", std::bind(&LocationFileViewer::callback_delete_location, this, std::placeholders::_1, std::placeholders::_2));

    template_marker_.type = visualization_msgs::msg::Marker::ARROW;
    template_marker_.action = visualization_msgs::msg::Marker::ADD;
    template_marker_.scale.x = 0.5; 
    template_marker_.scale.y = 0.1;
    template_marker_.scale.z = 0.1;
    template_marker_.color.r = 0.0;
    template_marker_.color.g = 0.0;
    template_marker_.color.b = 1.0;
    template_marker_.color.a = 1.0;
    template_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    template_text_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    template_text_.action = visualization_msgs::msg::Marker::ADD;
    template_text_.scale.z = 0.15; 
    template_text_.color.r = 0.0;
    template_text_.color.g = 0.0;
    template_text_.color.b = 0.0;
    template_text_.color.a = 1.0;
    template_text_.lifetime = rclcpp::Duration::from_seconds(0.1);

    this->declare_parameter<std::string>("location_file_path", "");
    this->get_parameter("location_file_path", file_name_);

    goal_pose_.position.x = goal_pose_.position.y = goal_pose_.position.z = NAN;

    if (file_name_ == "") dynamic_tf = true;
    else loadLocationFile();

    timer_ = this->create_wall_timer(100ms, std::bind(&LocationFileViewer::timer_callback, this));
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationFileViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}