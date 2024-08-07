#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;

class LocationPose {
    public:
        std::string name;
        std::string frame_id;
        geometry_msgs::msg::Pose pose;
};

class LocationFileViewer : public rclcpp::Node {
    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_msg_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_location_marker_;
        std::vector<LocationPose> location_poses_;
        std::string file_name_;

        visualization_msgs::msg::MarkerArray marker_array_;
        int marker_id_;

        void loadLocationFile();
        void callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        visualization_msgs::msg::Marker makeMakerArrow(const geometry_msgs::msg::Pose& pose, int marker_id, const float size, const bool is_add = false);
        visualization_msgs::msg::Marker makeMakerString(const std::string string, const geometry_msgs::msg::Pose& pose, int marker_id, const float size);

    public:
        LocationFileViewer();
        void viewer();
        void displayMarker();
};

// ロケーションファイルを読み込む関数
void LocationFileViewer::loadLocationFile() {
    this->declare_parameter<std::string>("location_pose", "/home/sobits/colcon_ws/src/sobit_navigation_stack/location/location.yaml");

    rclcpp::Parameter param;
    if (this->get_parameter("location_pose", param)) {
        RCLCPP_INFO(this->get_logger(), "[ LocationFileViewer ] Load the location file\n");
    } else {
        RCLCPP_ERROR(this->get_logger(), "[ LocationFileViewer ] The location file path does not exist.\n");
        return;
    }
    auto pose_val = param.as_string();

    YAML::Node node = YAML::LoadFile(pose_val);
    YAML::Node config = node["location_pose"];

    for (const auto location_node: config) {
        LocationPose pose;
        pose.name = location_node["location_name"].as<std::string>();
        pose.frame_id = location_node["frame_id"].as<std::string>();
        pose.pose.position.x = location_node["translation_x"].as<float>();
        pose.pose.position.y = location_node["translation_y"].as<float>();
        pose.pose.position.z = location_node["translation_z"].as<float>();
        pose.pose.orientation.x = location_node["rotation_x"].as<float>();
        pose.pose.orientation.y = location_node["rotation_y"].as<float>();
        pose.pose.orientation.z = location_node["rotation_z"].as<float>();
        pose.pose.orientation.w = location_node["rotation_w"].as<float>();
        location_poses_.push_back(pose);
        marker_array_.markers.push_back(makeMakerArrow(pose.pose, marker_id_, 0.5));
        marker_id_++;
        marker_array_.markers.push_back(makeMakerString(pose.name, pose.pose, marker_id_, 0.5));
        marker_id_++;
    }

    return;
}

void LocationFileViewer::callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::cout << "\n[ 地点を登録 ] \n場所名を入力してください。「q」で終了。\nLocation Name : ";
    std::string location_name;
    std::getline(std::cin, location_name);
    if (location_name == "q") {
        std::cout << "\nOK,I'll end...." << std::endl;
        rclcpp::shutdown();
        return;
    }
    std::cout << "\n現在地点を「" << location_name << "」で保存します。" << std::endl;

    std::cout << std::endl;
    std::cout << "transform.getOrigine().x(): " << fixed << std::setprecision(7) << msg->pose.position.x << std::endl;
    std::cout << "transform.getOrigine().y(): " << fixed << std::setprecision(7) << msg->pose.position.y << std::endl;
    std::cout << "transform.getOrigine().z(): " << fixed << std::setprecision(7) << msg->pose.position.z << std::endl;

    std::cout << "transform.getRotation().x(): " << fixed << std::setprecision(7) << msg->pose.orientation.x << std::endl;
    std::cout << "transform.getRotation().y(): " << fixed << std::setprecision(7) << msg->pose.orientation.y << std::endl;
    std::cout << "transform.getRotation().z(): " << fixed << std::setprecision(7) << msg->pose.orientation.z << std::endl;
    std::cout << "transform.getRotation().w(): " << fixed << std::setprecision(7) << msg->pose.orientation.w << std::endl;

    ofstream ofs(file_name_, ios::app);
    if (ofs) {
        ofs << "    - {" << std::endl;
        ofs << "        location_name: \"" << location_name << "\"," << std::endl;
        ofs << "        frame_id: \"map\"," << std::endl;
        ofs << fixed << std::setprecision(7) << "        translation_x: " << msg->pose.position.x << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        translation_y: " << msg->pose.position.y << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        translation_z: " << msg->pose.position.z << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        rotation_x: " << msg->pose.orientation.x << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        rotation_y: " << msg->pose.orientation.y << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        rotation_z: " << msg->pose.orientation.z << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "        rotation_w: " << msg->pose.orientation.w << "," << std::endl;
        ofs << fixed << std::setprecision(7) << "    }" << std::endl;
        ofs << fixed << std::setprecision(7) << std::endl;
        ofs.close();

        std::cout << "「" << file_name_ << "」に追記完了。" << std::endl;
    } else {
        ofs.close();
        std::cout << file_name_ << "は作成できませんでした。" << std::endl;
        std::cout << "ファイルのパスを確認して下さい。" << std::endl;
    }
    LocationPose pose;
    pose.name = location_name;
    pose.frame_id = msg->header.frame_id;
    pose.pose = msg->pose;
    location_poses_.push_back(pose);
    marker_array_.markers.push_back(makeMakerArrow(pose.pose, marker_id_, 0.5, true));
    marker_id_++;
    marker_array_.markers.push_back(makeMakerString(pose.name, pose.pose, marker_id_, 0.5));
    marker_id_++;
    std::cout << "========================================" << std::endl;
    std::cout << "[ 登録地点一覧 ]" << std::endl;
    int i = 1;
    for (const auto& pose : location_poses_) std::cout << "    [ " << i++ << " ] : " << pose.name << std::endl;
    return;
}

visualization_msgs::msg::Marker LocationFileViewer::makeMakerArrow(const geometry_msgs::msg::Pose& pose, int marker_id, const float size, const bool is_add) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "location_pose";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = pose;

    marker.scale.x = size;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;

    if (is_add) {
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

    return marker;
}

visualization_msgs::msg::Marker LocationFileViewer::makeMakerString(const std::string string, const geometry_msgs::msg::Pose& pose, int marker_id, const float size) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "location_pose";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

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

    return marker;
}

LocationFileViewer::LocationFileViewer() : Node("location_file_viewer") {
    this->declare_parameter<std::string>("location_file_path", "/map");
    this->get_parameter("location_file_path", file_name_);
    loadLocationFile();
    sub_msg_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose2", 1, std::bind(&LocationFileViewer::callbackMessage, this, std::placeholders::_1));
    pub_location_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/location_pose_marker", 1);
}

void LocationFileViewer::viewer() {
    rclcpp::Rate loop_rate(30);
    std::cout << "========================================" << std::endl;
    std::cout << "[ 登録地点一覧 ]" << std::endl;
    int i = 1;
    for (const auto& pose : location_poses_) std::cout << "    [ " << i++ << " ] : " << pose.name << std::endl;
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        pub_location_marker_->publish(marker_array_);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationFileViewer>();
    node->viewer();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
