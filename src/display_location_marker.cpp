#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class LocationPose {
public:
    LocationPose(std::string location_name, geometry_msgs::msg::Pose pose)
        : location_name_(location_name), pose_(pose) {}
    std::string location_name_;
    geometry_msgs::msg::Pose pose_;
};

class DisplayLocationMarker : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_msg_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_location_marker_;
    std::vector<LocationPose> location_poses_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    visualization_msgs::msg::Marker makeMarkerArrow(const geometry_msgs::msg::Pose& pose, int marker_id, const float size);
    visualization_msgs::msg::Marker makeMarkerString(const std::string string, const geometry_msgs::msg::Pose& pose, int marker_id, const float size);
    void displayMarker();

public:
    DisplayLocationMarker();
};

void DisplayLocationMarker::callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    location_poses_.push_back(LocationPose(msg->header.frame_id, msg->pose));
}

visualization_msgs::msg::Marker DisplayLocationMarker::makeMarkerArrow(const geometry_msgs::msg::Pose& pose, int marker_id, const float size) {
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

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

visualization_msgs::msg::Marker DisplayLocationMarker::makeMarkerString(const std::string string, const geometry_msgs::msg::Pose& pose, int marker_id, const float size) {
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

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

DisplayLocationMarker::DisplayLocationMarker()
    : Node("display_location_marker") {
    sub_msg_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/location_pose", 10, std::bind(&DisplayLocationMarker::callbackMessage, this, std::placeholders::_1));
    pub_location_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/location_pose_marker", 10);
    timer_ = this->create_wall_timer(33ms, std::bind(&DisplayLocationMarker::displayMarker, this));
}

void DisplayLocationMarker::displayMarker() {
    int marker_id = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& lp : location_poses_) {
        marker_array.markers.push_back(makeMarkerArrow(lp.pose_, marker_id, 0.5));
        marker_id++;
        marker_array.markers.push_back(makeMarkerString(lp.location_name_, lp.pose_, marker_id, 0.5));
        marker_id++;
    }
    pub_location_marker_->publish(marker_array);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayLocationMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
