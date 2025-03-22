#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
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
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>

using namespace std;

class CreateLocationFile : public rclcpp::Node {
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_location_file_path_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_msg_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<std::string> location_names;
        std::string file_name_;
        std::string robot_name_;
        bool use_robot_;
        geometry_msgs::msg::Pose nav_goal_msg_;

        bool saveLocation(const std::string &location_name);
        std::string getSavePath();
        void loadYaml();
        void callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    public:
        CreateLocationFile();
        void createLocationFile();
};

std::string CreateLocationFile::getSavePath() {
    char buffer[1024];
    FILE* pipe = popen("zenity --file-selection --save --confirm-overwrite --filename=/home/$USER/colcon_ws/src/sobit_navigation_stack/sobits_mapping/location/location_example.yaml  2>/dev/null", "r");
    if (!pipe) return "";
    fgets(buffer, sizeof(buffer), pipe);
    pclose(pipe);
    std::string path(buffer);
    path.erase(path.find_last_not_of(" \n\r\t") + 1);
    
    return path;
}

void CreateLocationFile::loadYaml() {
    YAML::Node config = YAML::LoadFile(file_name_);
    auto location_poses = config["location_pose"];

    std::cout << "========================================" << std::endl;
    std::cout << "[ LOCATION LIST ]" << std::endl;
    location_names.clear();
    int i = 1;
    for (const auto& location_pose : location_poses) {
        location_names.push_back(location_pose.first.as<std::string>());
        std::cout << "    [ " << i++ << " ] : " << location_pose.first.as<std::string>() << std::endl;
    }
}

bool CreateLocationFile::saveLocation(const std::string &location_name) {
    geometry_msgs::msg::Transform transform;
    try {
        if (use_robot_) {
            geometry_msgs::msg::TransformStamped transformStamped;
            if (tfBuffer_->canTransform("map", robot_name_ + "/base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5))) {
                transformStamped = tfBuffer_->lookupTransform("map", robot_name_ + "/base_footprint", tf2::TimePointZero);
                transform = transformStamped.transform;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Transform failed");
                std::cout << "[ CONNECTION ERROR ] Robot and PC are not connected..." << std::endl;
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
        ofs << "  \"" << location_name << "\": " << std::endl;
        ofs << "    frame_id: \"map\"" << std::endl;
        ofs << "    translation: " << std::endl;
        ofs << "      x: " << fixed << std::setprecision(7) << transform.translation.x << std::endl;
        ofs << "      y: " << fixed << std::setprecision(7) << transform.translation.y << std::endl;
        ofs << "      z: " << fixed << std::setprecision(7) << transform.translation.z << std::endl;
        ofs << "    rotation: " << std::endl;
        ofs << "      x: " << fixed << std::setprecision(7) << transform.rotation.x << std::endl;
        ofs << "      y: " << fixed << std::setprecision(7) << transform.rotation.y << std::endl;
        ofs << "      z: " << fixed << std::setprecision(7) << transform.rotation.z << std::endl;
        ofs << "      w: " << fixed << std::setprecision(7) << transform.rotation.w << std::endl;
        ofs << "" << std::endl;
        ofs.close();
        std::cout << "Saved in \"" << file_name_ << "\"." << std::endl;

        std_msgs::msg::String file_path_;
        file_path_.data = file_name_;
        pub_location_file_path_->publish(file_path_);
    } else {
        ofs.close();
        std::cout << file_name_ << " could not be created. Check the path of the file." << std::endl;
        return false;
    }
    return true;
}

void CreateLocationFile::callbackMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    nav_goal_msg_ = msg->pose;
    std::cout << "========================================" << std::endl;
    std::cout << "[ LOCATION LIST ]" << std::endl;
    int i = 1;
    for (const auto &name : location_names) std::cout << "    [ " << i++ << " ] : " << name << std::endl;
    std::cout << "[ ENTER THE LOCATION ]" << std::endl << "Please enter the Location Name. If you want to exit, type \"q\"." << std::endl << "Location Name : ";

    std::string location_name;
    std::getline(std::cin, location_name);

    if (location_name == "q") {
        std::cout << "\nOK, I'll end...." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(2));
        exit(EXIT_SUCCESS);
    } else {
        bool there_is_location_ = false;
        for (const auto &name : location_names) {
            if (name == location_name) there_is_location_ = true;
        }
        if (!there_is_location_) {
            std::cout << "Save the clicked location with \"" << location_name << "\"." << std::endl;
            if (saveLocation(location_name))
                location_names.push_back(location_name);
        }
        else std::cout << "The \""  << location_name << "\" already exists." << std::endl;
    }
    std::cout << "Click on the location you wish to register with the 2D Goal Pose." << std::endl;
}

CreateLocationFile::CreateLocationFile() : Node("create_location_file"), tfBuffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tfBuffer_)) {
    pub_location_file_path_ = this->create_publisher<std_msgs::msg::String>("/location_file_path", 1);
    while (rclcpp::ok()) {
        file_name_ = getSavePath();
        if (!file_name_.empty()) break;
        else std::cout << "Not get the correct path. Please try again." << std::endl;
    }

    std::ifstream ifs(file_name_);
    bool is_empty = true;
    if (ifs) {
        is_empty = ifs.peek() == std::ifstream::traits_type::eof();
        ifs.close();
    }
    ofstream ofs(file_name_, ios::app);
    if (ofs) {
        if (is_empty) ofs << "location_pose:" << std::endl;
        else loadYaml();
        std_msgs::msg::String file_path_;
        file_path_.data = file_name_;
        pub_location_file_path_->publish(file_path_);
    } else {
        std::cout << file_name_ << " could not be created. Check the path of the file." << std::endl;
    }
    use_robot_ = this->declare_parameter<bool>("use_robot", false);
    robot_name_ = this->declare_parameter<std::string>("robot_name", "");
    if (use_robot_) {
        createLocationFile();
    } else {
        sub_msg_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&CreateLocationFile::callbackMessage, this, std::placeholders::_1));
        std::cout << "Click on the location you wish to register with the 2D Goal Pose." << std::endl;
    }
}

void CreateLocationFile::createLocationFile() {
    while (rclcpp::ok()) {
        std::cout << "[ ENTER THE LOCATION OF ROBOT ]" << std::endl << "Please enter the Location Name. If you want to exit, type \"q\"." << std::endl << "Location Name : ";

        std::string location_name;
        std::getline(std::cin, location_name);
        if (location_name == "q") {
            std::cout << std::endl << "OK, I'll end...." << std::endl;
            rclcpp::sleep_for(std::chrono::seconds(2));
            exit(EXIT_SUCCESS);
        } else {
            bool there_is_location_ = false;
            for (const auto &name : location_names) {
                if (name == location_name) there_is_location_ = true;
            }
            if (!there_is_location_) {
                std::cout << "Save the current location as \"" << location_name << "\"." << std::endl;
                if (saveLocation(location_name))
                    location_names.push_back(location_name);
            }
            else std::cout << "The \""  << location_name << "\" already exists." << std::endl;
        }

        std::cout << "========================================" << std::endl;
        std::cout << "[ LOCATION LIST ]" << std::endl;
        int i = 1;
        for (const auto &name : location_names) std::cout << "    [ " << i++ << " ] : " << name << std::endl;
    }
}

int main(int argc, char **argv) {
    // std::cout << "========================================" << std::endl;
    rclcpp::init(argc, argv);
    // std::cout << "場所名を入力すると位置座標を保存" << std::endl;
    auto node = std::make_shared<CreateLocationFile>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}