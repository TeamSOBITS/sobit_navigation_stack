#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdlib>
#include <string>

std::string getSavePath() {
    char buffer[1024];
    FILE* pipe = popen("zenity --file-selection --save --confirm-overwrite --filename=/home/$USER/colcon_ws/src/map_name 2>/dev/null", "r");
    if (!pipe) return "";
    fgets(buffer, sizeof(buffer), pipe);
    pclose(pipe);
    std::string path(buffer);
    path.erase(path.find_last_not_of(" \n\r\t") + 1);
    
    return path;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sobits_map_saver");

    std::string runPath;
    std::string savePath;
    while (rclcpp::ok()) {
        savePath = getSavePath();
        if (!savePath.empty()) break;
        else std::cout << "正しいパスが得られませんでした。もう一度お願いします。" << std::endl;
    }
    std::cout << savePath << "に保存します..." << std::endl;

    runPath = "ros2 run nav2_map_server map_saver_cli -f " + savePath;
    FILE* pipe = popen(runPath.c_str(), "r");
    pclose(pipe);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}