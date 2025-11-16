# #!/bin/bash
# # 参考①：https://demikko-no-bibouroku.hatenablog.com/entry/2020/08/11/015340
# # 参考②：https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB

echo "╔══╣ Install: Sobit Navigation Stack (STARTING) ╠══╗"

sudo apt update

sudo apt install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    zenity

git clone -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/explore_ros2.git

git clone -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/flex_nav.git

echo export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp >> ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "╚══╣ Install: Sobit Navigation Stack (FINISHED) ╠══╝"
