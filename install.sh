#!/bin/bash
# 参考①：https://demikko-no-bibouroku.hatenablog.com/entry/2020/08/11/015340
# 参考②：https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB


echo "╔══╣ Install: Sobit Navigation Stack (STARTING) ╠══╗"


sudo apt-get update

# Install gmapping, amcl, move_base etc.
sudo apt-get install -y \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-laser-proc \
    ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-rosserial-python \
    ros-${ROS_DISTRO}-rosserial-server \
    ros-${ROS_DISTRO}-rosserial-client \
    ros-${ROS_DISTRO}-rosserial-msgs \
    ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-gmapping \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-interactive-markers

# Install rtabmap, octomap etc.
sudo apt-get install -y \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-octomap-mapping \
    ros-${ROS_DISTRO}-octomap-rviz-plugins \
    ros-${ROS_DISTRO}-rtabmap \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-ira-laser-tools


echo "╚══╣ Install: Sobit Navigation Stack (FINISHED) ╠══╝"
