#!/bin/bash

echo "╔══╣ Install: SOBITS Navigation (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`


# Install common dependencies
python3 -m pip install -U pip
python3 -m pip install \
    scipy

sudo apt-get update
sudo apt-get install -y \
    xterm \
    libyaml-cpp-dev

sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-nav2-common


# Go back to previous directory
cd ${DIR}


echo "╚══╣ Install: SOBITS Navigation (FINISHED) ╠══╝"
