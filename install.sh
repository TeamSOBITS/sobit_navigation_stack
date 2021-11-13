#!/bin/sh

sudo apt update 
# Install gmapping, amcl, move_base etc.
# Ref : https://demikko-no-bibouroku.hatenablog.com/entry/2020/08/11/015340
echo -e "\e[34m\nInstall ros-melodic-joy\e[m"
sudo apt-get install ros-melodic-joy -y

echo -e "\e[34m\nInstall ros-melodic-teleop-twist-joy\e[m"
sudo apt-get install ros-melodic-teleop-twist-joy -y

echo -e "\e[34m\nInstall ros-melodic-teleop-twist-keyboard\e[m"
sudo apt-get install ros-melodic-teleop-twist-keyboard -y

echo -e "\e[34m\nInstall ros-melodic-laser-proc\e[m"
sudo apt-get install ros-melodic-laser-proc -y

echo -e "\e[34m\nInstall ros-melodic-rgbd-launch\e[m"
sudo apt-get install ros-melodic-rgbd-launch -y

echo -e "\e[34m\nInstall ros-melodic-depthimage-to-laserscan\e[m"
sudo apt-get install ros-melodic-depthimage-to-laserscan -y

echo -e "\e[34m\nInstall ros-melodic-rosserial-arduino\e[m"
sudo apt-get install ros-melodic-rosserial-arduino -y

echo -e "\e[34m\nInstall ros-melodic-rosserial-python\e[m"
sudo apt-get install ros-melodic-rosserial-python -y

echo -e "\e[34m\nInstall ros-melodic-rosserial-server\e[m"
sudo apt-get install ros-melodic-rosserial-server -y

echo -e "\e[34m\nInstall ros-melodic-rosserial-client\e[m"
sudo apt-get install ros-melodic-rosserial-client -y

echo -e "\e[34m\nInstall ros-melodic-rosserial-msgs\e[m"
sudo apt-get install ros-melodic-rosserial-msgs -y

echo -e "\e[34m\nInstall ros-melodic-amcl\e[m"
sudo apt-get install ros-melodic-amcl -y

echo -e "\e[34m\nInstall ros-melodic-map-server\e[m"
sudo apt-get install ros-melodic-map-server -y

echo -e "\e[34m\nInstall ros-melodic-move-base\e[m"
sudo apt-get install ros-melodic-move-base -y

echo -e "\e[34m\nInstall ros-melodic-urdf\e[m"
sudo apt-get install ros-melodic-urdf -y

echo -e "\e[34m\nInstall ros-melodic-xacro\e[m"
sudo apt-get install ros-melodic-xacro -y

echo -e "\e[34m\nInstall ros-melodic-compressed-image-transport\e[m"
sudo apt-get install ros-melodic-compressed-image-transport -y

echo -e "\e[34m\nInstall ros-melodic-rqt-image-view\e[m"
sudo apt-get install ros-melodic-rqt-image-view -y

echo -e "\e[34m\nInstall ros-melodic-gmapping\e[m"
sudo apt-get install ros-melodic-gmapping -y

echo -e "\e[34m\nInstall ros-melodic-navigation\e[m"
sudo apt-get install ros-melodic-navigation -y

echo -e "\e[34m\nInstall ros-melodic-interactive-markers\e[m"
sudo apt-get install ros-melodic-interactive-markers -y

# Install rtabmap, octomap etc.
# Ref : https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB
echo -e "\e[34m\nInstall os-melodic-octomap\e[m"
sudo apt-get install ros-melodic-octomap -y

echo -e "\e[34m\nInstall os-melodic-octomap-mapping\e[m"
sudo apt-get install ros-melodic-octomap-mapping -y

echo -e "\e[34m\nInstall ros-melodic-octomap-rviz-plugins\e[m"
sudo apt-get install ros-melodic-octomap-rviz-plugins -y

echo -e "\e[34m\nInstall ros-melodic-rtabmap\e[m"
sudo apt-get install ros-melodic-rtabmap -y

echo -e "\e[34m\nInstall ros-melodic-rtabmap-ros\e[m"
sudo apt install ros-melodic-rtabmap-ros -y

echo -e "\e[34m\nInstall ros-melodic-pointcloud-to-laserscan\e[m"
sudo apt install ros-melodic-pointcloud-to-laserscan -y

echo -e "\e[34m\nInstall ros-melodic-ira-laser-tools\e[m"
sudo apt install ros-melodic-ira-laser-tools -y
