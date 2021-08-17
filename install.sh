#!/bin/sh

sudo apt update 
# Ref : https://demikko-no-bibouroku.hatenablog.com/entry/2020/08/11/015340
echo "Install ros-melodic-joy"
sudo apt-get install ros-melodic-joy -y

echo "Install ros-melodic-teleop-twist-joy"
sudo apt-get install ros-melodic-teleop-twist-joy -y

echo "Install ros-melodic-teleop-twist-keyboard"
sudo apt-get install ros-melodic-teleop-twist-keyboard -y

echo "Install ros-melodic-laser-proc"
sudo apt-get install ros-melodic-laser-proc -y

echo "Install ros-melodic-rgbd-launch"
sudo apt-get install ros-melodic-rgbd-launch -y

echo "Install ros-melodic-depthimage-to-laserscan"
sudo apt-get install ros-melodic-depthimage-to-laserscan -y

echo "Install ros-melodic-rosserial-arduino"
sudo apt-get install ros-melodic-rosserial-arduino -y

echo "Install ros-melodic-rosserial-python"
sudo apt-get install ros-melodic-rosserial-python -y

echo "Install ros-melodic-rosserial-server"
sudo apt-get install ros-melodic-rosserial-server -y

echo "Install ros-melodic-rosserial-client"
sudo apt-get install ros-melodic-rosserial-client -y

echo "Install ros-melodic-rosserial-msgs"
sudo apt-get install ros-melodic-rosserial-msgs -y

echo "Install ros-melodic-amcl"
sudo apt-get install ros-melodic-amcl -y

echo "Install ros-melodic-map-server"
sudo apt-get install ros-melodic-map-server -y

echo "Install ros-melodic-move-base"
sudo apt-get install ros-melodic-move-base -y

echo "Install ros-melodic-urdf"
sudo apt-get install ros-melodic-urdf -y

echo "Install ros-melodic-xacro"
sudo apt-get install ros-melodic-xacro -y

echo "Install ros-melodic-compressed-image-transport"
sudo apt-get install ros-melodic-compressed-image-transport -y

echo "Install ros-melodic-rqt-image-view "
sudo apt-get install ros-melodic-rqt-image-view -y

echo "Install ros-melodic-gmapping"
sudo apt-get install ros-melodic-gmapping -y

echo "Install ros-melodic-navigation"
sudo apt-get install ros-melodic-navigation -y

echo "Install ros-melodic-interactive-markers"
sudo apt-get install ros-melodic-interactive-markers -y