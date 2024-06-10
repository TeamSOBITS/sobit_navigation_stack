# #!/bin/bash
# # 参考①：https://demikko-no-bibouroku.hatenablog.com/entry/2020/08/11/015340
# # 参考②：https://qiita.com/ryu_software/items/d13a70aacfc6a71cacdb#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB

echo "╔══╣ Install: Sobit Navigation Stack (STARTING) ╠══╗"

sudo apt-get update

cd ~/catkin_ws/src/sobit_navigation_stack/
mkdir sobit_navigation_packages
cd ~/catkin_ws/src/sobit_navigation_stack/sobit_navigation_packages/
if [ "clone_mode" == $1 ]; then
    echo -e "\e[34m git clone mode \e[m"
    git clone https://github.com/ros-perception/slam_gmapping.git
    git clone https://github.com/ros-perception/openslam_gmapping.git
    git clone https://github.com/ros-planning/navigation_msgs.git
    sudo rm -rv navigation
    git clone -b sobit_pro/y-dwa git clone https://github.com/TeamSOBITS/navigation.git
    git clone https://github.com/ros/geometry2.git

else 
    echo -e "\e[34m install mode \e[m"
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-amcl \
        ros-${ROS_DISTRO}-map-server \
        ros-${ROS_DISTRO}-move-base \
        ros-${ROS_DISTRO}-gmapping \
        ros-${ROS_DISTRO}-navigation
fi

git clone https://github.com/TeamSOBITS/als_ros.git
cd ~/catkin_ws/src/sobit_navigation_stack/

# Install gmapping, amcl, move_base etc.
sudo apt-get install -y \
    ros-${ROS_DISTRO}-interactive-markers \
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
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-explore-lite

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