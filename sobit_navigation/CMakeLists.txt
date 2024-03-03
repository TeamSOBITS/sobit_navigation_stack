cmake_minimum_required(VERSION 3.10.2)
project(sobit_navigation)

## Compile as C++11, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  actionlib
  visualization_msgs
  pcl_ros
  pcl_conversions

)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   move_base_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sobit_navigation
#  CATKIN_DEPENDS geometry_msgs move_base_msgs roscpp rospy sensor_msgs std_msgs tf tf2_ros actionlib visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(noise_removal_pointcloud_publisher src/noise_removal_pointcloud_publisher.cpp
                                                  src/point_cloud_processor.cpp)
target_link_libraries(noise_removal_pointcloud_publisher  ${catkin_LIBRARIES})

add_executable(noise_color_pointcloud_publisher src/noise_color_pointcloud_publisher.cpp
                                                src/point_cloud_processor.cpp)
target_link_libraries(noise_color_pointcloud_publisher  ${catkin_LIBRARIES})

add_executable(object_pointcloud_publisher src/object_pointcloud_publisher.cpp
                                           src/point_cloud_processor.cpp)
target_link_libraries(object_pointcloud_publisher  ${catkin_LIBRARIES})

add_executable(floor_color_setting src/floor_color_setting.cpp
                                   src/point_cloud_processor.cpp)
target_link_libraries(floor_color_setting  ${catkin_LIBRARIES} yaml-cpp)