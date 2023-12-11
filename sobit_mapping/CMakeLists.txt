cmake_minimum_required(VERSION 3.10.2)
project(sobit_mapping)

## Compile as C++11, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  # tf
  tf2
  tf2_ros
  visualization_msgs
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
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sobit_mapping
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf tf2_ros visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(create_location_file src/create_location_file.cpp)
target_link_libraries(create_location_file  ${catkin_LIBRARIES})

add_executable(display_location_marker src/display_location_marker.cpp)
target_link_libraries(display_location_marker  ${catkin_LIBRARIES})

# add_executable(projected_map_saver src/projected_map_saver.cpp)
# target_link_libraries(projected_map_saver  ${catkin_LIBRARIES})

add_executable(location_file_viewer src/location_file_viewer.cpp)
target_link_libraries(location_file_viewer  ${catkin_LIBRARIES})
