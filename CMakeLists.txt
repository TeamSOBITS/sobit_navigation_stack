cmake_minimum_required(VERSION 3.5)
project(sobit_navigation_stack)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

nav2_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(create_location_file src/create_location_file.cpp)
ament_target_dependencies(create_location_file rclcpp tf2_ros geometry_msgs tf2_geometry_msgs)

add_executable(display_location_marker src/display_location_marker.cpp)
ament_target_dependencies(display_location_marker rclcpp visualization_msgs geometry_msgs)

add_executable(location_file_viewer src/location_file_viewer.cpp)
ament_target_dependencies(location_file_viewer yaml-cpp rclcpp visualization_msgs geometry_msgs)
target_link_libraries(location_file_viewer yaml-cpp)


install(PROGRAMS
  scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  create_location_file
  display_location_marker
  location_file_viewer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
