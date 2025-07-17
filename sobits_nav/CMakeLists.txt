cmake_minimum_required(VERSION 3.8)
project(sobits_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  param/sobit_pro
  param/sobit_edu
  param/sobit_mini
  param/sobit_light
  param/hsr_sim
  param/hsrb_robot
  DESTINATION share/${PROJECT_NAME}/param
)

# include_directories(include)

# set(executable_name sobits_controller_server)
# set(library_name ${executable_name}_core)

# add_library(${library_name} SHARED src/sobits_research/sobits_controller_server.cpp)

# ament_target_dependencies(${library_name} rclcpp nav2_core nav_2d_msgs rclcpp_components nav_2d_utils lifecycle_msgs)

# add_executable(${executable_name} src/sobits_research/main.cpp)
# ament_target_dependencies(${executable_name} rclcpp nav2_core nav_2d_msgs rclcpp_components nav_2d_utils lifecycle_msgs)

# target_link_libraries(${executable_name} ${library_name})

# install(TARGETS
#   ${library_name}
#   ${executable_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# pluginlib_export_plugin_description_file(nav2_core plugin.xml)

add_executable(location_tf_broadcaster src/location_tf_broadcaster.cpp)
ament_target_dependencies(location_tf_broadcaster rclcpp geometry_msgs tf2_ros tf2 nav2_msgs tf2_geometry_msgs yaml-cpp rosidl_default_runtime)
target_link_libraries(location_tf_broadcaster yaml-cpp)
install(TARGETS location_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs rosidl_default_runtime)
install(TARGETS keyboard_teleop DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
