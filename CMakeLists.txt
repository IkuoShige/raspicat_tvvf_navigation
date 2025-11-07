cmake_minimum_required(VERSION 3.8)
project(raspicat_tvvf_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WaypointStatus.msg"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

# CSV Reader library
add_library(csv_reader src/csv_reader.cpp)
ament_target_dependencies(csv_reader rclcpp geometry_msgs)

# Waypoint Manager library
add_library(waypoint_manager src/waypoint_manager.cpp)
target_link_libraries(waypoint_manager csv_reader)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Waypoint Follower Node
add_executable(waypoint_follower_node
  src/waypoint_follower_node.cpp
  src/waypoint_follower_main.cpp
)
target_link_libraries(waypoint_follower_node waypoint_manager csv_reader)
ament_target_dependencies(waypoint_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link against generated message library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_follower_node "${cpp_typesupport_target}")

# Simple Map Server
add_executable(simple_map_server
  src/simple_map_server.cpp
  src/simple_map_server_main.cpp
)
target_link_libraries(simple_map_server yaml-cpp ${OpenCV_LIBS})
ament_target_dependencies(simple_map_server
  rclcpp
  nav_msgs
)

# Install targets
install(TARGETS
  waypoint_follower_node
  simple_map_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  csv_reader
  waypoint_manager
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

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

# Export targets
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

ament_package()
