cmake_minimum_required(VERSION 3.8)
project(aruco_tracker_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies (cv_bridge REMOVED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_tracker_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(aruco_node_cpp src/aruco_node.cpp)

# Includes (Simple again)
target_include_directories(aruco_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_tracker_msgs_INCLUDE_DIRS}
)

# Linking (cv_bridge REMOVED)
ament_target_dependencies(aruco_node_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  aruco_tracker_msgs
  cv_bridge
)

# Link OpenCV Manually
target_link_libraries(aruco_node_cpp
  ${OpenCV_LIBRARIES}
)

install(TARGETS aruco_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()