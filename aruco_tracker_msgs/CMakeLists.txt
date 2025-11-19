cmake_minimum_required(VERSION 3.20)
project(aruco_tracker_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/ArucoMarkers.msg
        DEPENDENCIES std_msgs geometry_msgs ${SERVICE_MSGS_DEPS}
        )

ament_package()
