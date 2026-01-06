cmake_minimum_required(VERSION 3.16)
project(hik_mvs_camera)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# MVS SDK
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)

add_executable(mvs_camera_node src/mvs_camera_node.cpp)
ament_target_dependencies(mvs_camera_node rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(mvs_camera_node MvCameraControl)

install(TARGETS mvs_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
