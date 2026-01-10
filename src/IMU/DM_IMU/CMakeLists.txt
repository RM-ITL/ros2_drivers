cmake_minimum_required(VERSION 3.8)
project(dm_imu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 串口源码按平台选择
if(UNIX)
  set(SERIAL_SRC
    ../serial/src/serial.cc
    ../serial/src/impl/unix.cc
    ../serial/src/impl/list_ports/list_ports_linux.cc
  )
elseif(WIN32)
  set(SERIAL_SRC
    ../serial/src/serial.cc
    ../serial/src/impl/win.cc
    ../serial/src/impl/list_ports/list_ports_win.cc
  )
endif()

add_library(imu_driver
  src/imu_driver.cpp
  ${SERIAL_SRC}
)

target_include_directories(imu_driver
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../serial/include
)

# ament 只写 ament 包
ament_target_dependencies(imu_driver
  rclcpp
  sensor_msgs
)

# 非 ament 包用 target_link_libraries
target_link_libraries(imu_driver
  yaml-cpp
  Eigen3::Eigen
)

add_executable(imu_driver_node src/imu_driver_node.cpp)
target_link_libraries(imu_driver_node imu_driver)
ament_target_dependencies(imu_driver_node
  rclcpp
  sensor_msgs
)

install(TARGETS imu_driver imu_driver_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/)

ament_package()
