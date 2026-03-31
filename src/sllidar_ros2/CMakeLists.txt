cmake_minimum_required(VERSION 3.8)
project(sllidar_ros2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RPLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")

file(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(sllidar_node
  src/node.cpp
  ${RPLIDAR_SDK_SRC}
)

target_include_directories(sllidar_node PRIVATE
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
)

ament_target_dependencies(sllidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS sllidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
