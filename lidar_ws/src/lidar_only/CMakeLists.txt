cmake_minimum_required(VERSION 3.5)
project(lidar_only)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(custom_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

# Include paths
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Node
add_executable(lidar_only_node src/lidar_only_node.cpp)

ament_target_dependencies(lidar_only_node
  rclcpp
  sensor_msgs
  std_msgs  
  custom_msgs
  pcl_conversions
)

target_link_libraries(lidar_only_node ${PCL_LIBRARIES})

# Install
install(TARGETS lidar_only_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

