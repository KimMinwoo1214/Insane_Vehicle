cmake_minimum_required(VERSION 3.8)
project(yolo_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(bbox_to_lidar src/bbox_to_lidar.cpp)

ament_target_dependencies(bbox_to_lidar
  rclcpp
  sensor_msgs
  std_msgs
  custom_msgs
  pcl_conversions
)

target_link_libraries(bbox_to_lidar
  ${PCL_LIBRARIES}
)

install(TARGETS
  bbox_to_lidar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

