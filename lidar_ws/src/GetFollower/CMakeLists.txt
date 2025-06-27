cmake_minimum_required(VERSION 3.8)
project(reactive_follow_gap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(reactive_follow_gap_node src/getfollower.cpp)

ament_target_dependencies(reactive_follow_gap_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(reactive_follow_gap_node ${PCL_LIBRARIES})

install(TARGETS
  reactive_follow_gap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
