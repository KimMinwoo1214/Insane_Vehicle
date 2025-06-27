cmake_minimum_required(VERSION 3.5)
project(gapfollower)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 종속 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(reactive_follow_gap src/reactive_follow_gap.cpp)

ament_target_dependencies(reactive_follow_gap
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(reactive_follow_gap
  ${PCL_LIBRARIES}
)

install(TARGETS reactive_follow_gap
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
