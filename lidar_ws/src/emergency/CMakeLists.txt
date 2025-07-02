cmake_minimum_required(VERSION 3.5)
project(emergency)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Node executable
add_executable(emergency src/emergency.cpp)
ament_target_dependencies(emergency
  rclcpp
  std_msgs
  visualization_msgs
  pcl_conversions
)

# PCL include & link
target_include_directories(emergency PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(emergency ${PCL_LIBRARIES})

# Install
install(TARGETS
  emergency
  DESTINATION lib/${PROJECT_NAME}
)

# Package export
ament_package()

