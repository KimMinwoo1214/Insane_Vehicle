cmake_minimum_required(VERSION 3.8)
project(gps_kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 종속성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 실행 파일 생성
add_executable(gps_kalman_filter src/gps_kalman_filter.cpp)
ament_target_dependencies(gps_kalman_filter rclcpp sensor_msgs Eigen3)

install(TARGETS
  gps_kalman_filter
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 라이선스 검사를 건너뜀
  set(ament_cmake_cpplint_FOUND TRUE)    # cpplint 검사를 건너뜀
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
