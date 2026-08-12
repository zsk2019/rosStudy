cmake_minimum_required(VERSION 3.8)
project(mini_robot_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(laser_mock src/laser_mock.cpp)
ament_target_dependencies(laser_mock rclcpp sensor_msgs)

add_executable(laser_listener src/laser_listener.cpp)
ament_target_dependencies(laser_listener rclcpp sensor_msgs)

install(TARGETS
  laser_mock
  laser_listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
