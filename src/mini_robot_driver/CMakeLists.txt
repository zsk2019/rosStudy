cmake_minimum_required(VERSION 3.8)
project(mini_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mini_robot_interfaces REQUIRED)

add_executable(robot_monitor_node src/robot_monitor_node.cpp)
target_include_directories(robot_monitor_node PRIVATE include)
ament_target_dependencies(robot_monitor_node rclcpp mini_robot_interfaces)

add_executable(robot_mode_client src/robot_mode_client.cpp)
target_include_directories(robot_mode_client PRIVATE include)
ament_target_dependencies(robot_mode_client rclcpp mini_robot_interfaces)

add_executable(robot_thread_test src/robot_thread_test.cpp)
ament_target_dependencies(robot_thread_test rclcpp mini_robot_interfaces)

add_executable(robot_controller_node
  src/robot_controller_node.cpp
  src/robot_state_machine.cpp
)
target_include_directories(robot_controller_node PRIVATE include)
ament_target_dependencies(robot_controller_node
  rclcpp
  rclcpp_action
  std_msgs
  mini_robot_interfaces
)

add_executable(robot_controller_client src/robot_controller_client.cpp)
target_include_directories(robot_controller_client PRIVATE include)
ament_target_dependencies(robot_controller_client
  rclcpp
  rclcpp_action
  mini_robot_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  robot_monitor_node
  robot_mode_client
  robot_thread_test
  robot_controller_node
  robot_controller_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
