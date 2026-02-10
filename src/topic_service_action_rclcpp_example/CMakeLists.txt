cmake_minimum_required(VERSION 3.8)
project(topic_service_action_rclcpp_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(msg_srv_action_interface_example REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Build
add_executable(argument src/arithmetic/argument.cpp)
ament_target_dependencies(argument
  msg_srv_action_interface_example
  rclcpp
)

add_executable(operator src/arithmetic/operator.cpp)
ament_target_dependencies(operator
  msg_srv_action_interface_example
  rclcpp
)

add_executable(calculator src/calculator/main.cpp src/calculator/calculator.cpp)
ament_target_dependencies(calculator
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
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

install(TARGETS
  argument
  operator
  calculator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
