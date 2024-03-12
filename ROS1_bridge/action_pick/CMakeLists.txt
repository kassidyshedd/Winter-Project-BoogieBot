cmake_minimum_required(VERSION 3.8)
project(action_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # Add this line to find std_msgs package
find_package(messages REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Add the action_pick executable
add_executable(action_pick src/action_pick.cpp)

# Link the executable to the necessary ROS 2 packages
ament_target_dependencies(
  action_pick
  rclcpp
  std_msgs  # Add std_msgs to the dependencies
  messages
)

# Install the executable
install(TARGETS action_pick DESTINATION lib/${PROJECT_NAME})

ament_index_register_resource("ros1_bridge_foreign_mapping")
install(
  FILES rl_map_rule.yaml
  DESTINATION share/${PROJECT_NAME})

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
