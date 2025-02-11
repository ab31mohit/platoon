cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

# Add executables (and their dependencies) for all cpp based nodes
add_executable(initial_pose_publisher_cpp src/initial_pose_publisher_cpp.cpp)
ament_target_dependencies(initial_pose_publisher_cpp rclcpp geometry_msgs nav_msgs)

# Install all python nodes
install(PROGRAMS
  src/update_ns_param.py
  src/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all cpp nodes
install(TARGETS initial_pose_publisher_cpp DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
