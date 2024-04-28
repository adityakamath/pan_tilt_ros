cmake_minimum_required(VERSION 3.8)
project(pan_tilt_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(akros2_msgs REQUIRED)

include_directories(include)

# Add executables
add_executable(pan_tilt src/pan_tilt_node.cpp)
ament_target_dependencies(pan_tilt rclcpp sensor_msgs diagnostic_msgs)
install(TARGETS pan_tilt DESTINATION lib/${PROJECT_NAME})
target_link_libraries(pan_tilt ${CMAKE_SOURCE_DIR}/include/SCServo_Linux/libSCServo.a)

# ROS config files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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