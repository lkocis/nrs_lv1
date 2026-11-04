cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_3f_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robotiq_3f_controller_node src/controller_node.cpp)
ament_target_dependencies(robotiq_3f_controller_node
  control_msgs
  rclcpp
  rclcpp_action
  robotiq_3f_interfaces
  std_srvs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS robotiq_3f_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/test_controller.py TIMEOUT 30)
endif()

ament_package()
