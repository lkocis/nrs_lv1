cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_state_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotiq_3f_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(robotiq_3f_joint_mapping src/joint_mapping.cpp)
target_include_directories(robotiq_3f_joint_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(robotiq_3f_joint_state_publisher_node
  src/joint_state_publisher_node.cpp
)
target_link_libraries(
  robotiq_3f_joint_state_publisher_node robotiq_3f_joint_mapping
)
ament_target_dependencies(robotiq_3f_joint_state_publisher_node
  rclcpp
  robotiq_3f_interfaces
  sensor_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  robotiq_3f_joint_mapping
  robotiq_3f_joint_state_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_joint_mapping test/test_joint_mapping.cpp)
  target_link_libraries(test_joint_mapping robotiq_3f_joint_mapping)
endif()

ament_export_include_directories(include)
ament_export_libraries(robotiq_3f_joint_mapping)
ament_package()
