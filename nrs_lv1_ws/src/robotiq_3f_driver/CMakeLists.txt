cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotiq_3f_interfaces REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}_core
  src/modbus_tcp_client.cpp
  src/protocol.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Boost::system Threads::Threads)
ament_target_dependencies(${PROJECT_NAME}_core robotiq_3f_interfaces)

add_executable(robotiq_3f_driver_node src/driver_node.cpp)
target_link_libraries(robotiq_3f_driver_node ${PROJECT_NAME}_core)
ament_target_dependencies(robotiq_3f_driver_node
  diagnostic_updater
  diagnostic_msgs
  rclcpp
  robotiq_3f_interfaces
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(TARGETS
  ${PROJECT_NAME}_core
  robotiq_3f_driver_node
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

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_core)
  ament_target_dependencies(test_protocol robotiq_3f_interfaces)

  ament_add_gtest(test_modbus_tcp_client test/test_modbus_tcp_client.cpp)
  target_link_libraries(test_modbus_tcp_client
    ${PROJECT_NAME}_core Boost::system Threads::Threads
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(robotiq_3f_interfaces)
ament_package()
