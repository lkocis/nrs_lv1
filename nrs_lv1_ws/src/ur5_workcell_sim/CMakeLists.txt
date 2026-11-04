cmake_minimum_required(VERSION 3.22)
project(ur5_workcell_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS scripts/robotiq_3f_sim_backend.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(idle_simulation test/test_idle_simulation.py TIMEOUT 150)
endif()

ament_package()
