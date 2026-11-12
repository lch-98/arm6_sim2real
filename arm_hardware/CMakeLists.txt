cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/dm_bench scripts/dm_fake_motor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dm_protocol test/test_dm_protocol.py TIMEOUT 120)
  ament_add_pytest_test(test_bench_fake test/test_bench_fake.py TIMEOUT 300)
endif()

ament_package()
