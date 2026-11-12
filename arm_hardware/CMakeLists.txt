cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ros2_control 하드웨어 플러그인 (C++)
add_library(${PROJECT_NAME} SHARED src/damiao_system.cpp src/can_transport.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# 벤치 도구 (Python)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/dm_bench scripts/dm_fake_motor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_dm_protocol_cpp test/test_dm_protocol.cpp)
  target_include_directories(test_dm_protocol_cpp PRIVATE include)
  ament_add_pytest_test(test_dm_protocol test/test_dm_protocol.py TIMEOUT 120)
  ament_add_pytest_test(test_bench_fake test/test_bench_fake.py TIMEOUT 300)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
