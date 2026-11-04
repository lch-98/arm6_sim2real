cmake_minimum_required(VERSION 3.8)
project(arm_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_reference.py scripts/analyze_baseline.py DESTINATION lib/${PROJECT_NAME})

ament_package()
