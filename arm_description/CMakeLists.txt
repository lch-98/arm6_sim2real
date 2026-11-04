cmake_minimum_required(VERSION 3.8)
project(arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz cad
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/joint_sweep.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
