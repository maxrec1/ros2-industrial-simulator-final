cmake_minimum_required(VERSION 3.8)

project(bobby)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz textures urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
