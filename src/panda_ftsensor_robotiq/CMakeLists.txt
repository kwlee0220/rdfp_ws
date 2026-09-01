cmake_minimum_required(VERSION 3.8)
project(panda_ftsensor_robotiq)

find_package(ament_cmake REQUIRED)

# 자산만 설치한다. `package://panda_ftsensor_robotiq/meshes/...` 가 여기서 해석된다.
install(DIRECTORY meshes urdf srdf DESTINATION share/${PROJECT_NAME})

ament_package()
