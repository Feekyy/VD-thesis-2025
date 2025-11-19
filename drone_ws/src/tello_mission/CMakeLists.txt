cmake_minimum_required(VERSION 3.5)
project(tello_mission)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tello_mission/tello_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
