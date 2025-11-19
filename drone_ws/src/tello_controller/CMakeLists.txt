cmake_minimum_required(VERSION 3.5)
project(tello_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tello_controller/tello_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
