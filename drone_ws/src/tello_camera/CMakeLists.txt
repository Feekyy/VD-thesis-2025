cmake_minimum_required(VERSION 3.5)
project(tello_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tello_camera/circle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
