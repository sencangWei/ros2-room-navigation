cmake_minimum_required(VERSION 3.8)
project(room_assignment)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config rviz maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/drive_mapping.py DESTINATION lib/${PROJECT_NAME})
ament_package()
