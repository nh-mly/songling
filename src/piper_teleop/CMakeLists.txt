cmake_minimum_required(VERSION 3.8)
project(piper_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/teleop_bridge.py
        DESTINATION lib/${PROJECT_NAME} RENAME teleop_bridge)
install(PROGRAMS scripts/teleop_setup.py
        DESTINATION lib/${PROJECT_NAME} RENAME teleop_setup)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
