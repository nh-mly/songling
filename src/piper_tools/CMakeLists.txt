cmake_minimum_required(VERSION 3.8)
project(piper_tools)

find_package(ament_cmake REQUIRED)

# 每个脚本独立安装成可执行,并去掉 .py 后缀(ros2 run piper_tools jog 而不是 jog.py)
install(PROGRAMS scripts/status.py DESTINATION lib/${PROJECT_NAME} RENAME status)
install(PROGRAMS scripts/jog.py    DESTINATION lib/${PROJECT_NAME} RENAME jog)
install(PROGRAMS scripts/reset.py  DESTINATION lib/${PROJECT_NAME} RENAME reset)
install(PROGRAMS scripts/move_to.py DESTINATION lib/${PROJECT_NAME} RENAME move_to)
install(PROGRAMS scripts/add_table.py DESTINATION lib/${PROJECT_NAME} RENAME add_table)

# common.py 与三个脚本同目录安装 → 脚本里 import common 直接可用
install(FILES scripts/common.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
