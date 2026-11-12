cmake_minimum_required(VERSION 3.22)
project(so101_examples)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/keeper.py DESTINATION lib/${PROJECT_NAME} RENAME keeper)
install(PROGRAMS scripts/sorter.py DESTINATION lib/${PROJECT_NAME} RENAME sorter)
install(DIRECTORY config worlds DESTINATION share/${PROJECT_NAME})

ament_package()
