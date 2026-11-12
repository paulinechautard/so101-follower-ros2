cmake_minimum_required(VERSION 3.16)
project(so101_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(dir config launch)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# MuJoCo model of a world file (common.launch.py)
ament_python_install_package(${PROJECT_NAME})

ament_package()
