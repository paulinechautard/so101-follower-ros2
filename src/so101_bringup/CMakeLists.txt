cmake_minimum_required(VERSION 3.16)
project(so101_bringup)

find_package(ament_cmake REQUIRED)

foreach(dir config launch)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
