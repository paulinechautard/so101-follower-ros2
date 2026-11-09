cmake_minimum_required(VERSION 3.16)
project(so101_description)

find_package(ament_cmake REQUIRED)

foreach(dir urdf mjcf meshes config launch rviz)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
