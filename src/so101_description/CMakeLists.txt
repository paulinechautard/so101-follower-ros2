cmake_minimum_required(VERSION 3.16)
project(so101_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

foreach(dir urdf meshes config launch rviz)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

install(DIRECTORY mjcf DESTINATION share/${PROJECT_NAME}
  PATTERN "upstream" EXCLUDE
  PATTERN "overlays" EXCLUDE)

set(build_mjcf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_mjcf.py)
set(upstream_mjcf ${CMAKE_CURRENT_SOURCE_DIR}/mjcf/upstream/so101_new_calib.xml)
set(camera_overlay ${CMAKE_CURRENT_SOURCE_DIR}/mjcf/overlays/wrist_camera.xml)
set(arm_mjcf ${CMAKE_CURRENT_BINARY_DIR}/mjcf/so101_new_calib.xml)
set(arm_camera_mjcf ${CMAKE_CURRENT_BINARY_DIR}/mjcf/so101_new_calib_camera.xml)

add_custom_command(
  OUTPUT ${arm_mjcf} ${arm_camera_mjcf}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/mjcf
  COMMAND Python3::Interpreter ${build_mjcf} ${upstream_mjcf} ${arm_mjcf}
  COMMAND Python3::Interpreter ${build_mjcf} ${upstream_mjcf} ${arm_camera_mjcf}
          --model so101_new_calib_camera --overlay ${camera_overlay}
  DEPENDS ${build_mjcf} ${upstream_mjcf} ${camera_overlay}
  COMMENT "Generating the arm MJCF from mjcf/upstream/"
)
add_custom_target(arm_mjcf ALL DEPENDS ${arm_mjcf} ${arm_camera_mjcf})
install(FILES ${arm_mjcf} ${arm_camera_mjcf} DESTINATION share/${PROJECT_NAME}/mjcf)

ament_package()
