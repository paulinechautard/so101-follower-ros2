cmake_minimum_required(VERSION 3.22)
project(so101_skills)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(so101_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(skills
  src/main.cpp
  src/skills_server.cpp
  src/motion.cpp
  src/find_object.cpp
  src/pick.cpp
  src/place.cpp
  src/move_to.cpp
)
target_link_libraries(skills
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  ${shape_msgs_TARGETS}
  ${so101_interfaces_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

install(TARGETS skills DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scene_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME scene_publisher)
install(PROGRAMS scripts/object_detector.py DESTINATION lib/${PROJECT_NAME} RENAME object_detector)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# Python client of the skills: from so101_skills import So101
ament_python_install_package(${PROJECT_NAME})

ament_package()
