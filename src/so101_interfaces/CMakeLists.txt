cmake_minimum_required(VERSION 3.22)
project(so101_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/FindObject.action
  action/MoveTo.action
  action/Pick.action
  action/Place.action
  srv/DetectObjects.srv
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
