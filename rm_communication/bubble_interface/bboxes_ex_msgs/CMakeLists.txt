cmake_minimum_required(VERSION 3.5)
project(bboxes_ex_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2D.msg"
  "msg/BoundingBoxes2D.msg"
  "msg/BoundingPolygonBox2D.msg"
  "msg/BoundingPolygonBoxes2D.msg"
  "msg/ObjectCount.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# ament_export_dependencies(rosidl_default_runtime)
ament_package()