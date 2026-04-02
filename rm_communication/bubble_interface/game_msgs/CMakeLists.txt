cmake_minimum_required(VERSION 3.5)
project(game_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameStatus.msg"
  "msg/RobotHP.msg"
  "msg/Zone.msg"
  DEPENDENCIES std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()