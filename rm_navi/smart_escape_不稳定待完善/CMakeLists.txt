cmake_minimum_required(VERSION 3.5)
project(smart_escape)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# 1. 生成 Action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SmartEscape.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

include_directories(include)

# ==================== 核心逻辑库 ====================
add_library(smart_escape_server_lib SHARED src/smart_escape_server.cpp)

# 【核心修复】这里加上了 visualization_msgs，否则链接会报错
ament_target_dependencies(smart_escape_server_lib
  rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs nav2_msgs nav2_util tf2 tf2_ros tf2_geometry_msgs visualization_msgs
)

rosidl_target_interfaces(smart_escape_server_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 【修复点】Galactic 强制要求 EXECUTABLE 参数
rclcpp_components_register_node(smart_escape_server_lib
  PLUGIN "smart_escape::SmartEscapeServer"
  EXECUTABLE smart_escape_server_component 
)

# ==================== BT 节点插件 ====================
add_library(smart_escape_nodes SHARED src/bt_smart_escape_node.cpp)
ament_target_dependencies(smart_escape_nodes
  rclcpp rclcpp_action behaviortree_cpp_v3 geometry_msgs nav2_msgs nav2_util
)
rosidl_target_interfaces(smart_escape_nodes ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==================== 我们自己的多线程可执行文件 ====================
add_executable(smart_escape_server src/main.cpp)
target_link_libraries(smart_escape_server smart_escape_server_lib)
ament_target_dependencies(smart_escape_server rclcpp rclcpp_components)

# ==================== 安装规则 ====================
install(TARGETS smart_escape_server_lib smart_escape_nodes smart_escape_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS smart_escape_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(smart_escape_server_lib smart_escape_nodes)
ament_package()