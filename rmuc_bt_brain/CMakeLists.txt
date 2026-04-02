cmake_minimum_required(VERSION 3.8)
project(rmuc_bt_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# 新增 ament_index_cpp 用于在代码中动态获取 tree.xml 的绝对路径
find_package(ament_index_cpp REQUIRED) 

# 2. 编译可执行文件
add_executable(bt_brain_node src/bt_brain_node.cpp)

# 3. 链接依赖
ament_target_dependencies(bt_brain_node 
  rclcpp 
  std_msgs 
  behaviortree_cpp_v3
  ament_index_cpp
)

# ================= 新增：安装规则 =================

# 安装编译好的 C++ 节点，让 ros2 run 能找到它
install(TARGETS
  bt_brain_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装你的 xml 行为树文件，让程序运行时能读到它
install(FILES
  tree.xml
  DESTINATION share/${PROJECT_NAME}
)

# ===============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()