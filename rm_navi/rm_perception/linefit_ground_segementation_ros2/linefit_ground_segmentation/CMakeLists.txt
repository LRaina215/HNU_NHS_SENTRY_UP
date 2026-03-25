cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation)

# find_package(catkin_simple 0.1.0 REQUIRED )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

# 【修改 1】必须在 ament 之前显式查找 PCL，并指定所有需要的组件
# 你的报错里缺了 point_cloud.h (common) 和 pcl_visualizer.h (visualization)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation visualization)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# 【修改 2】显式包含 PCL 的头文件路径 (/usr/include/pcl-1.10)
# ament_auto 往往会忽略掉 PCL 这种非 ament 包的路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 【修改 3】添加 PCL 的库目录和定义
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 生成库
ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)

# 【修改 4】将 PCL 库链接到你的目标上
# ament_auto_add_library 自动链接了 package.xml 里的依赖，但往往链不上 PCL
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

ament_auto_package()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
