cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++17 -O3)

find_package(ament_cmake_auto REQUIRED)

# 【新增关键修复 1/3】手动查找 PCL 和 libusb
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB libusb-1.0 REQUIRED)

ament_auto_find_build_dependencies()

# 【新增关键修复 2/3】包含路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

# 定义可执行文件
ament_auto_add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
ament_auto_add_executable(ground_segmentation_test_node src/ground_segmentation_test_node.cc)

# 【新增关键修复 3/3】手动链接库 (ament_auto 不会自动链接外部库)
target_link_libraries(ground_segmentation_node 
  ${PCL_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

target_link_libraries(ground_segmentation_test_node 
  ${PCL_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE launch)
