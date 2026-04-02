cmake_minimum_required(VERSION 3.5)
project(point_lio)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_compile_options(-O3 -pthread -fexceptions)

# --- 查找依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

# 【新增关键修复 1/3】查找 PCL 和 libusb
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB libusb-1.0 REQUIRED)

# OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# 【新增关键修复 2/3】包含头文件路径
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PYTHON_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${LIBUSB_INCLUDE_DIRS} 
)

add_executable(pointlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp)

ament_target_dependencies(pointlio_mapping
        rclcpp
        rclpy
        geometry_msgs
        nav_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        visualization_msgs
        tf2_ros
        livox_ros_driver2
)

# 【新增关键修复 3/3】强制链接 libusb 和 PCL
target_link_libraries(pointlio_mapping 
    ${PYTHON_LIBRARIES}
    ${PCL_LIBRARIES}
    ${LIBUSB_LIBRARIES}
)

target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS
        pointlio_mapping
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY config launch rviz_cfg
        DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp rclpy geometry_msgs nav_msgs sensor_msgs pcl_ros pcl_conversions tf2_ros livox_ros_driver2 Eigen3)

ament_package()
