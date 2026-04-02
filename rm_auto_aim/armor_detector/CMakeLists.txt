cmake_minimum_required(VERSION 3.10)
project(armor_detector)

# ==================== 【核心修复：这里必须加！】 ====================
# 你的 OpenCV 4.5.4 库所在的绝对路径
set(OPENCV_LIB_PATH "/home/rm/opencv_4.5.4_local/lib")

# 告诉 CMake：编译出来的程序，身上必须纹上这个路径 (RPATH)
set(CMAKE_INSTALL_RPATH "${OPENCV_LIB_PATH}")
# 即使在 build 目录没安装前，也强制使用这个路径
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
# 链接时自动继承其他依赖库的路径
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# ===================================================================

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ... (后面保持和你原来的一模一样，不需要动) ...
# ==================== 新增部分 ====================
# 定义一个选项，默认名为 ENABLE_GPU_BATCHING，默认值为 ON (开启)
option(ENABLE_GPU_BATCHING "Enable GPU batching for number classification" FALSE)

if(ENABLE_GPU_BATCHING)
    message(STATUS "GPU Batching Enabled!")
    add_definitions(-DENABLE_GPU_BATCHING)
endif()
# =================================================

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${G2O_LIBRARIES})
set(OpenCV_DIR "/home/rm/opencv_4.5.4_local/lib/cmake/opencv4") # 这行保留，给 find_package 用的
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)

find_package(TBB REQUIRED COMPONENTS tbb)
find_package(g2o REQUIRED) # 注意这里你改回小写了，很好
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
g2o_core
g2o_stuff
g2o_solver_csparse
g2o_types_sba
g2o_types_slam3d
g2o_solver_dense
fmt::fmt
tbb
)

ament_target_dependencies(${PROJECT_NAME} rm_interfaces)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_detector test/test_detector.cpp)
  target_link_libraries(test_detector ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
