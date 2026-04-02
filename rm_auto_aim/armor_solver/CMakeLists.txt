cmake_minimum_required(VERSION 3.10)
project(armor_solver)

# ==================== 【核心修复：请务必保留这段！】 ====================
# 1. 编译时：强制指定 OpenCV 4.5.4 配置文件的路径
set(OpenCV_DIR "/home/rm/opencv_4.5.4_local/lib/cmake/opencv4")

# 2. 运行时：你的 OpenCV 4.5.4 库所在的绝对路径
set(OPENCV_LIB_PATH "/home/rm/opencv_4.5.4_local/lib")

# 3. RPATH 设置：告诉 CMake 把这个路径“刻录”到可执行文件里
set(CMAKE_INSTALL_RPATH "${OPENCV_LIB_PATH}")
# 即使在 build 目录没安装前，也强制使用这个路径
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
# 链接时自动继承其他依赖库的路径
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# ===================================================================

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(image_transport REQUIRED)
#find_package(tf2_eigen REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# 注意：虽然这里用 auto 找依赖，但只要 package.xml 里有 opencv，
# 它就会受上面 OpenCV_DIR 的控制，找到对的版本。
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
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
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

endif()

#############
## Install ##
#############

ament_auto_package()
