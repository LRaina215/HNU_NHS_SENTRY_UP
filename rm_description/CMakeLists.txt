cmake_minimum_required(VERSION 3.8)
project(rm_description)

# 1. 查找依赖 (Standard)
find_package(ament_cmake REQUIRED)

# 2. 安装文件 (Install Files)
# 将 launch, urdf, meshes, config, world 等文件夹安装到 share/包名 下
install(DIRECTORY
  config
  launch
  meshes
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)

# 3. 测试模块 (Testing)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查 (可选)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. 生成包钩子 (Package Hook)
ament_package()
