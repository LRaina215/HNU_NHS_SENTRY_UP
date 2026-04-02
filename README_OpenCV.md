# OpenCV 4.5.4 环境配置与使用指南

**最后更新时间:** 2025-12-21
**适用主机:** lraina-Dell-G16-7620
**相关项目:** RoboMaster 自瞄系统 (ROS 2)

## 1. 安装位置 (Where is it?)

为了避免与系统自带的 OpenCV 4.2.0 冲突，OpenCV 4.5.4 被安装在一个**隔离的自定义目录**中：

* **安装根目录 (`CMAKE_INSTALL_PREFIX`):**
`/usr/local/opencv4.5.4`
* **库文件路径 (`.so` files):**
`/usr/local/opencv4.5.4/lib`
* **CMake 配置文件路径:**
`/usr/local/opencv4.5.4/lib/cmake/opencv4`

---

## 2. 如何在 CMakeLists.txt 中链接 (How to Link?)

在任何需要使用此版本 OpenCV 的 C++ 项目（如 `armor_detector`）中，**必须**在 `find_package` 之前显式指定 `OpenCV_DIR`。

**标准写法：**

```cmake
# 1. 强制指定去自定义目录寻找 OpenCV 4.5.4
set(OpenCV_DIR "/usr/local/opencv4.5.4/lib/cmake/opencv4")

# 2. 查找包（建议加上版本号以防万一）
find_package(OpenCV 4.5.4 REQUIRED)

# 3. 链接库
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  # ... 其他库
)

```

---

## 3. ROS 2 cv_bridge 特别注意事项

由于 ROS 2 默认的 `cv_bridge` 链接的是系统自带的 OpenCV 4.2.0，如果你的节点使用 4.5.4，**必须在工作空间源码编译 `cv_bridge**`。

### A. 修改 cv_bridge 的 CMakeLists.txt

位置: `src/vision_opencv/cv_bridge/CMakeLists.txt`
在文件开头添加：

```cmake
set(OpenCV_DIR "/usr/local/opencv4.5.4/lib/cmake/opencv4")

```

### B. 编译命令

在工作空间根目录 (`~/dhzzb`) 编译时，需要允许覆盖系统包：

```bash
colcon build --packages-up-to armor_detector --allow-overriding cv_bridge

```

---

## 4. 运行时环境配置 (Runtime Setup)

如果运行节点时报错 `error while loading shared libraries: libopencv_core.so.4.5: cannot open shared object file`，说明系统找不到动态库。

**已配置的解决方案 (Permanent):**
系统已在 `/etc/ld.so.conf.d/` 下创建了配置文件。

* 检查命令: `cat /etc/ld.so.conf.d/opencv4.5.4.conf`
* 内容应为: `/usr/local/opencv4.5.4/lib`
* 刷新缓存命令: `sudo ldconfig`

**临时解决方案 (Temporary):**
如果不想修改系统配置，可以在终端运行：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/opencv4.5.4/lib

```

---

## 5. 常用调试命令

* **检查当前项目链接的 OpenCV 版本:**
```bash
ldd install/armor_detector/lib/armor_detector/armor_detector_node | grep opencv
# 正确结果应指向 /usr/local/opencv4.5.4/lib/...

```


* **确认 OpenCV 4.5.4 安装完整性:**
```bash
ls /usr/local/opencv4.5.4/lib/cmake/opencv4/OpenCVConfig.cmake

```


