## DHZZB_Version 哨兵自瞄2.0

**自瞄代码修改日志**

**基于 【RM2024-自瞄开源】天津大学北洋机甲-自瞄系统&OpenRM 算法库开源**  **对当前自瞄节点继续初步优化尝试**

#### 1. 优化：数字识别启用 GPU 批处理 (Batching)

- **问题：** `armor_detector_node` 中，`std::execution::par` (CPU并行) 在 `for` 循环中调用 `classify`。但 `classify` 内部的 `std::mutex` 导致 GPU 推理被**串行**执行，性能低下。
- **优化：**
  1. `std::execution::par` 循环现在只负责 CPU 密集型任务 (如 `extractNumber`)。
  2. 添加了新的 `classify_batch(armors)` 函数，在循环**之后**被调用。
  3. 此函数将所有数字图像打包，**一次性**发送给 GPU 进行并行批处理。
- **收益：** 极大提升了多目标识别时的 GPU 利用率和帧率。
- **控制：** 通过 `CMakeLists.txt` 中的 `ENABLE_GPU_BATCHING` 宏来启停此功能。

​	**如果你以后想关闭它进行对比测试，不需要改代码，只需要在编译时输入： `colcon build --cmake-args -DENABLE_GPU_BATCHING=OFF`**





#### 2. 优化：实现迭代解算，提高弹道预测精度(回退)

- **问题：** `Solver::solve` 采用“单次预测”。它基于**当前**位置猜测 `flying_time`，然后预测**未来**位置。这导致“飞行时间”和“未来位置”不匹配（“鸡生蛋”问题）。

- **优化：**
  1. 在 `Solver::solve` 中实现了一个 3 次迭代的 `for` 循环。
  2. 循环内：a. 基于 `T_guess` 预测未来位置 `P`。 b. 计算击中 `P` 所需的真实飞行时间 `T_new`。 c. 将 `T_new` 作为下一次循环的 `T_guess`。
  3. 此迭代逻辑已应用于主预测路径。`controller_delay_` 分支遵照要求，保留了“单次预测”逻辑（但也应用了下方的坐标修正）。
  
- **收益：** 飞行时间和预测位置收敛到一个精确解，显著提高对移动目标（尤其反陀螺）的瞄准精度。

  



#### 3. 优化：修复坐标系错误，使用相对向量计算

- **问题：** 发现一个严重的数学BUG：`calcYawAndPitch`、`getFlyingTime` 和 `selectBestArmor` 等函数都隐式地假设云台（机器人）在 odom 原点 (0,0,0)。
- **优化：**
  1. 在 `Solver::solve` 中，首先查询 TF 获取云台的**当前 odom 位置** (`gimbal_pos_odom`)。
  2. 所有计算（弹道、选甲、Yaw/Pitch）全部修正为使用**相对向量** (`target_vec = future_pos - gimbal_pos_odom`)。
- **收益：** 解决了“机器人移动后自瞄不准”的根本问题。现在所有计算都是基于云台的**真实**相对位置，数学上完全鲁棒。

#### 4.算法验证模拟器

​	进入at_vision_simulator-master文件后输入cargo run --release -j4运行模拟器

​	此外只需要打开auto_aim_bringup节点

​	此外还需打开auto_aim_bringup sim[TAB]节点对自瞄发布信息进行桥接，否则模拟器无法读取自瞄转动角度



#### 5.FoxGlove可视化调试工具

​	常用命令一览.md中有介绍。

​	使用` ros2 launch auto_aim_bringup foxglove.launch.py ` 启动



#### 6. 代码重构：移除硬编码内参，适配 ROS 2 标准 CameraInfo (2025-12-21) 

- **问题 (Hardcoded Intrinsics)：**  之前的版本中，相机内参矩阵 ($K$) 和畸变系数 ($D$) 被硬编码在 `armor_detector` 的源码中。这导致每当调整相机焦距、更换镜头或修改分辨率时，都必须手动修改 C++ 代码并重新编译，极不灵活且容易出错。 
- **优化 (Auto-Subscription)：**

1. 在 `ArmorDetectorNode` 初始化逻辑中，删除了硬编码的 `cv::Mat` 参数。  

2. 新增了对标准话题 `camera_info` 的订阅。  

3. 重构了 PnP 解算器的初始化流程：节点启动后会等待相机驱动发布 `sensor_msgs::msg::CameraInfo`，一旦接收到内参，自动更新 PnP Solver 的参数。 - **收益：**  实现了算法与硬件的**完全解耦**。现在调整相机参数只需修改相机驱动的 `.yaml` 配置文件，无需触碰自瞄核心代码，也无需重新编译。  



#### 7. 维护：解决 OpenCV 多版本冲突导致的崩溃 

- **故障现象：**  开发机同时存在 OpenCV 4.2.0 (System) 和 4.5.4 (Local)。`ros2 launch` 时 `armor_solver` 因链接错误版本导致 `Segmentation Fault (Exit -11)`。

- **修复方案：**  在 `CMakeLists.txt` 中强制指定 `OpenCV_DIR` 为 4.5.4 路径，并清理缓存重新编译，解决了版本冲突。（vision_opencv、image_common、image_transport_plugins为新增包，上nuc时可以去掉）

  

#### 8. 稳定性优化：移除预处理阶段的 CPU 并行 (2025-12-21) （没有完全移除，两种代码都进行了保留）

- **背景：** 在引入 GPU Batching 时，为了追求极致性能，曾尝试使用 `std::execution::par` 对装甲板的预处理（如 `extractNumber` 提取数字图、`correctCorners` 角点修正）进行 CPU 多线程并行。

- **问题：** 经过调试发现，这些预处理操作计算量极小（"极其轻量"），但频繁开启多线程引入了**线程安全隐患**和额外的上下文切换开销，导致程序偶发不稳定。 

- **调整：** - 移除了 `std::execution::par`，回退为普通的串行 `std::for_each` 循环。  

  **保留了 GPU Batching**：仅将这部分可以安全并行的、计算密集的推理任务交给 GPU 批量处理。 

  **结论：** "串行预处理 + 并行推理" 的组合在保证程序绝对稳定的前提下，依然维持了极高的识别帧率。



#### 9.自收发优化： 解决通信层中IMU数据的自收发问题

- 问题：Hardware.py中存在发布IMU数据的发布者，而Dispatch.py中直接创建了订阅者订阅该IMU数据，导致自瞄延迟升高。

- 解决：Dispatch.py中注释订阅函数，需要使用IMU数据时直接跨py文件读取。



#### 10.通信整合：整合自瞄和导航下发数据包

- 目的：只使用一个NUC与C板通信。
- 方案：整合sentry_up与sentry_down的launch文件为一个sentry_launch，并合并sentry_up与sentry_down为一个sentry。同一个数据包同时下发底盘、云台、开火数据。

#### 11.功能包整合：rm_description优化合并

**代码仓库:** git@github.com:LRaina215/HNU_NHS_BBG.git  or https://github.com/LRaina215/HNU_NHS_BBG.git 

 
