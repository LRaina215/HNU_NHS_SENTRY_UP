# HNU_NHS_SENTRY_UP-down

本仓库是一个面向 RoboMaster 哨兵机器人联调的 ROS 2 工作空间，当前代码已经把自瞄、串口通信、导航建图、机器人描述、战术决策和行为树“上/下位机”逻辑整合到了同一套工程里。

从当前源码来看，它更像一套“比赛联调工作空间”而不是单一功能包：既包含可直接运行的主链路，也保留了若干实验模块、第三方移植包、日志文件、示意资源和调试脚本。

## 1. 工作空间概览

当前工作空间的核心目标是服务一台哨兵机器人，形成如下闭环：

- 自瞄链路：工业相机取流 -> 装甲板识别 -> 跟踪/弹道解算 -> 云台控制指令下发
- 通信链路：板间串口收发 -> ROS 话题发布/订阅 -> TF、里程计、裁判系统状态同步
- 导航链路：Livox 点云 -> 车体点云过滤/地面分割 -> 2D LaserScan -> Nav2 定位与导航
- 决策链路：底层状态与目标信息汇总 -> Python 战术动作层 / C++ 行为树“大脑” -> 姿态与导航目标切换

## 2. 目录结构

```text
HNU_NHS_SENTRY_UP-down/
├─ rm_auto_aim/                 自瞄相关包
│  ├─ armor_detector/           装甲板检测与数字分类
│  ├─ armor_solver/             EKF 跟踪、目标选择、弹道解算
│  ├─ rm_vision_ros2_hik_camera/海康工业相机 ROS 2 驱动
│  ├─ auto_aim_bringup/         轻量自瞄启动入口
│  ├─ rm_interfaces/            自瞄链路自定义消息/服务
│  ├─ rm_utils/                 PnP、EKF、日志、弹道等通用工具
│  ├─ rm_robot_description/     自瞄侧机器人描述
│  ├─ rm_bringup/               旧版综合 bringup
│  └─ rmoss_projectile_motion/  弹道模型工具库
├─ rm_communication/            通信与决策相关包
│  ├─ bubble_protocol/          串口协议、收发调度、状态发布
│  ├─ bubble_decision/          哨兵战术决策与动作层
│  └─ bubble_interface/         game_msgs / rmctrl_msgs / bboxes_ex_msgs
├─ rm_description/              哨兵整机 URDF、Gazebo 模型、比赛场地 SDF
├─ rm_navi/                     导航与感知相关包
│  ├─ rm_driver/livox_ros_driver2/
│  ├─ rm_localization/          point_lio / icp_registration
│  ├─ rm_navigation/navi/       SLAM、AMCL、Nav2、RViz 启动与参数
│  ├─ rm_perception/            地面分割、点云转激光、Terrain Analysis
│  ├─ rm_lidar_filter/          车体点云过滤
│  ├─ fake_vel_transform/       全向底盘速度坐标适配
│  └─ smart_escape_不稳定待完善/ 实验性脱困模块
├─ rmuc_bt_brain/               C++ 行为树战术“大脑”
├─ 0308pre.sh / pre.sh          导航预处理与联调脚本
├─ mapping.sh / map.sh / nav.sh 建图、地图与导航脚本
├─ game.sh                      综合联调脚本
├─ decision_uc.sh               C++/Python 决策链启动脚本
├─ autoaimstart.sh              自瞄看门狗脚本
├─ frames.gv / frames.pdf       TF 结构快照
├─ test.pgm / test.yaml         测试地图
└─ README_OpenCV.md 等          补充说明文档
```

补充说明：

- `costmap_converter/`、`teb_local_planner/` 目录当前为空，占位但未形成实际源码包。
- `rm_auto_aim/rm_vision_ros2_hik_camera/build`、`cmake-build-debug`、`MvSdkLog` 等目录属于构建/调试产物，已被保留在工作空间中。

## 3. 代码主链路

### 3.1 自瞄链路

核心包：

- `rm_auto_aim/rm_vision_ros2_hik_camera`
- `rm_auto_aim/armor_detector`
- `rm_auto_aim/armor_solver`
- `rm_auto_aim/auto_aim_bringup`
- `rm_auto_aim/rm_interfaces`

实际数据流：

```text
hik_camera(image_raw + camera_info)
  -> armor_detector
  -> armor_solver
  -> armor_solver/cmd_gimbal
  -> bubble_protocol
  -> MCU / 云台
```

当前源码中的关键实现：

- `hik_camera_node` 使用海康 USB3.0 SDK 枚举设备、设置分辨率/曝光/增益，并发布 `image_raw` 与 `camera_info`
- `armor_detector` 订阅 `image_raw` 和 `camera_info`，发布 `armor_detector/armors`、调试图像和可视化 marker
- `armor_detector` 还会订阅 `red_blue_info`，动态切换敌我颜色
- `armor_solver` 通过 TF 将目标统一到 `gimbal_odom`，做 EKF 跟踪、目标选择和弹道补偿，发布：
  - `armor_solver/target`
  - `armor_solver/measurement`
  - `armor_solver/cmd_gimbal`
- `auto_aim_bringup/launch/auto_aim.launch.py` 是当前仓库中最直接的自瞄启动入口

### 3.2 通信链路

核心包：

- `rm_communication/bubble_protocol`
- `rm_communication/bubble_interface`

当前 `bubble_protocol` 的职责不是单纯串口驱动，而是“ROS <-> 串口板间协议”的调度层：

- 订阅：
  - `armor_solver/cmd_gimbal`
  - `/cmd_vel`
  - `/manual/sentry_posture`
  - `serial/receive`
- 下发到串口：
  - 云台控制
  - 底盘控制
  - 发射控制
  - 哨兵姿态切换
- 从串口回传并发布：
  - `/joint_states`
  - `/odom`
  - `/imu`
  - `/status/barrel`
  - `/status/game`
  - `/status/zone`
  - `/status/robotHP`
  - `/status/sentry_posture_feedback`

`rm_communication/bubble_interface` 提供了三类接口包：

- `game_msgs`：裁判系统相关消息
- `rmctrl_msgs`：底盘/云台/射击控制消息
- `bboxes_ex_msgs`：扩展目标框消息

### 3.3 导航与建图链路

核心包：

- `rm_navi/rm_driver/livox_ros_driver2`
- `rm_navi/rm_lidar_filter`
- `rm_navi/rm_perception/linefit_ground_segementation_ros2`
- `rm_navi/rm_perception/pointcloud_to_laserscan`
- `rm_navi/rm_localization/point_lio`
- `rm_navi/rm_localization/icp_registration`
- `rm_navi/rm_navigation/navi`
- `rm_navi/fake_vel_transform`

当前主流程可概括为：

```text
Livox点云
  -> rm_lidar_filter              去车体点云
  -> linefit_ground_segmentation  地面/障碍分割
  -> pointcloud_to_laserscan      转 /scan
  -> Nav2(localization/navigation)
```

并行的定位/建图链路：

```text
Livox点云 + IMU
  -> point_lio
  -> /odom
  -> odom_to_base_node.py
  -> /odom_base
  -> Nav2 controller/amcl
```

当前导航栈特征：

- `navi/launch/slam_launch.py`：SLAM Toolbox 建图
- `navi/launch/localization_launch.py`：地图服务器 + AMCL
- `navi/launch/navigation_launch.py`：Nav2 控制、规划、恢复、BT Navigator
- `navi/launch/rviz_launch.py`：RViz
- `navi/params/nav2_params.yaml`：
  - `planner_server` 默认使用 `nav2_theta_star_planner/ThetaStarPlanner`
  - `controller_server` 默认使用 `DWBLocalPlanner`
  - `robot_base_frame` 以 `base_link` 为主
  - 局部与全局 costmap 均以 `/scan` 为主要障碍输入
- `fake_vel_transform` 用于将 Nav2 输出速度从“虚拟不旋转底盘系”转换回真实底盘坐标系，适配高频旋转或全向运动底盘
- `icp_registration` 读取 PCD 地图并发布 `map -> odom`，作为补充定位方案

### 3.4 决策链路

当前仓库里其实有两套相关逻辑同时存在：

#### A. `bubble_decision` 的战术动作层

主要包含：

- `decision.py`
- `gameAction.py`
- `rmuc_body.py`
- `rmuc_action.py`
- `rmuc_decision.py`

其中当前更值得关注的是：

- `gameAction.py` 中的 `SentryGameAction`
  - 根据血量、目标距离、当前位置等信息执行“守中、回补、切姿态”等动作
  - 直接对接 `/navigate_to_pose`
- `rmuc_body.py`
  - 更像 Python 动作执行层
  - 汇总黑板数据到 `/sentry/blackboard_data`
  - 接收 C++ 大脑发来的 `/sentry/tactic_cmd`
  - 管理 `/manual/sentry_posture`
  - 处理人工接管状态 `/sentry/human_override`

#### B. `rmuc_bt_brain` 的 C++ 行为树大脑

该包通过 BehaviorTree.CPP 实现一个轻量战术决策树：

- 订阅 `/sentry/blackboard_data`
- 订阅 `/sentry/human_override`
- 发布 `/sentry/tactic_cmd`

当前 `tree.xml` 的战术逻辑非常直接：

- 人工接管 -> `OVERRIDE_IDLE`
- 低血量 -> `DEFEND`
- 有敌人且距离近 -> `MELEE`
- 有敌人且距离远 -> `PURSUIT`
- 其他情况 -> `PATROL`

这意味着当前仓库的战术架构是：

```text
通信/导航/自瞄状态
  -> Python动作层汇总黑板
  -> C++ BT大脑做战术判断
  -> Python动作层执行姿态切换与导航目标下发
```

### 3.5 机器人描述与仿真

核心包：

- `rm_description`
- `rm_auto_aim/rm_robot_description`

其中：

- `rm_description` 更偏向“当前整机联调版本”
  - `launch/model.launch.py` 同时发布导航侧和自瞄侧 robot_state_publisher
  - `launch/sim_bringup.launch.py` 可加载 Gazebo 世界与模型
  - `models/`、`world/` 中保存了 RMUC/RMUL 2024/2025 相关场地与模型资源
- `rm_robot_description` 更偏向自瞄模块继承来的描述包

## 4. 自定义接口

### 4.1 `rm_auto_aim/rm_interfaces`

当前使用最频繁的消息/服务包括：

- `Armors.msg`
- `Armor.msg`
- `Target.msg`
- `GimbalCmd.msg`
- `Measurement.msg`
- `SerialReceiveData.msg`
- `SetMode.srv`

### 4.2 `rm_communication/bubble_interface`

- `game_msgs`
  - `GameStatus.msg`
  - `RobotHP.msg`
  - `Zone.msg`
- `rmctrl_msgs`
  - `Chassis.msg`
  - `Gimbal.msg`
  - `Shooter.msg`
  - `Odom.msg`

## 5. 典型启动方式

### 5.1 直接使用根目录脚本

当前仓库保留了多份 Linux + GNOME 终端脚本，适合现场联调：

- `mapping.sh`
  - 启动 Livox 驱动与 Point-LIO
- `map.sh`
  - 启动 SLAM Toolbox 与 RViz，用于建图
- `nav.sh`
  - 启动定位、导航与 RViz
- `pre.sh` / `0308pre.sh`
  - 启动导航前置链路，如点云过滤、地面分割、点云转激光、Point-LIO
- `game.sh`
  - 综合启动导航/决策/定位相关节点
- `decision_uc.sh`
  - 启动 `rmuc_body` 与 `bt_brain_node`
- `autoaimstart.sh`
  - 自瞄链路看门狗，自动拉起相机、描述、解算与通信节点

### 5.2 直接使用 `ros2 launch`

常见入口如下：

```bash
ros2 launch hik_camera hik_camera.launch.py
ros2 launch auto_aim_bringup auto_aim.launch.py
ros2 launch bubble_protocol sentry_launch.py
ros2 launch point_lio mapping_mid360.launch.py
ros2 launch icp_registration icp.launch.py
ros2 launch navi slam_launch.py
ros2 launch navi localization_launch.py
ros2 launch navi navigation_launch.py
ros2 launch navi rviz_launch.py
ros2 launch rm_description model.launch.py
```

## 6. 构建与运行环境

从当前代码和脚本判断，本工作空间主要面向以下环境：

- Ubuntu Linux
- ROS 2 Galactic
- Bash + `gnome-terminal`
- Livox MID360 / Livox ROS 2 驱动
- 海康 USB3.0 工业相机 SDK

常规构建方式：

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
```

说明：

- 自瞄相机驱动已经在仓库中带有 `hikSDK` 动态库，但实际能否运行仍依赖本机驱动环境
- 导航依赖 Nav2、SLAM Toolbox、PCL、TF2、RViz 等 ROS 2 组件
- 工作空间中有较多源码移植包和第三方代码，首次构建前建议先跑 `rosdep install --from-paths . --ignore-src -r -y`

## 7. 当前代码中的重要现实约束

这部分非常重要，因为 README 介绍的是“当前仓库实际状态”，不是理想状态。

### 7.1 存在硬编码绝对路径

当前源码中多处直接写死了类似路径：

- `/home/robomaster/shaobing/install/setup.bash`
- `/home/robomaster/shaobing/src/...`
- `/home/robomaster/opencv_4.5.4_local/lib`

受影响位置包括但不限于：

- 根目录多个 `.sh` 启动脚本
- `auto_aim_bringup/launch/auto_aim.launch.py`
- `rm_navi/rm_navigation/navi/launch/navigation_launch.py`
- `rm_navi/rm_navigation/navi/params/nav2_params.yaml`
- `rm_navi/rm_localization/icp_registration/config/icp.yaml`

如果你在其它机器上使用本仓库，这部分通常需要先修改。

### 7.2 仓库中同时存在“当前主链路”和“实验/备用链路”

例如：

- `smart_escape_不稳定待完善/`：实验性脱困方案，当前默认未在 Nav2 主链中启用
- `terrain_analysis/`：作为 linefit 方案的替代探索，脚本中保留了切换痕迹
- `icp_registration/`：补充定位方案，不是所有脚本都会默认启动
- `rm_bringup/` 与 `auto_aim_bringup/`：存在新旧两套 bringup 风格

### 7.3 仓库中保留了构建产物和日志

例如：

- 海康相机构建目录
- `MvSdkLog/`
- 图片、PDF、流程图等调试资源

这些内容对联调排障有帮助，但也说明当前仓库不是“纯净源码包”。

## 8. 推荐阅读顺序

如果你第一次接触这个工作空间，建议按下面顺序看代码：

1. `rm_auto_aim/auto_aim_bringup/launch/auto_aim.launch.py`
2. `rm_communication/bubble_protocol/bubble_protocol/dispatch.py`
3. `rm_navi/rm_navigation/navi/launch/*.launch.py`
4. `rm_navi/rm_navigation/navi/params/nav2_params.yaml`
5. `rm_communication/bubble_decision/bubble_decision/`
6. `rmuc_bt_brain/src/bt_brain_node.cpp`
7. 根目录 `.sh` 启动脚本

这样最容易理解“现场实际怎么跑”。

## 9. 总结

当前 `HNU_NHS_SENTRY_UP-down` 不是单一算法仓库，而是一套面向哨兵机器人联调的 ROS 2 综合工作空间。它的特点是：

- 自瞄、通信、导航、决策都已经接在一起
- 现场脚本较多，适合快速起系统
- 代码里带有明显的比赛调参痕迹和机器相关路径
- 主链路已经可读且相对完整，但仍夹杂实验模块与历史遗留结构

如果你的目标是“理解当前仓库怎么工作”，重点看自瞄、通信、导航和 C++/Python 联合决策这四条主线即可。
