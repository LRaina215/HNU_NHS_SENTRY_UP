# 导航工作流与使用说明

## 1. 当前工作空间检查结论

截至本次检查，当前工作空间下的导航链路已经满足“可构建、可解析、可启动脚本”的条件：

- `pre.sh`、`nav.sh` 语法正确。
- `gnome-terminal` 可正常启动，当前图形显示为 `:0`。
- 工作空间已成功 `colcon build`，`install/setup.bash` 已生成。
- 以下 launch 均可在当前安装空间内被 `ros2 launch -s` 正确解析：
  - `livox_ros_driver2 msg_MID360_cloud_launch.py`
  - `rm_description model.launch.py`
  - `point_lio mapping_mid360.launch.py`
  - `bubble_protocol sentry_launch.py`
  - `linefit_ground_segmentation_ros segmentation.launch.py`
  - `pointcloud_to_laserscan pointcloud_to_laserscan_launch.py`
  - `icp_registration icp.launch.py`
  - `navi localization_launch.py`
  - `navi navigation_launch.py`
  - `navi rviz_launch.py`

本次还修正了两个会导致换机后直接失败的问题：

- `nav.sh` 与 `pre.sh` 现在会自动定位工作空间根目录，并在 `install/setup.bash` 缺失时直接报错提示构建。
- `navigation_launch.py` 和 `icp.launch.py` 已改为基于 package share 动态查找行为树 XML 与 PCD 地图，不再依赖 `/home/robomaster/...`。

## 2. 导航整体工作流

这套导航分成两段启动：

1. `pre.sh`
2. `nav.sh`

顺序必须是先 `pre.sh`，再 `nav.sh`。

### 2.1 `pre.sh` 负责什么

`pre.sh` 是导航前置链路，负责把原始传感器数据和底层导航输入准备好。

启动项如下：

- `ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py`
  - 启动 Livox MID360 雷达驱动。
  - 提供原始点云输入，是整个导航链路的源头。

- `ros2 launch rm_description model.launch.py`
  - 发布机器人模型与关键 TF。
  - 当前 launch 中还补了几条静态 TF，用来把 `map`、`odom`、底盘、云台、雷达坐标系缝合起来。

- `ros2 launch point_lio mapping_mid360.launch.py`
  - 启动 Point-LIO。
  - 使用 LiDAR 点云做里程计/建图相关处理，并启动 `rm_lidar_filter`。

- `ros2 launch bubble_protocol sentry_launch.py`
  - 启动底盘通信节点。
  - 默认串口参数是 `/dev/ttyCBoard`，负责机器人底层通信。

- `ros2 launch linefit_ground_segmentation_ros segmentation.launch.py`
  - 做地面/障碍物分割。
  - 当前配置把 `/livox/lidar_no_body` 作为输入，输出 `/segmentation/obstacle` 和 `/segmentation/ground`。

- `ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py`
  - 把障碍物点云 `/segmentation/obstacle` 转成 `Nav2` 需要的 `/scan`。

`pre.sh` 的产出可以概括为：

- 原始雷达数据
- TF 树
- 里程计/点云定位基础
- 障碍物分割结果
- 2D 激光话题 `/scan`

### 2.2 `nav.sh` 负责什么

`nav.sh` 是导航上层链路，负责把定位、地图和 Nav2 真正拉起来。

启动项如下：

- `ros2 launch icp_registration icp.launch.py`
  - 用 ICP 做 3D 点云到离线 PCD 地图的配准。
  - 提供 `map -> odom` 相关定位能力。

- `ros2 launch navi localization_launch.py`
  - 启动 `map_server` 与 `lifecycle_manager_localization`。
  - 当前明确移除了 AMCL，避免与 ICP 冲突。

- `ros2 launch navi navigation_launch.py`
  - 启动 `controller_server`、`planner_server`、`recoveries_server`、`bt_navigator`、`waypoint_follower`。
  - 这是完整的 Nav2 运行栈。

- `ros2 launch navi rviz_launch.py`
  - 打开 RViz 调试界面。

### 2.3 数据流怎么串起来

整体数据流如下：

1. MID360 输出原始点云。
2. Point-LIO 消化点云，给出点云运动估计与处理后的点云流。
3. 地面分割从点云中提取障碍物，输出 `/segmentation/obstacle`。
4. `pointcloud_to_laserscan` 把障碍物点云转成 `/scan`。
5. ICP 使用实时点云和离线 PCD 地图做全局配准，建立 `map` 系定位。
6. `map_server` 提供 2D 栅格地图。
7. Nav2 使用 `/scan`、`map`、TF、里程计与定位结果进行规划和控制。
8. RViz 负责可视化目标点、地图、路径与局部状态。

## 3. 使用前提

运行前请确认：

- ROS 2 Galactic 环境可用。
- 已在工作空间根目录完成构建。
- 图形桌面可用，因为脚本依赖 `gnome-terminal` 和 RViz。
- MID360 已连接并可被 Livox 驱动访问。
- 底盘串口设备 `/dev/ttyCBoard` 存在，且当前用户有权限访问。
- 机器人 TF 命名与当前 URDF/静态 TF 配置一致。
- ICP 所需 PCD 地图 `point_lio/PCD/scans.pcd` 存在。
- Nav2 栅格地图 YAML 存在，当前默认是 `navi/maps/nine.yaml`。

## 4. 标准使用步骤

### 4.1 构建工作空间

在工作空间根目录执行：

```bash
cd /home/lraina/shaobing_up
colcon build
source install/setup.bash
```

如果只想复用当前构建结果，也至少要先：

```bash
cd /home/lraina/shaobing_up
source install/setup.bash
```

### 4.2 启动前置链路

```bash
cd /home/lraina/shaobing_up/src
./pre.sh
```

执行后会弹出多个 `gnome-terminal` 窗口，每个窗口对应一个 ROS 2 launch。

建议观察：

- Livox 驱动是否正常连上雷达。
- `bubble_protocol` 是否成功打开 `/dev/ttyCBoard`。
- Point-LIO 是否持续输出，不报传感器或 TF 错误。
- 地面分割与 `pointcloud_to_laserscan` 是否开始发布数据。

### 4.3 启动导航链路

确认 `pre.sh` 相关节点稳定后，再执行：

```bash
cd /home/lraina/shaobing_up/src
./nav.sh
```

执行后会再弹出 4 个终端窗口，分别用于：

- ICP 定位
- 地图服务器
- Nav2
- RViz

### 4.4 在 RViz 中使用导航

一般流程如下：

1. 打开 RViz 后先确认 `TF`、`Map`、`LaserScan`、`Path` 等显示项正常。
2. 检查机器人位姿是否与地图对齐。
3. 使用 `2D Goal Pose` 或相应导航工具下发目标点。
4. 观察全局路径、局部轨迹和速度输出是否合理。

## 5. 常用检查命令

### 5.1 查看关键话题

```bash
source /home/lraina/shaobing_up/install/setup.bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /tf
ros2 topic echo /cmd_vel_nav
```

### 5.2 查看 TF 关系

```bash
source /home/lraina/shaobing_up/install/setup.bash
ros2 run tf2_tools view_frames
```

### 5.3 只检查 launch 能否被解析

```bash
source /home/lraina/shaobing_up/install/setup.bash
ros2 launch icp_registration icp.launch.py -s
ros2 launch navi localization_launch.py -s
ros2 launch navi navigation_launch.py -s
```

## 6. 可配置项

### 6.1 切换障碍物分割方案

`pre.sh` 当前启用的是 Linefit 方案：

```bash
ros2 launch linefit_ground_segmentation_ros segmentation.launch.py
```

如果你要改成 Terrain Analysis，则在 `pre.sh` 里：

- 注释掉 Linefit 那行
- 取消注释 `terrain_analysis terrain_analysis.launch`

### 6.2 切换导航地图

默认地图由 `localization_launch.py` 指向：

- `navi/maps/nine.yaml`

你可以手动指定：

```bash
source /home/lraina/shaobing_up/install/setup.bash
ros2 launch navi localization_launch.py map:=/绝对路径/你的地图.yaml
```

### 6.3 切换 ICP 地图

ICP 默认使用：

- `point_lio/PCD/scans.pcd`

如果要替换，请修改：

- `rm_navi/rm_localization/icp_registration/launch/icp.launch.py`

或在后续版本里扩展成 launch 参数。

## 7. 常见故障与排查

### 7.1 脚本提示缺少 `install/setup.bash`

原因：工作空间还没构建。

处理：

```bash
cd /home/lraina/shaobing_up
colcon build
```

### 7.2 `gnome-terminal` 启不来

原因通常是：

- 当前不在图形桌面
- `DISPLAY` 未设置
- 远程会话没有 X 转发

处理：

- 在本地图形桌面运行
- 确认 `echo $DISPLAY` 有值
- 必要时改写脚本为 `tmux` 或单终端后台运行模式

### 7.3 底盘串口打不开

检查：

```bash
ls -l /dev/ttyCBoard
```

确认：

- 设备节点存在
- 当前用户有权限
- 串口没被其他程序占用

### 7.4 RViz 有地图但机器人不动

重点检查：

- `/tf` 是否完整
- `/scan` 是否有数据
- `map -> odom -> base_link/base_footprint` 是否连通
- ICP 是否正常输出定位
- `cmd_vel_nav` 是否有输出但未下发到底盘

### 7.5 Nav2 报定位异常或路径规划失败

重点检查：

- ICP 配准是否稳定
- 栅格地图与实际环境是否匹配
- 机器人坐标系名字是否和参数文件一致
- 局部代价地图是否收到 `/scan`

## 8. 当前仍需注意的点

以下内容不影响当前 `pre.sh` / `nav.sh` 主流程，但需要你知情：

- `pcd2pgm` 相关配置里仍有旧的绝对路径硬编码，如果后续要使用离线点云转栅格地图工具，需要单独再改。
- 本次验证确认的是“工作空间可构建、launch 可解析、图形终端可拉起”。
- 真正的实机导航表现仍然受雷达连接、串口状态、TF 正确性、地图质量和底盘控制链路影响。

## 9. 推荐启动顺序总结

```bash
cd /home/lraina/shaobing_up
colcon build
source install/setup.bash

cd /home/lraina/shaobing_up/src
./pre.sh
./nav.sh
```
