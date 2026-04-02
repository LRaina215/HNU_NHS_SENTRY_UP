# 智能脱困导航系统 (Smart Escape Navigation)

基于**四川大学火锅战队2025年哨兵导航算法**的智能脱困系统。

## 算法原理

当机器人被障碍物卡住时，系统会：

1. **以机器人为中心**，在一定半径范围内搜索代价地图
2. **统计无障碍栅格**（代价为0的栅格）数量
3. **动态调整搜索半径**：若无障碍栅格少于阈值，则扩大半径继续搜索
4. **计算无障碍区域的质心**（平均坐标）
5. **生成脱困目标点**：朝向质心方向移动一定距离

![脱困原理](docs/escape_principle.png)

## 为什么不会卡在 bt_navigator Activating？

### 问题根源（Gemini分析正确）

当你使用**插件模式**（Plugin）时：
- bt_navigator 在 Activating 时会扫描 XML 中的所有节点
- 发现 `<BackUp>` 等自定义节点时，会立即尝试与 recoveries_server 握手
- 如果插件初始化耗时，会导致超时

### 解决方案：独立 Action Server 模式

本系统采用**独立的 Action Server 架构**：

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   bt_navigator  │────▶│  SmartEscape BT  │────▶│  smart_escape   │
│   (Activating)  │     │    Node (轻量)    │     │   Server (独立)  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                              │                            │
                              │ 异步Action调用              │ 访问代价地图
                              │                            │
                              ▼                            ▼
                        ┌──────────────────┐     ┌─────────────────┐
                        │   不阻塞启动      │     │  计算脱困目标    │
                        └──────────────────┘     └─────────────────┘
```

**关键优势**：
- BT节点只负责**轻量级的Action调用**
- 重型计算在**独立进程**中完成
- bt_navigator 启动时**不需要等待** smart_escape_server 初始化

## 文件结构

```
smart_escape/
├── action/
│   └── SmartEscape.action          # Action接口定义
├── behavior_trees/
│   └── navigate_with_smart_escape.xml  # 行为树XML
├── config/
│   ├── smart_escape_params.yaml    # 脱困参数
│   └── nav2_params.yaml            # Nav2参数
├── include/smart_escape/
│   ├── smart_escape_server.hpp     # Action Server头文件
│   └── bt_smart_escape_node.hpp    # BT节点头文件
├── launch/
│   ├── smart_escape.launch.py      # 单独启动脱困服务
│   └── navigation_with_smart_escape.launch.py  # 完整导航
├── src/
│   ├── smart_escape_server.cpp     # 脱困算法实现
│   └── bt_smart_escape_node.cpp    # BT节点实现
├── CMakeLists.txt
└── package.xml
```

## 编译安装

```bash
# 1. 将本包放入你的ROS2工作空间
cd ~/your_ros2_ws/src
cp -r /path/to/smart_escape .

# 2. 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译
cd ~/your_ros2_ws
colcon build --packages-select smart_escape

# 4.  source
source install/setup.bash
```

## 使用方法

### 方法1：启动完整导航系统（推荐）

```bash
ros2 launch smart_escape navigation_with_smart_escape.launch.py
```

这会启动：
- smart_escape_server（智能脱困服务）
- Nav2 完整导航栈
- 配置好的行为树（包含 SmartEscape 节点）

### 方法2：单独启动脱困服务

如果你已经有 Nav2 在运行：

```bash
ros2 launch smart_escape smart_escape.launch.py
```

### 方法3：测试脱困功能

```bash
# 终端1：启动脱困服务
ros2 run smart_escape smart_escape_server

# 终端2：发送测试请求
ros2 action send_goal /smart_escape nav2_msgs/action/SmartEscape {}
```

## 参数配置

编辑 `config/smart_escape_params.yaml`：

```yaml
smart_escape_server:
  ros__parameters:
    # 搜索半径参数
    initial_radius: 0.5        # 初始搜索半径（米）
    max_radius: 2.0            # 最大搜索半径（米）
    radius_increment: 0.3      # 半径增量
    
    # 脱困参数
    min_free_cells_threshold: 10   # 最小无障碍栅格数
    escape_distance: 0.3           # 脱困移动距离（米）
```

### 参数调优建议

| 参数 | 默认值 | 适用场景 |
|------|--------|----------|
| `initial_radius` | 0.5m | 狭窄环境可减小，开阔环境可增大 |
| `max_radius` | 2.0m | 根据地图大小调整 |
| `min_free_cells_threshold` | 10 | 栅格分辨率0.05m时，10个栅格=直径0.5m的圆 |
| `escape_distance` | 0.3m | 机器人直径的1-1.5倍 |

## 行为树节点使用

在行为树XML中使用：

```xml
<Sequence name="RecoveryActions">
  <!-- 智能脱困 -->
  <SmartEscape 
    name="SmartEscapeAction"
    server_name="smart_escape"
    server_timeout="10000"
    escape_pose="{escape_pose}"
    result_message="{escape_result_msg}"/>
  
  <!-- 移动到脱困目标点 -->
  <ComputePathToPose goal="{escape_pose}" path="{escape_path}" planner_id="GridBased"/>
  <FollowPath path="{escape_path}" controller_id="FollowPath"/>
</Sequence>
```

## 调试与日志

查看脱困服务日志：

```bash
# 查看实时日志
ros2 topic echo /smart_escape/feedback

# 查看代价地图
ros2 topic echo /global_costmap/costmap

# 查看脱困目标点
ros2 topic echo /escape_pose  # 需要在代码中添加发布
```

## 常见问题

### Q: bt_navigator 还是卡在 Activating？

**检查清单**：
1. 确保 `smart_escape_server` 在 `lifecycle_manager` 的 `node_names` 列表中
2. 检查 `plugin_lib_names` 是否正确包含 `smart_escape_nodes`
3. 查看日志：`ros2 launch` 时添加 `--debug` 参数

### Q: 脱困失败，提示 "Not enough free cells"？

**解决方案**：
- 增大 `max_radius` 参数
- 减小 `min_free_cells_threshold` 参数
- 检查代价地图是否正确发布

### Q: 脱困方向不正确？

**可能原因**：
- 代价地图膨胀层设置过大
- 机器人被完全包围，没有真正的"自由空间"

## 算法对比

| 特性 | Nav2默认恢复 | 本智能脱困 |
|------|-------------|-----------|
| 恢复策略 | 固定模式（旋转→后退） | 基于地图分析的智能决策 |
| 脱困方向 | 随机/固定 | 朝向无障碍区域质心 |
| 成功率 | 依赖运气 | 更高（有方向性） |
| 计算开销 | 低 | 中等 |

## 参考

- 四川大学火锅战队2025年哨兵导航算法
- Nav2 Behavior Tree 文档: https://navigation.ros.org/behavior_trees/

## License

Apache-2.0
