# 智能脱困系统 - 实现总结

## 项目概述

本项目实现了基于**四川大学火锅战队2025年哨兵导航算法**的智能脱困系统，解决了使用插件模式导致的 `bt_navigator` 卡在 Activating 状态的问题。

## 核心创新

### 1. 架构创新：独立 Action Server 模式

**问题**：插件模式会阻塞 bt_navigator 启动  
**解决**：使用独立的 Action Server，BT 节点只做轻量级封装

```
传统插件模式：
bt_navigator ──► recoveries_server ──► 插件初始化（阻塞！）

本方案：
bt_navigator ──► BT节点（轻量）──Action──► smart_escape_server（独立）
```

### 2. 算法创新：基于代价地图质心的智能脱困

**传统方法**：随机旋转 + 后退（无方向性）  
**本方法**：分析代价地图 → 找到自由空间质心 → 有方向性地脱困

```
机器人被卡住
    │
    ├── 以机器人为中心搜索代价地图
    ├── 统计无障碍栅格
    ├── 计算质心（自由空间的"重心"）
    └── 朝向质心移动
        └── ✅ 有方向性的脱困！
```

## 文件清单

| 文件 | 作用 | 重要程度 |
|------|------|---------|
| `src/smart_escape_server.cpp` | 脱困算法实现 | ⭐⭐⭐⭐⭐ |
| `src/bt_smart_escape_node.cpp` | BT节点封装 | ⭐⭐⭐⭐⭐ |
| `behavior_trees/navigate_with_smart_escape.xml` | 行为树定义 | ⭐⭐⭐⭐⭐ |
| `launch/navigation_with_smart_escape.launch.py` | 完整导航启动 | ⭐⭐⭐⭐⭐ |
| `config/smart_escape_params.yaml` | 脱困参数 | ⭐⭐⭐⭐ |
| `action/SmartEscape.action` | Action接口 | ⭐⭐⭐⭐ |
| `CMakeLists.txt` / `package.xml` | 构建配置 | ⭐⭐⭐⭐ |

## 快速开始

```bash
# 1. 编译
cd ~/your_ros2_ws
colcon build --packages-select smart_escape
source install/setup.bash

# 2. 测试脱困服务
ros2 run smart_escape smart_escape_server
ros2 run smart_escape test_smart_escape.py

# 3. 启动完整导航
ros2 launch smart_escape navigation_with_smart_escape.launch.py
```

## 关键配置

### lifecycle_manager 必须包含 smart_escape_server

```python
lifecycle_manager_node = Node(
    parameters=[{
        'node_names': [
            'controller_server',
            'planner_server',
            'recoveries_server',
            'bt_navigator',
            'smart_escape_server',  # 必须包含！
        ]
    }]
)
```

### bt_navigator 必须加载 smart_escape_nodes 插件

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - ...
      - smart_escape_nodes  # 必须包含！
```

## 参数调优

编辑 `config/smart_escape_params.yaml`：

```yaml
smart_escape_server:
  ros__parameters:
    initial_radius: 0.5        # 初始搜索半径（米）
    max_radius: 2.0            # 最大搜索半径（米）
    radius_increment: 0.3      # 半径增量
    min_free_cells_threshold: 10   # 最小无障碍栅格数
    escape_distance: 0.3           # 脱困移动距离（米）
```

## 调试工具

| 工具 | 命令 | 用途 |
|------|------|------|
| 测试脚本 | `ros2 run smart_escape test_smart_escape.py` | 测试脱困功能 |
| 可视化 | `ros2 run smart_escape visualize_escape.py` | RViz显示脱困目标 |
| 日志 | `--ros-args --log-level debug` | 查看详细日志 |

## 与 Gemini 方案的区别

| 特性 | Gemini 方案 | 本方案 |
|------|------------|--------|
| 架构 | 插件模式 | 独立 Action Server |
| 是否阻塞 bt_navigator | ❌ 是 | ✅ 否 |
| 启动可靠性 | 低 | 高 |
| 计算位置 | BT 节点 | 独立进程 |
| 川大是否使用 | ❌ 否 | ✅ 是 |

## 为什么不会卡在 Activating？

```
Gemini 方案的问题：
bt_navigator Activating
    ├── 扫描 XML 节点
    ├── 发现自定义节点
    ├── 尝试与 recoveries_server 握手
    │       └── 插件初始化阻塞
    └── ❌ 卡住！

本方案：
bt_navigator Activating
    ├── 扫描 XML 节点
    ├── 发现 SmartEscape 节点
    ├── BT 节点轻量初始化（不阻塞）
    └── ✅ 成功启动！

smart_escape_server（独立进程）
    ├── 独立初始化
    └── 等待请求（不影响 bt_navigator）
```

## 后续优化方向

1. **多方向脱困**：计算多个候选方向，选择最优
2. **历史信息融合**：避免重复访问同一区域
3. **动态参数调整**：根据卡住频率自动调整参数
4. **机器学习**：使用RL学习最优脱困策略

## 参考

- 四川大学火锅战队2025年哨兵导航算法
- Nav2 Behavior Tree 文档
- ROS2 Action 设计模式

## 许可证

Apache-2.0
