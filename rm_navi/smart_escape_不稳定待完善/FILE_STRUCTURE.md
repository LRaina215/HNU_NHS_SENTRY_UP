# 文件结构说明

## 完整文件列表

```
smart_escape/
├── action/
│   └── SmartEscape.action          # Action接口定义
│                                   # - Goal: 空（脱困不需要输入参数）
│                                   # - Result: success, message, escape_pose
│                                   # - Feedback: status, progress
│
├── behavior_trees/
│   └── navigate_with_smart_escape.xml  # 行为树XML
│                                       # - MainTree: 基本导航+脱困
│                                       # - NavigateWithFullRecovery: 完整恢复
│                                       # - SmartEscapeOnly: 仅脱困（测试用）
│                                       # - NavigateWithRetry: 带重试的导航
│
├── config/
│   ├── smart_escape_params.yaml    # 智能脱困参数
│   │                               # - 搜索半径参数
│   │                               # - 脱困距离参数
│   │                               # - TF参数
│   │
│   └── nav2_params.yaml            # Nav2完整参数
│                                   # - 控制器参数
│                                   # - 规划器参数
│                                   # - 代价地图参数
│                                   # - BT插件列表（包含smart_escape_nodes）
│
├── docs/
│   ├── ARCHITECTURE.md             # 架构设计文档
│   │                               # - 系统架构图
│   │                               # - 设计决策说明
│   │                               # - 通信流程
│   │
│   ├── COMPARISON.md               # 方案对比文档
│   │                               # - 与Gemini方案对比
│   │                               # - 为什么不会卡在Activating
│   │
│   ├── QUICKSTART.md               # 快速开始指南
│   │                               # - 5分钟上手
│   │                               # - 常见问题速查
│   │
│   └── escape_principle.png        # 脱困原理图（需要用户自己添加）
│
├── include/smart_escape/
│   ├── smart_escape_server.hpp     # Action Server头文件
│   │                               # - SmartEscapeServer类
│   │                               # - Action回调函数声明
│   │                               # - 脱困算法函数声明
│   │
│   └── bt_smart_escape_node.hpp    # BT节点头文件
│                                   # - BtSmartEscapeNode类
│                                   # - 端口定义
│                                   # - 状态机接口
│
├── launch/
│   ├── smart_escape.launch.py      # 单独启动脱困服务
│   │                               # - 只启动smart_escape_server
│   │                               # - 用于测试或与现有Nav2集成
│   │
│   └── navigation_with_smart_escape.launch.py  # 完整导航启动
│                                               # - 启动所有Nav2节点
│                                               # - 包含smart_escape_server
│                                               # - 包含lifecycle_manager
│
├── scripts/
│   ├── test_smart_escape.py        # 脱困功能测试脚本
│   │                               # - 发送Action请求
│   │                               # - 打印结果
│   │
│   └── visualize_escape.py         # 可视化调试脚本
│                                   # - 在RViz中显示脱困目标
│                                   # - 显示搜索区域
│
├── src/
│   ├── smart_escape_server.cpp     # Action Server实现
│   │                               # - 核心算法: compute_escape_target()
│   │                               # - 代价地图处理
│   │                               # - TF变换
│   │
│   └── bt_smart_escape_node.cpp    # BT节点实现
│                                   # - Action客户端封装
│                                   # - 状态机实现
│                                   # - 节点注册
│
├── CMakeLists.txt                  # CMake构建配置
│                                   # - 依赖查找
│                                   # - 库编译
│                                   # - 节点注册
│                                   # - 安装规则
│
├── package.xml                     # ROS2包配置
│                                   # - 包信息
│                                   # - 依赖声明
│                                   # - 成员组
│
├── README.md                       # 主文档
│                                   # - 项目介绍
│                                   # - 使用方法
│                                   # - 参数说明
│
└── FILE_STRUCTURE.md               # 本文件
```

## 核心文件详解

### 1. 算法实现文件

#### `src/smart_escape_server.cpp`

**核心函数**：`compute_escape_target()`

```cpp
// 川大火锅战队脱困算法
bool SmartEscapeServer::compute_escape_target(...)
{
    // 1. 以机器人为中心，在一定半径圆内搜索
    // 2. 统计无障碍栅格（cost == 0）
    // 3. 若无障碍栅格 < 阈值，扩大半径
    // 4. 计算无障碍栅格的质心
    // 5. 生成脱困目标点
}
```

**关键参数**：
- `initial_radius`: 初始搜索半径（默认0.5m）
- `max_radius`: 最大搜索半径（默认2.0m）
- `min_free_cells_threshold`: 最小无障碍栅格数（默认10）
- `escape_distance`: 脱困移动距离（默认0.3m）

### 2. 行为树文件

#### `behavior_trees/navigate_with_smart_escape.xml`

**行为树结构**：
```
RecoveryNode (number_of_retries=6)
├── Sequence (正常导航)
│   ├── ComputePathToPose
│   └── FollowPath
└── SequenceStar (恢复行为)
    └── SmartEscape (智能脱困)
```

**工作原理**：
1. 正常导航失败时触发恢复
2. 首先尝试智能脱困
3. 脱困成功后重新导航

### 3. 启动文件

#### `launch/navigation_with_smart_escape.launch.py`

**启动顺序**（重要！）：
1. `smart_escape_server` - 必须先启动
2. `global_costmap` / `local_costmap`
3. `controller_server`
4. `planner_server`
5. `recoveries_server`
6. `bt_navigator`
7. `lifecycle_manager` - 最后启动，管理所有节点

**关键配置**：
```python
# lifecycle_manager必须包含smart_escape_server
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

## 修改指南

### 修改脱困算法

编辑 `src/smart_escape_server.cpp` 中的 `compute_escape_target()` 函数。

### 修改行为树

编辑 `behavior_trees/navigate_with_smart_escape.xml`。

### 修改参数

编辑 `config/smart_escape_params.yaml`。

### 添加新的恢复行为

1. 在 `behavior_trees/navigate_with_smart_escape.xml` 中添加新节点
2. 在 `RecoveryNode` 的恢复序列中添加

## 调试文件

### 日志输出

```bash
# 查看smart_escape_server日志
ros2 run smart_escape smart_escape_server --ros-args --log-level debug

# 查看所有相关日志
ros2 topic echo /rosout | grep smart_escape
```

### 可视化

```bash
# 启动可视化节点
ros2 run smart_escape visualize_escape.py

# 启动RViz
rviz2
# 添加MarkerArray显示，订阅 /smart_escape/markers
```

### 测试

```bash
# 发送测试请求
ros2 run smart_escape test_smart_escape.py

# 或手动发送
ros2 action send_goal /smart_escape nav2_msgs/action/SmartEscape {}
```
