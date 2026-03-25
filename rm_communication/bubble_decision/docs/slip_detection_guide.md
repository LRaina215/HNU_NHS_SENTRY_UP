# 打滑检测模块使用指南

## 概述

打滑检测模块为机器人系统提供了实时的打滑检测和响应能力。通过比较目标速度和实际速度的差异，系统能够检测到打滑情况并采取相应的控制策略。

## 核心组件

### 1. 打滑检测节点 (SlipDetection)

**主要功能：**
- 订阅底盘控制消息 (`/chassis`)
- 订阅里程计消息 (`/odom`)
- 计算打滑率和打滑严重程度
- 发布打滑状态 (`/slip_status`, `/slip_ratio`, `/slip_severity`)

**关键参数：**
- `slip_threshold`: 打滑阈值 (默认: 0.3)
- `window_size`: 滑动窗口大小 (默认: 10)
- `min_speed_threshold`: 最小速度阈值 (默认: 0.1 m/s)

### 2. 打滑响应模块 (SlipResponse)

**主要功能：**
- 根据打滑状态自动调整底盘控制命令
- 在打滑时降低机器人速度
- 提供平滑的恢复机制

### 3. 打滑感知游戏动作 (SlipAwareGameAction)

**主要功能：**
- 集成到现有决策系统中
- 提供打滑感知的移动决策
- 根据打滑严重程度调整行为优先级

## 安装和配置

### 1. 构建包

```bash
cd /path/to/your/workspace
colcon build --packages-select bubble_decision
source install/setup.bash
```

### 2. 运行打滑检测节点

**方法一：使用启动文件**
```bash
ros2 launch bubble_decision slip_detection_launch.py
```

**方法二：直接运行节点**
```bash
ros2 run bubble_decision slip_detection
```

### 3. 配置参数

可以通过启动文件或运行时参数调整：

```bash
ros2 run bubble_decision slip_detection --ros-args -p slip_threshold:=0.4 -p window_size:=15
```

## 集成到现有系统

### 1. 在决策系统中集成

在现有的 `GameAction` 类中添加打滑感知：

```python
from bubble_decision.slip_integration import SlipAwareGameAction

class EnhancedGameAction(GameAction):
    def __init__(self, node: Node) -> None:
        super().__init__(node)
        self.slip_aware = SlipAwareGameAction(node, self)
    
    def enhanced_movement_decision(self):
        if self.slip_aware.should_avoid_movement():
            # 避免在严重打滑时移动
            return
        
        speed_factor = self.slip_aware.get_safe_speed_factor()
        # 使用安全速度因子调整移动
```

### 2. 在控制系统中集成

```python
from bubble_decision.slip_integration import SlipResponse

class EnhancedController:
    def __init__(self):
        self.slip_response = SlipResponse()
    
    def send_chassis_command(self, chassis_msg):
        # 根据打滑状态调整控制命令
        adjusted_msg = self.slip_response.adjust_chassis_command(chassis_msg)
        self.chassis_pub.publish(adjusted_msg)
```

## 话题接口

### 订阅的话题

- `/chassis` (rmctrl_msgs/Chassis): 底盘目标控制
- `/odom` (rmctrl_msgs/Odom): 里程计数据

### 发布的话题

- `/slip_status` (std_msgs/Bool): 打滑状态 (True/False)
- `/slip_ratio` (std_msgs/Float32): 打滑率 (0-1)
- `/slip_severity` (std_msgs/Float32): 打滑严重程度 (0-1)

## 打滑检测算法

### 打滑率计算

```
打滑率 = max(0, 1 - (实际速度 / 目标速度))
```

- 打滑率 = 0: 无打滑
- 打滑率 = 0.5: 50%的速度损失
- 打滑率 = 1: 完全打滑（目标速度但实际速度为0）

### 打滑判断

```
打滑状态 = 打滑率 > 打滑阈值
```

### 打滑严重程度

```
打滑严重程度 = min(1.0, 打滑率 / 打滑阈值)
```

## 调试和监控

### 1. 查看打滑状态

```bash
ros2 topic echo /slip_status
ros2 topic echo /slip_ratio
ros2 topic echo /slip_severity
```

### 2. 实时监控

```bash
# 查看所有打滑相关话题
ros2 topic list | grep slip

# 查看节点状态
ros2 node list
ros2 node info /slip_detection
```

### 3. 参数调整

```bash
# 查看当前参数
ros2 param get /slip_detection slip_threshold

# 动态调整参数
ros2 param set /slip_detection slip_threshold 0.4
```

## 故障排除

### 常见问题

1. **没有检测到打滑**
   - 检查 `/chassis` 和 `/odom` 话题是否有数据
   - 确认速度阈值设置是否合适
   - 检查打滑阈值是否设置过高

2. **误检测打滑**
   - 降低打滑阈值
   - 增加滑动窗口大小
   - 调整最小速度阈值

3. **响应延迟**
   - 减少滑动窗口大小
   - 检查系统负载
   - 确认话题发布频率

### 性能优化建议

1. **参数调优**
   - 在平滑地面：使用较低的阈值 (0.2-0.3)
   - 在粗糙地面：使用较高的阈值 (0.4-0.5)
   - 高速运动：增加窗口大小 (15-20)

2. **传感器融合**
   - 集成IMU数据检测异常加速度
   - 使用视觉数据识别地面条件
   - 结合轮速编码器数据

## 扩展功能

### 1. 高级打滑检测

```python
from bubble_decision.slip_detection import AdvancedSlipDetection

# 使用包含更多传感器融合的高级版本
advanced_detection = AdvancedSlipDetection()
```

### 2. 自定义响应策略

```python
class CustomSlipResponse(SlipResponse):
    def custom_recovery_strategy(self):
        # 实现自定义的恢复策略
        pass
```

## 安全注意事项

1. **紧急停止**: 在严重打滑时考虑实现紧急停止机制
2. **渐进恢复**: 使用渐进式速度恢复避免二次打滑
3. **地面适应**: 根据地面条件动态调整参数
4. **系统监控**: 定期检查打滑检测系统的健康状态

## 版本历史

- v1.0.0 (2025-11-09): 初始版本发布
  - 基础打滑检测功能
  - 打滑响应机制
  - 集成接口
