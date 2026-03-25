# 智能脱困快速开始指南

## 5分钟快速上手

### 1. 编译安装（2分钟）

```bash
# 进入你的ROS2工作空间
cd ~/your_ros2_ws/src

# 复制本包（假设已解压到Downloads）
cp -r ~/Downloads/smart_escape .

# 安装依赖
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --packages-select smart_escape

# source
source install/setup.bash
```

### 2. 测试脱困服务（1分钟）

确保你的机器人仿真或实际环境已经运行，然后：

```bash
# 终端1：启动脱困服务
ros2 run smart_escape smart_escape_server

# 终端2：发送测试请求
ros2 action send_goal /smart_escape nav2_msgs/action/SmartEscape {}
```

如果看到类似下面的输出，说明服务正常工作：
```
Result:
    success: true
    message: 'Escape target computed successfully'
    escape_pose:
      header:
        frame_id: map
      pose:
        position:
          x: 1.234
          y: 0.567
```

### 3. 集成到导航（2分钟）

修改你的导航启动文件，添加智能脱困：

**步骤1**：启动 `smart_escape_server`

```python
# 在你的navigation.launch.py中添加
smart_escape_server_node = Node(
    package='smart_escape',
    executable='smart_escape_server',
    name='smart_escape_server',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
)
```

**步骤2**：更新 lifecycle_manager

```python
lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'autostart': autostart,
        'node_names': [
            'controller_server',
            'planner_server', 
            'recoveries_server',
            'bt_navigator',
            'smart_escape_server',  # 添加这一行
        ]
    }]
)
```

**步骤3**：使用包含SmartEscape的行为树

```python
declare_bt_xml_cmd = DeclareLaunchArgument(
    'bt_xml_file',
    default_value=os.path.join(
        smart_escape_dir, 
        'behavior_trees', 
        'navigate_with_smart_escape.xml'  # 使用本包提供的行为树
    )
)
```

### 4. 启动完整导航

```bash
ros2 launch smart_escape navigation_with_smart_escape.launch.py
```

## 常见问题速查

### Q: 编译失败，找不到nav2_msgs/action/SmartEscape

**原因**：Action接口没有正确生成

**解决**：
```bash
cd ~/your_ros2_ws
rm -rf build/smart_escape install/smart_escape
colcon build --packages-select smart_escape --cmake-clean-cache
```

### Q: bt_navigator 报告找不到SmartEscape节点

**原因**：BT节点库没有正确加载

**解决**：
1. 检查 `nav2_params.yaml` 中的 `plugin_lib_names`：
```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - ...
      - smart_escape_nodes  # 确保包含这一行
```

2. 确保库文件存在：
```bash
ls ~/your_ros2_ws/install/smart_escape/lib/libsmart_escape_nodes.so
```

### Q: 脱困服务启动但收不到代价地图

**原因**：代价地图话题名称不匹配

**解决**：检查remapping：
```python
smart_escape_server_node = Node(
    ...
    remappings=[
        ('/global_costmap/costmap', '/your_costmap_topic'),
    ]
)
```

### Q: 脱困总是失败

**调试步骤**：

1. 检查代价地图是否正常：
```bash
ros2 topic echo /global_costmap/costmap/info
```

2. 调整参数：
```yaml
# config/smart_escape_params.yaml
smart_escape_server:
  ros__parameters:
    initial_radius: 0.3        # 减小初始半径
    max_radius: 3.0            # 增大最大半径
    min_free_cells_threshold: 5  # 减小阈值
```

3. 查看详细日志：
```bash
ros2 run smart_escape smart_escape_server --ros-args --log-level debug
```

## 验证脱困功能

### 方法1：使用测试脚本

```bash
ros2 run smart_escape test_smart_escape.py
```

### 方法2：可视化调试

```bash
# 终端1：启动可视化节点
ros2 run smart_escape visualize_escape.py

# 终端2：启动RViz
rviz2

# 在RViz中添加MarkerArray显示，订阅 /smart_escape/markers
```

### 方法3：手动触发

```bash
# 1. 让机器人导航到一个会被卡住的位置
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0}}}'

# 2. 当机器人卡住时，观察是否触发脱困
ros2 topic echo /smart_escape/result
```

## 下一步

- 阅读完整文档：[README.md](../README.md)
- 了解架构设计：[ARCHITECTURE.md](ARCHITECTURE.md)
- 调优参数：编辑 `config/smart_escape_params.yaml`
