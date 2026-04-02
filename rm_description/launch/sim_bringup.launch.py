import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 获取功能包安装后的共享目录
    share_dir = get_package_share_directory('rm_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # 🌟 解决卡死的核心：把本地 models 目录加入 Gazebo 搜索路径
    models_dir = os.path.join(share_dir, 'models')
    
    # 指定要加载的比赛场地 world 文件 (这里以 rmuc_2025_world.sdf 为例，你可以按需更改)
    world_file = os.path.join(share_dir, 'world', 'rmul_2025_world.sdf')
    
    # 读取配置文件参数
    config_file = os.path.join(share_dir, 'config', 'launch_params.yaml')
    with open(config_file, 'r') as f:
        launch_params = yaml.safe_load(f)
    xyz_param = launch_params['odom2camera']['xyz'].replace('"', '')
    rpy_param = launch_params['odom2camera']['rpy'].replace('"', '')

    # 定位仿真的顶层 URDF 文件
    xacro_file = os.path.join(share_dir, 'urdf', 'sentry_sim_top.urdf.xacro')
    
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' xyz:="' + xyz_param + '"', 
        ' rpy:="' + rpy_param + '"'
    ])

    # --- 启动节点定义 ---
    
    # 1. 注入环境变量
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', models_dir]
    )

    # 2. 发布 TF 和 机器人模型状态
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content, 
            'use_sim_time': True 
        }]
    )

    # 3. 启动 Gazebo 服务端并加载指定的比赛场地
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # 4. 启动 Gazebo 客户端 (图形界面)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # 5. 将哨兵生成到世界中
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sentry_robot',
            '-x', '14.5',  # 对应场地模型的位移中心
            '-y', '8.0',   # 对应场地模型的位移中心
            '-z', '0.5'  
        ],
        output='screen'
    )
    
    # 6. 静态 TF (保持你们原有的雷达补丁)
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0', '0', '0.25', '0', '0', '0', 'base_footprint', 'sentry/base_footprint/lidar']
    )

    return LaunchDescription([
        set_gazebo_model_path,
        rsp_node,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        lidar_static_tf
    ])