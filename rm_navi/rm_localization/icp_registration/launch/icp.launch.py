import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('icp_registration'), 'launch'))


def generate_launch_description():
  from launch_ros.actions import Node
  from launch import LaunchDescription

  icp_share_dir = get_package_share_directory('icp_registration')
  point_lio_share_dir = get_package_share_directory('point_lio')
  params = os.path.join(icp_share_dir, 'config', 'icp.yaml')

  installed_pcd_path = os.path.join(point_lio_share_dir, 'PCD', 'scans.pcd')
  source_pcd_path = os.path.abspath(
      os.path.join(point_lio_share_dir, '..', '..', '..', 'src', 'rm_navi', 'rm_localization', 'point_lio', 'PCD', 'scans.pcd'))
  pcd_path = installed_pcd_path if os.path.exists(installed_pcd_path) else source_pcd_path

  node = Node(
    package='icp_registration',
    executable='icp_registration_node',
    output='screen',
    parameters=[params, {'pcd_path': pcd_path}]
  )

  return LaunchDescription([node])
