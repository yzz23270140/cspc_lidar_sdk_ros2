from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('cspc_lidar')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'cspc_lidar.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(
        package='cspc_lidar',
        executable='cspc_lidar',        # 改为 executable（不是 node_executable）
        name='cspc_lidar',               # 改为 name（不是 node_name）
        namespace='',                    # 空字符串表示全局命名空间
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
    ])