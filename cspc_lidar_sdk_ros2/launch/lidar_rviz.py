import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的 share 目录（推荐方式）
    pkg_share = get_package_share_directory('your_package_name')
    
    # RViz 配置文件路径
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    
    # 或者如果放在 launch 同级目录：
    # rviz_config = os.path.join(pkg_share, 'launch', 'rviz.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([rviz_node])