import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'occlusion_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # URDFファイルのパス
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_holonomic_robot.urdf')
    
    # Worldファイルのパス (既存のものを使用)
    world_file = os.path.join(pkg_share, 'worlds', 'occlusion.world')

    return LaunchDescription([
        # 1. Gazeboの起動
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. ロボットのSpawn (Gazeboへのモデル投入)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),
        
        # (オプション) TFの配信のために state_publisher を起動
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
    ])