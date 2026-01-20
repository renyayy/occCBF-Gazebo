import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('occlusion_sim')

    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_holonomic_robot.urdf')
    obs_sdf_file = os.path.join(pkg_share, 'models', 'moving_cylinder.sdf')
    world_file = os.path.join(pkg_share, 'worlds', 'simple.world')

    return LaunchDescription([
        # Gazebo起動
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Ego Robot spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        # Moving Obstacle spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'moving_cylinder', '-file', obs_sdf_file, '-x', '3.0', '-y', '-3.0', '-z', '0.1'],
            output='screen'
        ),

        # 障害物コントローラー
        Node(
            package='occlusion_sim',
            executable='obstacle_controller.py',
            output='screen'
        ),

        # CBF制御ノード
        Node(
            package='occlusion_sim',
            executable='cbf_wrapper_node.py',
            output='screen'
        ),
    ])
