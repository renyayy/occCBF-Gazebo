import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'occlusion_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # 各種ファイルのパス
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_holonomic_robot.urdf')
    obs_sdf_file = os.path.join(pkg_share, 'models', 'moving_cylinder.sdf')
    world_file = os.path.join(pkg_share, 'worlds', 'occlusion.world')

    return LaunchDescription([
        # 1. Gazeboの起動
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Ego Robot (自分) のSpawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        # 3. Moving Obstacle (障害物) のSpawn
        # x=3.0, y=-3.0 の位置に出現させ、y=3.0まで移動
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'moving_cylinder', '-file', obs_sdf_file, '-x', '3.0', '-y', '-3.0', '-z', '0.1'],
            output='screen'
        ),

        # 4. 障害物を動かすノード
        Node(
            package='occlusion_sim',
            executable='obstacle_controller.py',
            output='screen'
        ),

        # 5. CBF制御ノード (少し待ってから起動するとログが見やすいが、同時でもOK)
        Node(
            package='occlusion_sim',
            executable='cbf_wrapper_node.py', # または .pyなし
            output='screen',
            parameters=[{'goal_x': 5.0}, {'goal_y': 0.0}] # 必要ならパラメータ化
        ),
    ])
