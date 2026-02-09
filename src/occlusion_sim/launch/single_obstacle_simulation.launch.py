import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('occlusion_sim')
    urdf = os.path.join(pkg, 'urdf', 'simple_holonomic_robot.urdf')
    world = os.path.join(pkg, 'worlds', 'multi_obstacle.world')
    rviz_config = os.path.join(pkg, 'rviz', 'sensor_viz.rviz')

    # 障害物スポーン位置: スタート(1.0, 7.5)とゴール(20.0, 7.5)の中心
    obs_name = 'obs_0'
    obs_x, obs_y = 10.5, 7.5

    # SDF テンプレート置換 (namespace を obs_0 に設定)
    sdf = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
    with open(sdf, 'r') as f:
        sdf_content = f.read().replace('/obs_{id}', f'/{obs_name}')
    tmp_sdf = f'/tmp/{obs_name}.sdf'
    with open(tmp_sdf, 'w') as f:
        f.write(sdf_content)

    return LaunchDescription([
        # Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Ego robot at start position (1.0, 7.5)
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'ego_robot', '-file', urdf,
                        '-x', '1.0', '-y', '7.5', '-z', '0.2'],
             output='screen'),
        # Single dynamic obstacle at center (10.5, 7.5)
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', obs_name, '-file', tmp_sdf,
                        '-x', str(obs_x), '-y', str(obs_y), '-z', '0.1'],
             output='screen'),
        # Obstacle controller (chase ego robot)
        Node(package='occlusion_sim', executable='single_obstacle_controller.py',
             output='screen'),
        # CBF safe controller
        Node(package='occlusion_sim', executable='cbf_wrapper_node.py',
             output='screen'),
        # Sensor visualizer
        Node(package='occlusion_sim', executable='sensor_visualizer_node.py',
             output='screen'),
        # RViz2
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config], output='screen'),
    ])
