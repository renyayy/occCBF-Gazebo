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

    # Obstacle positions (safe_control/dynamic_env準拠: マップ中央〜右側)
    obstacles = [
        {'name': 'obs_0', 'x': 8.0, 'y': 5.0},
        {'name': 'obs_1', 'x': 10.0, 'y': 9.0},
        {'name': 'obs_2', 'x': 12.0, 'y': 3.0},
        {'name': 'obs_3', 'x': 14.0, 'y': 11.0},
        {'name': 'obs_4', 'x': 16.0, 'y': 7.0},
    ]

    sim_time_param = {'use_sim_time': True}

    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Ego robot spawn at start position (1.0, 7.5)
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'ego_robot', '-file', urdf, '-x', '1.0', '-y', '7.5', '-z', '0.2'],
             parameters=[sim_time_param],
             output='screen'),
    ])

    # Spawn multiple dynamic obstacles
    for obs in obstacles:
        sdf = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
        with open(sdf, 'r') as f:
            sdf_content = f.read().replace('/obs_{id}', f'/{obs["name"]}')
        tmp_sdf = f'/tmp/{obs["name"]}.sdf'
        with open(tmp_sdf, 'w') as f:
            f.write(sdf_content)
        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', obs['name'], '-file', tmp_sdf,
                       '-x', str(obs['x']), '-y', str(obs['y']), '-z', '0.1'],
            parameters=[sim_time_param],
            output='screen'))

    ld.add_action(Node(package='occlusion_sim', executable='multi_obstacle_controller.py',
                       parameters=[sim_time_param], output='screen'))
    ld.add_action(Node(package='occlusion_sim', executable='cbf_wrapper_node.py',
                       parameters=[sim_time_param], output='screen'))
    ld.add_action(Node(package='occlusion_sim', executable='sensor_visualizer_node.py',
                       parameters=[sim_time_param], output='screen'))
    ld.add_action(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
                       parameters=[sim_time_param], output='screen'))
    return ld
