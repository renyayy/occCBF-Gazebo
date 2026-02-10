"""Experiment launch: scenario-based simulation with DI or Unicycle mode.

Usage:
  ros2 launch occlusion_sim experiment.launch.py scenario:=corner_popout
  ros2 launch occlusion_sim experiment.launch.py scenario:=multi_random mode:=unicycle
  ros2 launch occlusion_sim experiment.launch.py scenario:=corner_popout record_bag:=false
"""
import os
import sys
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

_lib_dir = os.path.join(get_package_prefix('occlusion_sim'), 'lib', 'occlusion_sim')
if _lib_dir not in sys.path:
    sys.path.insert(0, _lib_dir)
from scenarios import load_scenario


def _get_argv(key, default):
    """sys.argvからlaunch引数を取得 (launch生成時に評価)"""
    for arg in sys.argv:
        if arg.startswith(f'{key}:='):
            return arg.split(':=', 1)[1]
    return default


def _resolve_tb3():
    """TurtleBot3 の SDF/URDF パスを解決"""
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    sdf = os.path.join(tb3_pkg, 'models', 'turtlebot3_burger', 'model.sdf')
    urdf = os.path.join(tb3_pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf, 'r') as f:
        urdf_xml = f.read()
    return urdf_xml, sdf


def generate_launch_description():
    pkg = get_package_share_directory('occlusion_sim')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    scenario_name = _get_argv('scenario', 'multi_random')
    mode = _get_argv('mode', 'di')
    use_unicycle = (mode == 'unicycle')

    sc = load_scenario(scenario_name)
    robot_cfg = sc['robot']

    tb3_urdf, tb3_sdf = ('', '') if not use_unicycle else _resolve_tb3()

    world = os.path.join(pkg, 'worlds', sc['gazebo']['world_file'])
    urdf_holonomic = os.path.join(pkg, 'urdf', 'simple_holonomic_robot.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'sensor_viz.rviz')
    sim_time_param = {'use_sim_time': True}

    mode_lc = LaunchConfiguration('mode')
    record_bag = LaunchConfiguration('record_bag')
    experiment_id = LaunchConfiguration('experiment_id')
    bag_output_dir = LaunchConfiguration('bag_output_dir')
    is_unicycle = PythonExpression(["'", mode_lc, "' == 'unicycle'"])

    # Gazebo server (公式launch経由で GAZEBO_MODEL_PATH を自動設定)
    gazebo_ros_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch, 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )

    ld = LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        DeclareLaunchArgument('mode', default_value='di'),
        DeclareLaunchArgument('scenario', default_value='multi_random'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('experiment_id', default_value=timestamp),
        DeclareLaunchArgument('bag_output_dir',
                              default_value='/root/Gazebo_ws/experiments'),
        gzserver,
    ])

    # --- 障害物スポーン ---
    sdf_template = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
    for obs_cfg in sc.get('obstacles', []):
        name = obs_cfg['name']
        with open(sdf_template, 'r') as f:
            sdf_content = f.read().replace('/obs_{id}', f'/{name}')
        tmp_sdf = f'/tmp/{name}.sdf'
        with open(tmp_sdf, 'w') as f:
            f.write(sdf_content)
        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', name, '-file', tmp_sdf,
                       '-x', str(obs_cfg['position'][0]),
                       '-y', str(obs_cfg['position'][1]),
                       '-z', '0.1'],
            parameters=[sim_time_param], output='screen'))

    # --- 障害物コントローラ ---
    ld.add_action(Node(
        package='occlusion_sim', executable='scenario_obstacle_controller.py',
        parameters=[sim_time_param, {'scenario_name': scenario_name}],
        output='screen'))

    # --- DI mode: holonomic robot ---
    ld.add_action(Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'ego_robot', '-file', urdf_holonomic,
                   '-x', str(robot_cfg['start'][0]),
                   '-y', str(robot_cfg['start'][1]),
                   '-z', '0.2'],
        parameters=[sim_time_param], output='screen',
        condition=UnlessCondition(is_unicycle)))

    ld.add_action(Node(
        package='occlusion_sim', executable='cbf_wrapper_node.py',
        parameters=[sim_time_param, {
            'goal_x': robot_cfg['goal'][0],
            'goal_y': robot_cfg['goal'][1],
            'v_max': robot_cfg['v_max'],
            'a_max': robot_cfg['a_max'],
        }],
        output='screen',
        condition=UnlessCondition(is_unicycle)))

    # --- Unicycle mode: TurtleBot3 ---
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{**sim_time_param, 'robot_description': tb3_urdf}],
        output='screen',
        condition=IfCondition(is_unicycle)))

    ld.add_action(Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'ego_robot', '-file', tb3_sdf,
                   '-x', str(robot_cfg['start'][0]),
                   '-y', str(robot_cfg['start'][1]),
                   '-z', '0.01'],
        parameters=[sim_time_param], output='screen',
        condition=IfCondition(is_unicycle)))

    ld.add_action(Node(
        package='occlusion_sim', executable='cbf_wrapper_node.py',
        remappings=[('/cmd_vel', '/di_cmd_vel')],
        parameters=[sim_time_param, {
            'goal_x': robot_cfg['goal'][0],
            'goal_y': robot_cfg['goal'][1],
            'v_max': 0.22,
            'a_max': robot_cfg['a_max'],
            'body_frame_odom': True,
        }],
        output='screen',
        condition=IfCondition(is_unicycle)))

    ld.add_action(Node(
        package='occlusion_sim', executable='cmd_vel_converter.py',
        parameters=[sim_time_param], output='screen',
        condition=IfCondition(is_unicycle)))

    # --- センサビジュアライザ ---
    ld.add_action(Node(
        package='occlusion_sim', executable='sensor_visualizer_node.py',
        parameters=[sim_time_param, {
            'start_x': robot_cfg['start'][0],
            'start_y': robot_cfg['start'][1],
            'goal_x': robot_cfg['goal'][0],
            'goal_y': robot_cfg['goal'][1],
        }],
        output='screen'))

    # --- RViz2 ---
    ld.add_action(Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param], output='screen'))

    # --- rosbag録画 (record_bag:=true の場合のみ) ---
    bag_subdir = PythonExpression([
        "'gazebo_unicycle' if '", mode_lc, "' == 'unicycle' else 'gazebo_di'"])
    ld.add_action(TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', PathJoinSubstitution([bag_output_dir, bag_subdir, experiment_id]),
                 '/cmd_vel', '/di_cmd_vel', '/odom', '/obstacle/state',
                 '/cbf_debug_info', '/tf'],
            output='screen',
            condition=IfCondition(record_bag),
        )],
    ))

    return ld
