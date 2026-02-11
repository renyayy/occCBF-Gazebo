"""Experiment launch: scenario-based simulation with DI, Unicycle, or Unicycle-TB3 mode.

Usage:
  ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout
  ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=multi_random mode:=unicycle
  ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout mode:=unicycle-tb3
  ros2 launch occlusion_sim gazebo_sim.launch.py scenario:=corner_popout record_bag:=false
"""
import os
import re
import sys
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

_lib_dir = os.path.join(get_package_prefix('occlusion_sim'), 'lib', 'occlusion_sim')
if _lib_dir not in sys.path:
    sys.path.insert(0, _lib_dir)
from scenarios import load_scenario
import sim_config


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
    auto_shutdown = _get_argv('auto_shutdown', 'false')
    sim_timeout = _get_argv('sim_timeout', '30.0')
    gui = _get_argv('gui', 'true')

    is_di = (mode == 'di')
    is_unicycle = (mode == 'unicycle')
    is_tb3 = (mode == 'unicycle-tb3')
    uses_converter = is_unicycle or is_tb3

    sc = load_scenario(scenario_name)
    robot_cfg = sc['robot']

    # モード別ロボットパラメータ
    if is_tb3:
        robot_v_max = sim_config.TB3_V_MAX
        robot_radius = sim_config.TB3_ROBOT_RADIUS
        robot_a_max = sim_config.TB3_A_MAX
        body_frame_odom = True
    else:
        robot_v_max = robot_cfg['v_max']
        robot_radius = robot_cfg['radius']
        robot_a_max = robot_cfg['a_max']
        body_frame_odom = False

    # bag_subdir (result_file パス構築にも使用)
    if is_tb3:
        bag_subdir = 'gazebo_unicycle_tb3'
    elif is_unicycle:
        bag_subdir = 'gazebo_unicycle'
    else:
        bag_subdir = 'gazebo_di'

    experiments_dir = _get_argv('bag_output_dir', '/root/Gazebo_ws/experiments')
    experiment_id_str = _get_argv('experiment_id', timestamp)
    result_file = os.path.join(experiments_dir, bag_subdir, experiment_id_str, 'result.json')

    # URDF / SDF 選択
    urdf_holonomic = os.path.join(pkg, 'urdf', 'simple_holonomic_robot.urdf')
    urdf_unicycle = os.path.join(pkg, 'urdf', 'simple_unicycle_robot.urdf')
    tb3_urdf, tb3_sdf = ('', '') if not is_tb3 else _resolve_tb3()

    world = os.path.join(pkg, 'worlds', sc['gazebo']['world_file'])
    rviz_config = os.path.join(pkg, 'rviz',
        'sensor_viz_unicycle.rviz' if is_tb3 else 'sensor_viz.rviz')
    sim_time_param = {'use_sim_time': True}

    record_bag = LaunchConfiguration('record_bag')
    experiment_id = LaunchConfiguration('experiment_id')
    bag_output_dir = LaunchConfiguration('bag_output_dir')

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
        DeclareLaunchArgument('auto_shutdown', default_value='false'),
        DeclareLaunchArgument('sim_timeout', default_value='30.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        gzserver,
    ])

    # --- 障害物スポーン (SDF半径をシナリオ設定に合わせる) ---
    sdf_template = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
    for obs_cfg in sc.get('obstacles', []):
        name = obs_cfg['name']
        obs_radius = obs_cfg['radius']
        with open(sdf_template, 'r') as f:
            sdf_content = f.read()
        sdf_content = sdf_content.replace('/obs_{id}', f'/{name}')
        sdf_content = re.sub(r'<radius>[^<]+</radius>',
                             f'<radius>{obs_radius}</radius>', sdf_content)
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

    # --- CBF共通パラメータ ---
    cbf_params = {
        'goal_x': robot_cfg['goal'][0],
        'goal_y': robot_cfg['goal'][1],
        'v_max': robot_v_max,
        'a_max': robot_a_max,
        'robot_radius': robot_radius,
        'scenario_name': scenario_name,
        'body_frame_odom': body_frame_odom,
        'auto_shutdown': auto_shutdown == 'true',
        'sim_timeout': float(sim_timeout),
        'result_file': result_file,
    }

    # --- センサビジュアライザ共通パラメータ ---
    env_cfg = sc['env']
    if is_tb3:
        robot_model_str = 'tb3'
    elif is_unicycle:
        robot_model_str = 'unicycle'
    else:
        robot_model_str = 'holonomic'

    viz_params = {
        'start_x': robot_cfg['start'][0],
        'start_y': robot_cfg['start'][1],
        'goal_x': robot_cfg['goal'][0],
        'goal_y': robot_cfg['goal'][1],
        'env_x_min': env_cfg['x_min'],
        'env_x_max': env_cfg['x_max'],
        'env_y_min': env_cfg['y_min'],
        'env_y_max': env_cfg['y_max'],
        'robot_model': robot_model_str,
        'robot_radius': robot_radius,
        'scenario_name': scenario_name,
    }

    # --- モード別エゴロボット構成 ---
    if is_di:
        # DI mode: holonomic robot
        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'ego_robot', '-file', urdf_holonomic,
                       '-x', str(robot_cfg['start'][0]),
                       '-y', str(robot_cfg['start'][1]),
                       '-z', '0.2'],
            parameters=[sim_time_param], output='screen'))

        cbf_node = Node(
            package='occlusion_sim', executable='cbf_wrapper_node.py',
            parameters=[sim_time_param, cbf_params],
            output='screen',
            **(dict(on_exit=[Shutdown()]) if auto_shutdown == 'true' else {}))
        ld.add_action(cbf_node)

    elif is_unicycle:
        # Unicycle mode: orange cylinder + planar_move + converter
        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'ego_robot', '-file', urdf_unicycle,
                       '-x', str(robot_cfg['start'][0]),
                       '-y', str(robot_cfg['start'][1]),
                       '-z', '0.2'],
            parameters=[sim_time_param], output='screen'))

        cbf_node = Node(
            package='occlusion_sim', executable='cbf_wrapper_node.py',
            remappings=[('/cmd_vel', '/di_cmd_vel')],
            parameters=[sim_time_param, cbf_params],
            output='screen',
            **(dict(on_exit=[Shutdown()]) if auto_shutdown == 'true' else {}))
        ld.add_action(cbf_node)

        ld.add_action(Node(
            package='occlusion_sim', executable='cmd_vel_converter.py',
            parameters=[sim_time_param, {'max_v': robot_v_max}],
            output='screen'))

    elif is_tb3:
        # Unicycle-TB3 mode: TurtleBot3 Burger
        ld.add_action(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{**sim_time_param, 'robot_description': tb3_urdf}],
            output='screen'))

        ld.add_action(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'ego_robot', '-file', tb3_sdf,
                       '-x', str(robot_cfg['start'][0]),
                       '-y', str(robot_cfg['start'][1]),
                       '-z', '0.01'],
            parameters=[sim_time_param], output='screen'))

        cbf_node = Node(
            package='occlusion_sim', executable='cbf_wrapper_node.py',
            remappings=[('/cmd_vel', '/di_cmd_vel')],
            parameters=[sim_time_param, cbf_params],
            output='screen',
            **(dict(on_exit=[Shutdown()]) if auto_shutdown == 'true' else {}))
        ld.add_action(cbf_node)

        ld.add_action(Node(
            package='occlusion_sim', executable='cmd_vel_converter.py',
            parameters=[sim_time_param, {
                'max_v': sim_config.TB3_V_MAX,
                'max_w': sim_config.TB3_MAX_OMEGA,
            }],
            output='screen'))

    # --- センサビジュアライザ ---
    ld.add_action(Node(
        package='occlusion_sim', executable='sensor_visualizer_node.py',
        parameters=[sim_time_param, viz_params],
        output='screen'))

    # --- RViz2 ---
    ld.add_action(Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param], output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))))

    # --- rosbag録画 (record_bag:=true の場合のみ) ---
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
