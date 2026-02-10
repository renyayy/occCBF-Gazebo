"""Experiment comparison launch: DI vs Unicycle on corner pop-out scenario."""
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_tb3():
    """TurtleBot3 の SDF (Gazebo用) と URDF (TF用) を解決"""
    tb3_gz_pkg = get_package_share_directory('turtlebot3_gazebo')
    # SDF: Gazebo spawn 用 (model:// URI でメッシュ解決、diff_drive プラグイン内蔵)
    sdf_path = os.path.join(
        tb3_gz_pkg, 'models', 'turtlebot3_burger', 'model.sdf')
    # URDF: robot_state_publisher 用 (TF ツリーのみ、Gazebo プラグイン不要)
    urdf_path = os.path.join(tb3_gz_pkg, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()
    return urdf_xml, sdf_path


def generate_launch_description():
    pkg = get_package_share_directory('occlusion_sim')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # 起動時引数をパースして mode を取得 (TB3 依存の遅延読み込み用)
    import sys
    use_unicycle = any('mode:=unicycle' in a for a in sys.argv)

    tb3_urdf = ''
    tb3_sdf_path = ''
    if use_unicycle:
        tb3_urdf, tb3_sdf_path = _resolve_tb3()

    # Arguments
    mode_arg = DeclareLaunchArgument('mode', default_value='di',
                                     description='di or unicycle')
    record_arg = DeclareLaunchArgument('record_bag', default_value='true')
    experiment_id_arg = DeclareLaunchArgument('experiment_id', default_value=timestamp)
    bag_dir_arg = DeclareLaunchArgument('bag_output_dir',
                                        default_value=os.path.join(
                                            os.path.dirname(pkg), '..', '..',
                                            'experiments'))

    mode = LaunchConfiguration('mode')
    record_bag = LaunchConfiguration('record_bag')
    experiment_id = LaunchConfiguration('experiment_id')
    bag_output_dir = LaunchConfiguration('bag_output_dir')
    is_unicycle = PythonExpression(["'", mode, "' == 'unicycle'"])

    # Paths
    world = os.path.join(pkg, 'worlds', 'experiment_corner.world')
    urdf_holonomic = os.path.join(pkg, 'urdf', 'simple_holonomic_robot.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'sensor_viz.rviz')

    # Obstacle SDF
    sdf = os.path.join(pkg, 'models', 'dynamic_obstacle.sdf')
    with open(sdf, 'r') as f:
        sdf_content = f.read().replace('/obs_{id}', '/obs_0')
    tmp_sdf = '/tmp/obs_0_corner.sdf'
    with open(tmp_sdf, 'w') as f:
        f.write(sdf_content)

    sim_time_param = {'use_sim_time': True}

    # Corner scenario params for sensor_visualizer
    corner_viz_params = {
        'use_sim_time': True,
        'start_x': 0.0, 'start_y': 1.0,
        'goal_x': 5.0, 'goal_y': 1.0,
    }

    # --- Common nodes ---
    # 公式 launch 経由で起動 (GAZEBO_MODEL_PATH 等を自動設定)
    gazebo_ros_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch, 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch, 'gzclient.launch.py')),
    )

    # Obstacle spawn at (3.0, 2.0) — behind wall
    spawn_obstacle = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'obs_0', '-file', tmp_sdf,
                   '-x', '3.0', '-y', '2.0', '-z', '0.1'],
        parameters=[sim_time_param], output='screen')

    obstacle_ctrl = Node(
        package='occlusion_sim', executable='scenario_obstacle_controller.py',
        parameters=[sim_time_param], output='screen')

    sensor_viz = Node(
        package='occlusion_sim', executable='sensor_visualizer_node.py',
        parameters=[corner_viz_params], output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param], output='screen')

    # --- DI mode ---
    spawn_di_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'ego_robot', '-file', urdf_holonomic,
                   '-x', '0.0', '-y', '1.0', '-z', '0.2'],
        parameters=[sim_time_param], output='screen',
        condition=UnlessCondition(is_unicycle))

    cbf_di = Node(
        package='occlusion_sim', executable='cbf_wrapper_node.py',
        parameters=[{**sim_time_param, 'goal_x': 5.0, 'goal_y': 1.0}],
        output='screen',
        condition=UnlessCondition(is_unicycle))

    # --- Unicycle mode ---
    tb3_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{**sim_time_param, 'robot_description': tb3_urdf}],
        output='screen',
        condition=IfCondition(is_unicycle))

    spawn_tb3_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'ego_robot', '-file', tb3_sdf_path,
                   '-x', '0.0', '-y', '1.0', '-z', '0.01'],
        parameters=[sim_time_param], output='screen',
        condition=IfCondition(is_unicycle))

    cbf_unicycle = Node(
        package='occlusion_sim', executable='cbf_wrapper_node.py',
        remappings=[('/cmd_vel', '/di_cmd_vel')],
        parameters=[{**sim_time_param, 'v_max': 0.22, 'goal_x': 5.0, 'goal_y': 1.0,
                      'body_frame_odom': True}],
        output='screen',
        condition=IfCondition(is_unicycle))

    converter = Node(
        package='occlusion_sim', executable='cmd_vel_converter.py',
        parameters=[sim_time_param], output='screen',
        condition=IfCondition(is_unicycle))

    # --- Rosbag recording ---
    bag_subdir = PythonExpression([
        "'gazebo_unicycle' if '", mode, "' == 'unicycle' else 'gazebo_di'"])
    bag_record = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', PathJoinSubstitution([bag_output_dir, bag_subdir, experiment_id]),
                 '/cmd_vel', '/di_cmd_vel', '/odom', '/obstacle/state',
                 '/cbf_debug_info', '/tf'],
            output='screen',
            condition=IfCondition(record_bag),
        )],
    )

    return LaunchDescription([
        # Environment
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        # Arguments
        mode_arg, record_arg, experiment_id_arg, bag_dir_arg,
        # Common
        gzserver, gzclient, spawn_obstacle, obstacle_ctrl, sensor_viz, rviz,
        # DI mode
        spawn_di_robot, cbf_di,
        # Unicycle mode
        tb3_state_publisher, spawn_tb3_robot, cbf_unicycle, converter,
        # Recording
        bag_record,
    ])
