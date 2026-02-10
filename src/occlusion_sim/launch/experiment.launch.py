"""Experiment launch: simulation + automatic rosbag recording."""
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('occlusion_sim')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    experiment_id_arg = DeclareLaunchArgument('experiment_id', default_value=timestamp)
    bag_dir_arg = DeclareLaunchArgument('bag_output_dir',
                                        default_value='/root/Gazebo_ws/experiments/gazebo_di')
    scenario_arg = DeclareLaunchArgument('scenario', default_value='multi')

    experiment_id = LaunchConfiguration('experiment_id')
    bag_output_dir = LaunchConfiguration('bag_output_dir')
    scenario = LaunchConfiguration('scenario')

    is_multi = PythonExpression(["'", scenario, "' == 'multi'"])

    multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'multi_obstacle_simulation.launch.py')),
        condition=IfCondition(is_multi),
    )
    single_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'single_obstacle_simulation.launch.py')),
        condition=UnlessCondition(is_multi),
    )

    bag_record = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', PathJoinSubstitution([bag_output_dir, experiment_id]),
                 '/cmd_vel', '/odom', '/obstacle/state', '/cbf_debug_info', '/tf'],
            output='screen',
        )],
    )

    return LaunchDescription([
        experiment_id_arg, bag_dir_arg, scenario_arg,
        multi_launch, single_launch, bag_record,
    ])
