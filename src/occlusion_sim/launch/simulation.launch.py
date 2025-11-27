import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. パスの設定
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_occlusion_sim = get_package_share_directory('occlusion_sim')

    # Worldファイルのパス
    world_file_path = os.path.join(pkg_occlusion_sim, 'worlds', 'occlusion.world')
    
    # 動的障害物(SDF)のパス
    obstacle_sdf_path = os.path.join(pkg_occlusion_sim, 'models', 'moving_cylinder.sdf')

    # TurtleBot3 のモデル指定 (環境変数またはデフォルト)
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    # TurtleBot3 の SDFパス (標準パッケージから取得)
    tb3_sdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + turtlebot3_model, 'model.sdf')

    # 2. Gazeboサーバーとクライアントの起動設定
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 3. 動的障害物のスポーン (名前空間: obstacle)
    # X=-1.0, Y=-2.0 あたりに出現させる
    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'moving_obstacle',
            '-file', obstacle_sdf_path,
            '-x', '-2.0',
            '-y', '-2.0',
            '-z', '0.25',
            '-robot_namespace', 'obstacle' # ここで名前空間を分離
        ],
        output='screen'
    )

    # 4. 自機(TurtleBot3)のスポーン (名前空間: なし = グローバル)
    # X=0.0, Y=0.0 (中心) に出現させる
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_ego',
            '-file', tb3_sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '-1.5708' # 向き(Yaw)
        ],
        output='screen'
    )

    # 5. LaunchDescriptionの生成
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_obstacle)
    ld.add_action(spawn_tb3)

    return ld
