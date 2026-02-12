#!/usr/bin/env python3
"""Sensor Visualizer Node for RViz2"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import sim_config  # safe_control パスも設定される

from robots.double_integrator2D import DoubleIntegrator2D
from utils.occlusion import OcclusionUtils
from scenarios import load_scenario


class SensorVisualizerNode(Node):
    def __init__(self):
        super().__init__('sensor_visualizer_node')

        # ROS パラメータ
        self.declare_parameter('start_x', sim_config.DEFAULT_START[0])
        self.declare_parameter('start_y', sim_config.DEFAULT_START[1])
        self.declare_parameter('goal_x', sim_config.DEFAULT_GOAL[0])
        self.declare_parameter('goal_y', sim_config.DEFAULT_GOAL[1])
        self.declare_parameter('env_x_min', sim_config.ENV_X_MIN)
        self.declare_parameter('env_x_max', sim_config.ENV_X_MAX)
        self.declare_parameter('env_y_min', sim_config.ENV_Y_MIN)
        self.declare_parameter('env_y_max', sim_config.ENV_Y_MAX)
        self.declare_parameter('robot_model', 'holonomic')
        self.declare_parameter('robot_radius', sim_config.ROBOT_RADIUS)
        self.declare_parameter('sensing_range', sim_config.SENSING_RANGE)
        self.declare_parameter('scenario_name', 'corner_popout')

        robot_model = self.get_parameter('robot_model').value
        self.is_tb3 = (robot_model == 'tb3')
        self.is_unicycle_color = (robot_model == 'unicycle')

        self.sensing_range = self.get_parameter('sensing_range').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # 障害物名→半径マップ (シナリオ設定から構築)
        sc = load_scenario(self.get_parameter('scenario_name').value)
        self._obs_radius_map = {o['name']: o['radius'] for o in sc.get('obstacles', [])}
        self._obs_type_map = {
            o['name']: (0 if o.get('behavior') == 'static' else 1)
            for o in sc.get('obstacles', [])
        }

        self.robot_spec = sim_config.make_robot_spec(radius=self.robot_radius,
                                                     sensing_range=self.sensing_range)
        self.robot = DoubleIntegrator2D(sim_config.DT, self.robot_spec)

        self.occlusion_manager = OcclusionUtils(
            robot=self.robot,
            robot_spec=self.robot_spec,
            sensing_range=self.sensing_range,
            barrier_fn=None
        )

        # Start/Goal positions
        self.start_pos = (self.get_parameter('start_x').value,
                          self.get_parameter('start_y').value)
        self.goal_pos = (self.get_parameter('goal_x').value,
                         self.get_parameter('goal_y').value)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False
        self.obstacle_states = {}

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)

        # Movement boundaries
        self.X_MIN = self.get_parameter('env_x_min').value
        self.X_MAX = self.get_parameter('env_x_max').value
        self.Y_MIN = self.get_parameter('env_y_min').value
        self.Y_MAX = self.get_parameter('env_y_max').value

        # Publishers
        self.sensing_range_pub = self.create_publisher(Marker, '/sensor_viz/sensing_range', 10)
        self.occlusion_pub = self.create_publisher(MarkerArray, '/sensor_viz/occlusion', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/sensor_viz/obstacles', 10)
        self.start_pub = self.create_publisher(Marker, '/sensor_viz/start', 10)
        self.goal_pub = self.create_publisher(Marker, '/sensor_viz/goal', 10)
        self.boundary_pub = self.create_publisher(Marker, '/sensor_viz/boundary', 10)
        self.ground_pub = self.create_publisher(Marker, '/sensor_viz/ground', 10)
        self.ego_robot_pub = self.create_publisher(Marker, '/sensor_viz/ego_robot', 10)

        # Timer (20Hz)
        self.create_timer(sim_config.DT, self.publish_markers)

        self.get_logger().info('Sensor Visualizer Node started')

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True

    def obs_cb(self, msg):
        name = msg.child_frame_id or 'obs_0'
        obs_radius = self._obs_radius_map.get(name, sim_config.OBSTACLE_RADIUS)
        self.obstacle_states[name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'radius': obs_radius,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'obs_type': self._obs_type_map.get(name, 1),
        }

    def publish_markers(self):
        if not self.odom_received:
            return
        self.publish_sensing_range_marker()
        self.publish_occlusion_markers()
        self.publish_obstacle_markers()
        self.publish_start_goal_markers()
        self.publish_boundary_marker()
        self.publish_ground_marker()
        if not self.is_tb3:
            self.publish_ego_robot_marker()

    def publish_sensing_range_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sensing_range'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)

        resolution = 64
        angles = np.linspace(0, 2 * np.pi, resolution + 1)
        for angle in angles:
            marker.points.append(Point(
                x=self.robot_x + self.sensing_range * np.cos(angle),
                y=self.robot_y + self.sensing_range * np.sin(angle),
                z=0.05
            ))

        self.sensing_range_pub.publish(marker)

    def publish_occlusion_markers(self):
        marker_array = MarkerArray()

        # ロボット状態を準備（OcclusionUtils用）
        robot_state = np.array([[self.robot_x], [self.robot_y], [0.0], [0.0]])

        # 障害物リストを準備（OcclusionUtils用）
        obs_list = []
        for name, obs in self.obstacle_states.items():
            obs_list.append([obs['x'], obs['y'], obs['radius'],
                            obs.get('vx', 0.0), obs.get('vy', 0.0),
                            0.0, 0.0, float(obs.get('obs_type', 1))])

        if len(obs_list) == 0:
            delete_marker = Marker()
            delete_marker.header.frame_id = 'odom'
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = 'occlusion'
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.occlusion_pub.publish(marker_array)
            return

        obs_array = np.array(obs_list)

        # OcclusionUtilsでフィルタリング（センシング範囲 + オクルージョン考慮）
        _, occl_scenarios = self.occlusion_manager._filter_visible_and_build_occ(
            robot_state, obs_array
        )

        # オクルージョンシナリオのポリゴンを直接描画
        for idx, sc in enumerate(occl_scenarios):
            poly = sc['poly']  # [t1, t2, far2, far1] (4,2)

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'occlusion'
            marker.id = idx
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.35)

            z = 0.04
            t1, t2, far2, far1 = poly[0], poly[1], poly[2], poly[3]
            # Triangle 1: t1, t2, far2
            marker.points.append(Point(x=float(t1[0]), y=float(t1[1]), z=z))
            marker.points.append(Point(x=float(t2[0]), y=float(t2[1]), z=z))
            marker.points.append(Point(x=float(far2[0]), y=float(far2[1]), z=z))
            # Triangle 2: t1, far2, far1
            marker.points.append(Point(x=float(t1[0]), y=float(t1[1]), z=z))
            marker.points.append(Point(x=float(far2[0]), y=float(far2[1]), z=z))
            marker.points.append(Point(x=float(far1[0]), y=float(far1[1]), z=z))

            marker_array.markers.append(marker)

        if len(marker_array.markers) == 0:
            delete_marker = Marker()
            delete_marker.header.frame_id = 'odom'
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = 'occlusion'
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        self.occlusion_pub.publish(marker_array)

    def publish_obstacle_markers(self):
        marker_array = MarkerArray()

        for idx, (name, obs) in enumerate(self.obstacle_states.items()):
            obs_radius = obs['radius']

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = obs_radius * 2
            marker.scale.y = obs_radius * 2
            marker.scale.z = 0.2

            # Set color to gray to match Gazebo
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)

            marker_array.markers.append(marker)

        if len(marker_array.markers) == 0:
            delete_marker = Marker()
            delete_marker.header.frame_id = 'odom'
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = 'obstacles'
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        self.obstacles_pub.publish(marker_array)

    def publish_start_goal_markers(self):
        # Start marker (green sphere)
        start = Marker()
        start.header.frame_id = 'odom'
        start.header.stamp = self.get_clock().now().to_msg()
        start.ns = 'start'
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x = self.start_pos[0]
        start.pose.position.y = self.start_pos[1]
        start.pose.position.z = 0.15
        start.pose.orientation.w = 1.0
        start.scale.x = 0.3
        start.scale.y = 0.3
        start.scale.z = 0.3
        start.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        self.start_pub.publish(start)

        # Goal marker (red sphere)
        goal = Marker()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.ns = 'goal'
        goal.id = 0
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = self.goal_pos[0]
        goal.pose.position.y = self.goal_pos[1]
        goal.pose.position.z = 0.15
        goal.pose.orientation.w = 1.0
        goal.scale.x = 0.3
        goal.scale.y = 0.3
        goal.scale.z = 0.3
        goal.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        self.goal_pub.publish(goal)

    def publish_boundary_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'boundary'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        # Draw boundary rectangle
        z = 0.01
        marker.points.append(Point(x=self.X_MIN, y=self.Y_MIN, z=z))
        marker.points.append(Point(x=self.X_MAX, y=self.Y_MIN, z=z))
        marker.points.append(Point(x=self.X_MAX, y=self.Y_MAX, z=z))
        marker.points.append(Point(x=self.X_MIN, y=self.Y_MAX, z=z))
        marker.points.append(Point(x=self.X_MIN, y=self.Y_MIN, z=z))

        self.boundary_pub.publish(marker)

    def publish_ground_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ground'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Center of movement area
        center_x = (self.X_MIN + self.X_MAX) / 2.0
        center_y = (self.Y_MIN + self.Y_MAX) / 2.0
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size of movement area
        marker.scale.x = self.X_MAX - self.X_MIN
        marker.scale.y = self.Y_MAX - self.Y_MIN
        marker.scale.z = 0.001

        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        self.ground_pub.publish(marker)

    def publish_ego_robot_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ego_robot'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.robot_radius * 2
        marker.scale.y = self.robot_radius * 2
        marker.scale.z = 0.2
        if self.is_unicycle_color:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # orange
        else:
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.9)  # purple

        self.ego_robot_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
