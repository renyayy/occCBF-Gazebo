#!/usr/bin/env python3
"""Sensor Visualizer Node for RViz2"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import math


class SensorVisualizerNode(Node):
    def __init__(self):
        super().__init__('sensor_visualizer_node')

        # Parameters (safe_control/dynamic_env準拠)
        self.fov_angle = np.deg2rad(70.0)
        self.cam_range = 10.0  
        self.sensing_range = 10.0
        self.robot_radius = 0.25
        self.obstacle_radius = 0.3

        # Start/Goal positions
        self.start_pos = (1.0, 7.5)
        self.goal_pos = (20.0, 7.5)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.obstacle_states = {}

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/obstacle/state', self.obs_cb, 10)

        # Movement boundaries (from multi_obstacle_controller.py)
        self.X_MIN, self.X_MAX = 0.0, 24.0
        self.Y_MIN, self.Y_MAX = 1.0, 14.0

        # Publishers
        self.fov_pub = self.create_publisher(Marker, '/sensor_viz/fov', 10)
        self.fov_outline_pub = self.create_publisher(Marker, '/sensor_viz/fov_outline', 10)
        self.sensing_range_pub = self.create_publisher(Marker, '/sensor_viz/sensing_range', 10)
        self.occlusion_pub = self.create_publisher(MarkerArray, '/sensor_viz/occlusion', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/sensor_viz/obstacles', 10)
        self.start_pub = self.create_publisher(Marker, '/sensor_viz/start', 10)
        self.goal_pub = self.create_publisher(Marker, '/sensor_viz/goal', 10)
        self.boundary_pub = self.create_publisher(Marker, '/sensor_viz/boundary', 10)
        self.ground_pub = self.create_publisher(Marker, '/sensor_viz/ground', 10)
        self.ego_robot_pub = self.create_publisher(Marker, '/sensor_viz/ego_robot', 10)

        # Timer (20Hz)
        self.create_timer(0.05, self.publish_markers)

        self.get_logger().info('Sensor Visualizer Node started')

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def obs_cb(self, msg):
        name = msg.child_frame_id or 'obs_0'
        self.obstacle_states[name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'radius': self.obstacle_radius
        }

    def publish_markers(self):
        if not self.odom_received:
            return
        self.publish_fov_marker()
        self.publish_fov_outline()
        self.publish_sensing_range_marker()
        self.publish_occlusion_markers()
        self.publish_obstacle_markers()
        self.publish_start_goal_markers()
        self.publish_boundary_marker()
        self.publish_ground_marker()
        self.publish_ego_robot_marker()

    def publish_fov_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fov_fill'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.25)

        resolution = 20
        angle_left = self.robot_yaw + self.fov_angle / 2
        angle_right = self.robot_yaw - self.fov_angle / 2
        angles = np.linspace(angle_right, angle_left, resolution + 1)

        robot_pos = Point(x=self.robot_x, y=self.robot_y, z=0.05)

        for i in range(resolution):
            p1 = Point(
                x=self.robot_x + self.cam_range * np.cos(angles[i]),
                y=self.robot_y + self.cam_range * np.sin(angles[i]),
                z=0.05
            )
            p2 = Point(
                x=self.robot_x + self.cam_range * np.cos(angles[i + 1]),
                y=self.robot_y + self.cam_range * np.sin(angles[i + 1]),
                z=0.05
            )
            marker.points.extend([robot_pos, p1, p2])

        self.fov_pub.publish(marker)

    def publish_fov_outline(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fov_outline'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)

        resolution = 20
        angle_left = self.robot_yaw + self.fov_angle / 2
        angle_right = self.robot_yaw - self.fov_angle / 2
        angles = np.linspace(angle_right, angle_left, resolution + 1)

        marker.points.append(Point(x=self.robot_x, y=self.robot_y, z=0.05))
        for angle in angles:
            marker.points.append(Point(
                x=self.robot_x + self.cam_range * np.cos(angle),
                y=self.robot_y + self.cam_range * np.sin(angle),
                z=0.05
            ))
        marker.points.append(Point(x=self.robot_x, y=self.robot_y, z=0.05))

        self.fov_outline_pub.publish(marker)

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

    def circle_tangents(self, p, c, R):
        """Compute tangent points from point p to circle (center c, radius R)."""
        p = np.asarray(p, dtype=float)
        c = np.asarray(c, dtype=float)
        v = p - c
        d2 = float(v @ v)
        R2 = R * R

        if d2 <= R2:
            return None, None

        x1, y1 = v
        x0 = R2 * x1 / d2
        y0 = R2 * y1 / d2
        k = R * np.sqrt(d2 - R2) / d2

        t1 = np.array([x0 - y1 * k, y0 + x1 * k]) + c
        t2 = np.array([x0 + y1 * k, y0 - x1 * k]) + c
        return t1, t2

    def publish_occlusion_markers(self):
        marker_array = MarkerArray()
        robot_pos = np.array([self.robot_x, self.robot_y])

        for idx, (name, obs) in enumerate(self.obstacle_states.items()):
            obs_center = np.array([obs['x'], obs['y']])
            obs_radius = obs['radius']

            # Check if obstacle is within sensing range
            dist = np.linalg.norm(obs_center - robot_pos)
            if dist > self.sensing_range + obs_radius:
                continue

            t1, t2 = self.circle_tangents(robot_pos, obs_center, obs_radius)
            if t1 is None:
                continue

            # Extend tangent directions to sensing range
            dir1 = (t1 - robot_pos)
            dir1 = dir1 / np.linalg.norm(dir1)
            dir2 = (t2 - robot_pos)
            dir2 = dir2 / np.linalg.norm(dir2)

            far1 = robot_pos + self.sensing_range * dir1
            far2 = robot_pos + self.sensing_range * dir2

            # Create wedge polygon marker (two triangles)
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
            # Triangle 1: t1, t2, far2
            marker.points.append(Point(x=float(t1[0]), y=float(t1[1]), z=z))
            marker.points.append(Point(x=float(t2[0]), y=float(t2[1]), z=z))
            marker.points.append(Point(x=float(far2[0]), y=float(far2[1]), z=z))
            # Triangle 2: t1, far2, far1
            marker.points.append(Point(x=float(t1[0]), y=float(t1[1]), z=z))
            marker.points.append(Point(x=float(far2[0]), y=float(far2[1]), z=z))
            marker.points.append(Point(x=float(far1[0]), y=float(far1[1]), z=z))

            marker_array.markers.append(marker)

        # Delete old markers if obstacles reduced
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
        robot_pos = np.array([self.robot_x, self.robot_y])

        for idx, (name, obs) in enumerate(self.obstacle_states.items()):
            obs_center = np.array([obs['x'], obs['y']])
            obs_radius = obs['radius']

            dist = np.linalg.norm(obs_center - robot_pos)
            detected = dist <= self.sensing_range

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
        marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.9)

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
