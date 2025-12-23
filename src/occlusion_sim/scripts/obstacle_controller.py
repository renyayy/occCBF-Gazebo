#!/usr/bin/env python3
"""
障害物コントローラー (Obstacle Controller)

指定した2点間(y_min, y_max)を往復運動する障害物を制御します。
/obstacle/odom から位置を取得し、/obstacle/cmd_vel で速度指令を送ります。
/obstacle/state で障害物情報を Publish（CBFノード用）。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        
        # パラメータ設定
        self.declare_parameter('y_min', -3.0)
        self.declare_parameter('y_max', 3.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('radius', 0.3)
        
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.speed = self.get_parameter('speed').value
        self.radius = self.get_parameter('radius').value
        
        # 内部状態
        self.current_state = None  # Odometry メッセージ全体を保持
        self.direction = 1  # 1: +y方向, -1: -y方向
        
        # ROS 2 通信
        self.odom_sub = self.create_subscription(
            Odometry, '/obstacle/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/obstacle/cmd_vel', 10)
        self.state_pub = self.create_publisher(
            Odometry, '/obstacle/state', 10)
        
        # 制御ループ (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(
            f'Obstacle Controller started\n'
            f'  Y range: [{self.y_min}, {self.y_max}]\n'
            f'  Speed: {self.speed} m/s, Radius: {self.radius} m'
        )
    
    def odom_callback(self, msg):
        """オドメトリ受信"""
        self.current_state = msg
    
    def control_loop(self):
        """制御ループ: 反射判定 → 速度指令 → 状態Publish"""
        if self.current_state is None:
            return  # オドメトリ未受信
        
        # 現在のy座標
        y = self.current_state.pose.pose.position.y
        
        # 反射境界での方向反転
        if y >= self.y_max and self.direction > 0:
            self.direction = -1
        elif y <= self.y_min and self.direction < 0:
            self.direction = 1
        
        # 速度指令
        cmd = Twist()
        cmd.linear.y = self.speed * self.direction
        self.cmd_vel_pub.publish(cmd)
        
        # 状態Publish (CBFノード用)
        state = Odometry()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'world'
        state.child_frame_id = 'moving_cylinder'
        state.pose.pose = self.current_state.pose.pose
        state.twist.twist.linear.y = self.speed * self.direction
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
