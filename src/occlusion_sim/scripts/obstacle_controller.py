#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        # /obstacle 名前空間下の cmd_vel に配信
        self.publisher_ = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Obstacle Controller Started')

    def timer_callback(self):
        msg = Twist()
        # GIFのような斜め移動 (XとYに速度を与える)
        # 座標系に合わせて調整してください。
        # ここでは単純に X軸マイナス方向（画面左）へ進む例とします
        msg.linear.x = -0.5 
        msg.linear.y = 0.2  # 少しY方向にも動かす
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止時に安全のため速度0を送る
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
