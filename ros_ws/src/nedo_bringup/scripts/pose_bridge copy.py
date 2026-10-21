#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D

class PoseBridgeNode(Node):
    def __init__(self):
        super().__init__('pose_bridge_node')
        
        # サブスクライバーとパブリッシャーの作成
        # トピック名は環境に合わせて変更してください
        self.subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose', # 受信するトピック名
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            Pose2D, 
            '/robot_pose2d',       # 送信するトピック名
            10)
            
        self.get_logger().info("PoseBridgeNodeを開始しました。")

    def listener_callback(self, msg):
        pose2d_msg = Pose2D()
        
        # 1. X, Y 座標の抽出
        pose2d_msg.x = msg.pose.position.x
        pose2d_msg.y = msg.pose.position.y
        
        # 2. クォータニオンからヨー角 (Theta) の計算
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        pose2d_msg.theta = yaw
        
        # 3. データのPublish
        self.publisher_.publish(pose2d_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
