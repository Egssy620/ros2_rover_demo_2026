#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

class PoseBridgeNode(Node):
    def __init__(self):
        super().__init__('pose_bridge_node')

        # /robot_pose は EKF (robot_localization) 出力を remap したものなので
        # 型は nav_msgs/Odometry。pose は msg.pose.pose (PoseWithCovariance) に入る。
        # トピック名は環境に合わせて変更してください
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered', # 受信するトピック名 (nav_msgs/Odometry)
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            Pose2D,
            '/robot_pose2d',       # 送信するトピック名
            10)

        self.get_logger().info("PoseBridgeNodeを開始しました。(Odometry -> Pose2D)")

    def listener_callback(self, msg):
        pose2d_msg = Pose2D()

        # Odometry は pose が msg.pose.pose (PoseWithCovariance) にネストしている
        pose = msg.pose.pose

        # 1. X, Y 座標の抽出
        pose2d_msg.x = pose.position.x
        pose2d_msg.y = pose.position.y

        # 2. クォータニオンからヨー角 (Theta) の計算
        q = pose.orientation
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
