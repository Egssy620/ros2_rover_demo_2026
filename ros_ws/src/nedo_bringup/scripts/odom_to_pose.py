#!/usr/bin/env python3
# odom_to_pose.py
#
# robot_localization (EKF) の出力 /odometry/filtered (nav_msgs/Odometry) を
# geometry_msgs/PoseStamped に変換して /robot_pose へ publish する。
#
# nedo_controller など PoseStamped /robot_pose を購読する下流に、EKF の連続/絶対
# pose を渡すためのアダプタ。Odometry の pose は msg.pose.pose にネストしている。
# header (stamp, frame_id) はそのまま引き継ぐ。
#
# パラメータ
#   input_topic  : 購読する Odometry トピック (default /odometry/filtered)
#   output_topic : 出力する PoseStamped トピック (default /robot_pose)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__('odom_to_pose_node')

        self.input_topic = self.declare_parameter('input_topic', '/odometry/filtered').value
        self.output_topic = self.declare_parameter('output_topic', '/robot_pose').value

        self.publisher_ = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.listener_callback, 10)

        self.get_logger().info(
            f'odom_to_pose: {self.input_topic} (Odometry) -> {self.output_topic} (PoseStamped)')

    def listener_callback(self, msg):
        pose_msg = PoseStamped()
        # header (stamp, frame_id) を引き継ぐ。Odometry は pose が pose.pose にネスト。
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseNode()
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
