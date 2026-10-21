#!/usr/bin/env python3
# 機構部 (マイコン) の簡易ノード: マスターからの依頼に、決まった時間のあと自動で答える (キー入力なし)
#   /mechanism/command "grasp"    → reply_delay 秒後に /mechanism/result "grasped" (capacity 回目ごとに "full")
#   /mechanism/command "unload N" → reply_delay 秒後に /mechanism/result "unloaded"
#   /mechanism/grasp_point は受け取ってログに出すだけ
# マイコンとつなぐときは work() の中身を「マイコンに頼んで返事を待つ」に置き換える
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class MechanismNode(Node):
    def __init__(self):
        super().__init__('mechanism_node')
        self.reply_delay = self.declare_parameter('reply_delay', 1.0).value  # 答えるまでの時間 [s]
        self.capacity = self.declare_parameter('capacity', 5).value         # 何回つかんだら箱が満杯か
        self.grasp_count = 0
        self.timer = None

        self.pub_result = self.create_publisher(String, '/mechanism/result', 10)
        self.create_subscription(String, '/mechanism/command', self.on_command, 10)
        self.create_subscription(PointStamped, '/mechanism/grasp_point', self.on_grasp_point, 10)
        self.get_logger().info('準備できました。マスターからの依頼を待ちます')

    def on_grasp_point(self, m):
        self.get_logger().info(f'つかむ点を受け取りました ({m.point.x:.2f}, {m.point.y:.2f}, {m.point.z:.2f})')

    def on_command(self, m):
        if self.timer is not None:
            self.get_logger().warn(f'前の依頼の途中なので「{m.data}」は無視します')
            return
        self.get_logger().info(f'依頼「{m.data}」を受け取りました。{self.reply_delay:.1f} 秒後に答えます')
        self.timer = self.create_timer(self.reply_delay, lambda: self.reply(m.data))

    def reply(self, cmd):
        self.timer.cancel()
        self.destroy_timer(self.timer)
        self.timer = None
        result = self.work(cmd)
        if result is None:
            self.get_logger().warn(f'知らない依頼「{cmd}」なので答えません')
            return
        self.pub_result.publish(String(data=result))
        self.get_logger().info(f'「{result}」と答えました')

    def work(self, cmd):
        if cmd == 'grasp':
            self.grasp_count += 1
            return 'full' if self.grasp_count % self.capacity == 0 else 'grasped'
        if cmd.startswith('unload'):
            return 'unloaded'
        return None


def main():
    rclpy.init()
    node = MechanismNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
