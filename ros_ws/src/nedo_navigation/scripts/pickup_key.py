#!/usr/bin/env python3
# pickup_planner_node をキーボードで操作する (グラップルが無い間の手動ピックアップ用)
#   Enter      : 今取れる一番高い点と台車の目標姿勢を出す (~/next)
#   q + Enter  : 終了
# launch ではキー入力を受け取れないので、別ターミナルで ros2 run nedo_navigation pickup_key.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('pickup_key')
    ns = node.declare_parameter('planner', '/pickup_planner_node').value
    clients = {k: node.create_client(Trigger, f'{ns}/{k}') for k in ('next',)}
    keys = {'': 'next'}
    print('Enter=次へ  q=終了')
    try:
        while rclpy.ok():
            key = input('> ').strip().lower()
            if key == 'q':
                break
            name = keys.get(key)
            if name is None:
                print('Enter / q のどちらか')
                continue
            cli = clients[name]
            if not cli.wait_for_service(timeout_sec=2.0):
                print(f'{ns}/{name} が見つからない (pickup_planner_node は起動しているか)')
                continue
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            res = fut.result()
            print(('OK: ' if res and res.success else 'NG: ') + (res.message if res else '応答なし'))
    except (EOFError, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
