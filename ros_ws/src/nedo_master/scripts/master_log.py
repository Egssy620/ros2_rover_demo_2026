#!/usr/bin/env python3
# マスターのログ (到着・依頼・フェーズの切り替えなど) だけを、別のターミナルにその場で出す
#   ros2 run nedo_master master_log.py
# 各ノードのログは /rosout にも流れるので、そこから master_node の分だけ取り出す。
# ログファイルの tail -f と違い、起動し直しても止めずに見続けられ、書き込みの遅れもない (2026-10-06 本人指示)
# 開いたときに、直前のログ (/rosout に残っている分) も出る
import sys
from datetime import datetime

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

NAME = 'master_node'
# 重要度ごとの色 (端末の ANSI)。INFO は色なし
COLORS = {Log.WARN: '\033[33m', Log.ERROR: '\033[31m', Log.FATAL: '\033[31m'}
RESET = '\033[0m'


def main():
    rclpy.init()
    node = Node('master_log')
    color = sys.stdout.isatty()

    def on_log(m):
        if m.name != NAME:
            return
        t = datetime.fromtimestamp(m.stamp.sec + m.stamp.nanosec * 1e-9).strftime('%H:%M:%S')
        line = f'{t}  {m.msg}'
        if color and m.level in COLORS:
            line = COLORS[m.level] + line + RESET
        print(line, flush=True)

    # /rosout は transient_local で直前のログを残しているので、開く前の分も受け取れる
    node.create_subscription(Log, '/rosout', on_log,
                             QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    print(f'{NAME} のログを待っています (Ctrl-C で終了)', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
