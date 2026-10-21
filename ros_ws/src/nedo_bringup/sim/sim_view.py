# 簡易シミュ: 箱形の山 (map系) + /cmd_vel 積分 → /robot_pose + 点群 (base_footprint系) + /sim/pick で帯内の点を除去
import math, numpy as np, rclpy, struct
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
X0, X1, Y0, Y1, H, G = 2.0, 3.5, -1.2, 1.5, 0.6, 0.05
# 山: 中央 (y=0.15) と奥 (x大) ほど高い。高さ 0.2〜1.0 m
xs, ys, zs = np.meshgrid(np.arange(X0, X1, G), np.arange(Y0, Y1, G), np.arange(0.1, 1.05, 0.1), indexing='ij')
Hxy = 0.2 + 0.6 * (1 - np.abs(ys - 0.15) / 1.35) + 0.2 * (xs - X0) / (X1 - X0)
keep = zs <= Hxy
P = np.stack([xs[keep], ys[keep], zs[keep]], 1)
ITEM = 0.25   # 1回の把持で取れる厚み
PICK_DELAY = 3.0
import time
class Sim(Node):
    def __init__(s):
        super().__init__('sim')
        s.x, s.y, s.yaw = 0.0, 0.3, -math.pi / 2 + 0.2; print('pile points', len(P), flush=True)   # アーム側 (左の長辺) を山に向け、少し斜め・横ずれから開始 (v3 は向きを変えない)
        s.v = (0.0, 0.0, 0.0); s.grasp = None; s.P = P.copy(); s.picks = 0
        s.pp = s.create_publisher(PoseStamped, '/robot_pose', 10)
        s.pc = s.create_publisher(PointCloud2, '/hesai_qt128/lidar_points', 10)
        s.create_subscription(Twist, '/cmd_vel', lambda m: setattr(s, 'v', (m.linear.x, m.linear.y, m.angular.z)), 10)
        s.create_service(Trigger, '/sim/pick', s.pick); s.grasp = None
        s.create_timer(0.05, s.step); s.create_timer(0.1, s.cloud)
        # rviz 用: map→base_link の TF、山の真値 (map 系、灰色)、把持点が出たら PICK_DELAY 秒後に手で取った扱い
        s.tfb = TransformBroadcaster(s); s.truth = s.create_publisher(PointCloud2, '/sim/pile_truth', 1)
        s.create_timer(0.5, s.pub_truth); s.pick_at = None
        s.create_subscription(PointStamped, '/pickup_planner_node/grasp_point', s.on_grasp, 10)
        s.create_timer(0.1, s.auto_pick)
    def step(s):
        vx, vy, w = s.v; c, sn = math.cos(s.yaw), math.sin(s.yaw)
        s.x += 0.05 * (c * vx - sn * vy); s.y += 0.05 * (sn * vx + c * vy); s.yaw += 0.05 * w
        m = PoseStamped(); m.header.frame_id = 'map'; m.header.stamp = s.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = s.x, s.y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
        s.pp.publish(m)
        t = TransformStamped(); t.header = m.header; t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = s.x, s.y, 0.3
        t.transform.rotation = m.pose.orientation; s.tfb.sendTransform(t)
    def body(s):
        d = s.P[:, :2] - [s.x, s.y]; c, sn = math.cos(s.yaw), math.sin(s.yaw)
        return np.stack([c * d[:, 0] + sn * d[:, 1], -sn * d[:, 0] + c * d[:, 1]], 1)
    def cloud(s):
        b = s.body(); pts = np.concatenate([b, s.P[:, 2:]], 1).astype(np.float32)
        m = PointCloud2(); m.header.frame_id = 'base_footprint'; m.header.stamp = s.get_clock().now().to_msg()
        m.height, m.width = 1, len(pts); m.point_step = 12; m.row_step = 12 * len(pts); m.is_dense = True
        m.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate('xyz')]
        m.data = pts.tobytes(); s.pc.publish(m)
    def on_grasp(s, m):
        s.grasp = m.point; s.pick_at = time.time() + PICK_DELAY
        print(f'grasp point ({m.point.x:.2f},{m.point.y:.2f}) z={m.point.z:.2f} -> {PICK_DELAY:.0f} s 後に取ります', flush=True)
    def auto_pick(s):
        if s.pick_at and time.time() > s.pick_at:
            s.pick_at = None; r = s.pick(None, Trigger.Response()); print(r.message, '  → Enter で次へ', flush=True)
    def pub_truth(s):
        pts = s.P.astype(np.float32)
        m = PointCloud2(); m.header.frame_id = 'map'; m.header.stamp = s.get_clock().now().to_msg()
        m.height, m.width = 1, len(pts); m.point_step = 12; m.row_step = 12 * len(pts); m.is_dense = True
        m.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate('xyz')]
        m.data = pts.tobytes(); s.truth.publish(m)
    def pick(s, req, res):
        # 把持点の円 (直径0.6) 内で、上面から ITEM の厚みだけ取る
        g = s.grasp; s.grasp = None
        if g is None: res.success = False; res.message = 'no grasp point'; return res
        inside = (np.hypot(s.P[:, 0] - g.x, s.P[:, 1] - g.y) <= 0.3) & (s.P[:, 2] >= g.z - ITEM)
        s.P = s.P[~inside]; s.picks += 1
        res.success = True; res.message = f'pick {s.picks}: at ({g.x:.2f},{g.y:.2f}) z={g.z:.2f} removed {inside.sum()} left {len(s.P)}'
        return res
rclpy.init(); rclpy.spin(Sim())
