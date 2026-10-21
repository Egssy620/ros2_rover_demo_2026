#!/usr/bin/env python3
# マスターの UI (仮, 2026-10-05): マスターと各ノードのトピックを読むだけの別ノード (開始ボタンだけ /master_node/start を呼ぶ)
#   車の計器盤のように「今なにをしているか」「異常はないか」が一目で分かることを優先する。
#   地図 (位置・目標・経路・山) は RViz で見る (launch が画面の右側に RViz、左側にこの UI を並べる)。
#
#   見た目の決まり (2026-10-06): 通常は灰と白だけ。色は 注意 (黄) / 異常 (赤) / 操作が必要 (青) の時だけ。
#     枠・角丸・影・光る効果は使わず、区切りは細い線と余白だけ
#
#   上から
#     異常       : 通常は灰で「異常なし」。異常は 1 件 1 行で「何が起きたか — どうすればよいか」(赤 = 異常 / 黄 = 注意)
#                  続いている状態 (直ったら自動で消える): ノードが切断 / 自己位置が来ない・古い / UI 自身の受信が止まった
#                  出来事 (青の「確認」を押すまで残る)  : master_node と経路計画 (代役) の WARN (黄) / ERROR (赤) ログ (/rosout)
#                  右端に日付と時刻 (小さく)
#     今の状態   : 段階 (待機 / スクラップ山 / ピックアップ / 荷降ろし / 終了) をいちばん大きく。
#                  下にフェーズと、移動中は目標までの距離 [m]、それ以外 (把持中など) はそのフェーズの経過時間
#                  (把持中に m が出ると混乱するため 2026-10-05)。続いて段階の中の流れ と 段階の並び (今が白)
#                  開始ボタンは押せるとき (待機・終了) だけ出す
#     位置・速度 : 左に現在と目標の x / y / θ (map)、右に 指令 (/cmd_vel) と オドメトリ (/odom の twist) の x / y / ω (機体座標)
#                  速度は 0.5 s 以上来なければ薄く表示 (簡易移動は止まっている間 /cmd_vel を出さない)
#     システム   : 相手ごとの 接続 / 切断 (ROS のノード一覧にいるか)・最後に届いた結果や依頼 (日本語に直す)。異常な行だけ色を付ける
#     電力       : 電池 (/battery_state, sensor_msgs/BatteryState) から 今の電力 [W] = 電圧 × 電流、
#                  使用電力量 [Wh] = 電力を UI 起動から足し上げ、電圧 [V]。台車がつながるまではシミュの仮の値
#     記録       : フェーズの切り替わり・結果・依頼・異常。直近 5 行だけ見せ、前の分はスクロール
#
#   作り: ROS の係 (UiNode, 別スレッド) はトピックを受けたら put() で箱 (queue) に入れるだけ。
#         画面の係 (Ui, Qt) が 0.05 秒ごとの poll() で取り出して画面を書き換える。画面の部品は画面の係からしか触らない
#   見た目: Qt (PyQt5) で描く。tkinter では線のなめらかさ・角丸・細い字 (Light など) が使えないため
import math
import queue
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QTextEdit,
                             QVBoxLayout, QWidget)
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

# master_node.cpp の phaseName と同じ並び・同じ名前にする (名前を変えたらここも変える)
#   (フェーズ名, 段階, 段階の中での表示名)
PHASES = [
    ('待機', '待機', '待機'),
    ('スクラップ山: 経路生成', 'スクラップ山', '経路生成'),
    ('スクラップ山: 移動', 'スクラップ山', '移動'),
    ('スクラップ山: 検出', 'スクラップ山', '検出'),
    ('ピックアップ: 経路生成', 'ピックアップ', '経路生成'),
    ('ピックアップ: 移動', 'ピックアップ', '移動'),
    ('ピックアップ', 'ピックアップ', '把持'),
    ('荷降ろし場所探索', '荷降ろし', '場所探索'),
    ('荷降ろし: 経路生成', '荷降ろし', '経路生成'),
    ('荷降ろし: 移動', '荷降ろし', '移動'),
    ('荷降ろし', '荷降ろし', '荷降ろし'),
    ('終了', '終了', '終了'),
]
STAGES = ['待機', 'スクラップ山', 'ピックアップ', '荷降ろし', '終了']
START_PHASES = ('待機', '終了')  # 開始ボタンを出すフェーズ
# 相手: 表示名 → ROS のノード名 (None = ノード名で生存確認しない)
PARTNERS = [
    ('マスター', 'master_node'),
    ('自己位置', 'localizer_node'),
    ('経路計画', 'simple_goal_follower_node'),
    ('把持計画', 'pickup_planner_node'),
    ('機構部', None),
]
DISPLAY_NAME = {ros_name: name for name, ros_name in PARTNERS if ros_name}  # ノード名 → 表示名
# 相手から届く結果の言葉 (simple_goal_follower_node.cpp / master_node.cpp と同じ) → 画面に出す日本語。知らない言葉はそのまま出す
RESULT_TEXT = {
    'planned': '経路生成済み', 'arrived': '到着', 'blocked': '進めず停止',
    'grasped': '把持成功', 'missed': '把持失敗', 'full': '回収箱が満杯', 'unloaded': '荷降ろし完了',
}


def request_text(cmd):
    """マスターから機構部への依頼 (/mechanism/command: grasp / unload N) → 日本語"""
    if cmd == 'grasp':
        return '把持を依頼'
    if cmd.startswith('unload '):
        return f'荷降ろし場所 {cmd.split()[1]} に降ろすよう依頼'
    return f'依頼 {cmd}'
# 異常として拾うログの出し手 (ノード名)。把持計画のエラーはマスターが同じ内容を ERROR で出すので入れない
WATCH_LOGS = ('master_node', 'simple_goal_follower_node')
WARN_LEVEL, ERROR_LEVEL = ord(Log.WARN), ord(Log.ERROR)  # Log の定数は bytes (b'\x1e')、受け取る level は int なので数に直して比べる
LOG_LINES = 5  # 記録欄に見せる行数
POSE_TIMEOUT = 1.0  # 自己位置がこれ [s] 以上来なければ異常 (localizer は検出のたび ~10 Hz で出す)


def use_nav_node(ros_name):
    """経路計画のノード名を差し替える (簡易移動 simple_goal_follower_node / 林研 manage_node_master)。窓を作る前に呼ぶ"""
    global WATCH_LOGS
    PARTNERS[:] = [(name, ros_name if name == '経路計画' else r) for name, r in PARTNERS]
    DISPLAY_NAME.clear()
    DISPLAY_NAME.update({r: name for name, r in PARTNERS if r})
    WATCH_LOGS = ('master_node', ros_name)

# ===== 見た目 =====
#   色: 通常は灰と白だけ。色は下の 3 つだけで、使うのは人が気にすべき時だけ (色を変えたらここだけ直す)
#       色だけに頼らない: 色を付ける所には必ず「切断」などの文字も出す
#   字: 日本語 Noto Sans CJK JP、数字は全部 Ubuntu Mono (桁がそろう)。単位は数字より小さく薄い灰
BG = '#16181b'  # 暗い灰
WHITE = '#f3f4f5'
CAUTION = '#f5c84c'  # 注意: 値が古い・遅い など
ALERT = '#ff7b72'    # 異常: ノード切断・停止 など
ACTION = '#79b8ff'   # 操作が必要: 人が押す・確認する
JP, MONO = 'Noto Sans CJK JP', 'Ubuntu Mono'


def color(hexstr, alpha=1.0):
    c = QColor(hexstr)
    c.setAlphaF(alpha)
    return c


def css(c):
    return f'rgba({c.red()},{c.green()},{c.blue()},{c.alphaF():.3f})'


# 文字は 3 段 (白 / 明るい灰 / 灰)。いちばん薄い TEXT3 でも背景 BG との明るさの比 7:1 以上 (屋外で読めるように)
TEXT, TEXT2, TEXT3 = QColor(WHITE), QColor('#c9cdd2'), QColor('#a4a8ad')
LINE = QColor('#34373c')  # 区切り線・まだの段 (文字には使わない)
DONE = QColor('#6b7077')  # 済んだ段 (文字には使わない)


def font(family, px, weight=QFont.Normal, spacing=0.0):
    f = QFont(family)
    f.setPixelSize(px)
    f.setWeight(weight)
    if spacing:
        f.setLetterSpacing(QFont.AbsoluteSpacing, spacing)
    return f


def label(text, fnt, col, align=Qt.AlignLeft | Qt.AlignVCenter):
    lb = QLabel(text)
    lb.setFont(fnt)
    lb.setStyleSheet(f'color: {css(col)}; background: transparent;')
    lb.setAlignment(align)
    return lb


def unit(text, px=12):
    """数値の後ろに付ける単位 (数値より小さく、薄い灰)。QLabel の文字に埋め込む"""
    return f'<span style="font-size:{px}px; color:{css(TEXT3)}">{text}</span>'


def set_color(lb, col):
    lb.setStyleSheet(f'color: {css(col)}; background: transparent;')


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class UiNode(Node):
    def __init__(self, events):
        super().__init__('master_ui')
        self.events = events  # (種類, 値) を UI スレッドへ渡す
        self.geometry = self.declare_parameter('geometry', '').value  # 窓の位置と大きさ "幅x高さ+X+Y" (launch が決める)
        use_nav_node(self.declare_parameter('nav_node', 'simple_goal_follower_node').value)  # 経路計画のノード名
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/master_node/phase', lambda m: self.put('phase', m.data), latched)
        self.create_subscription(PoseStamped, '/robot_pose', lambda m: self.put('pose', m), 10)
        self.create_subscription(PoseStamped, '/goal_pose', lambda m: self.put('goal', m), 10)
        self.create_subscription(String, '/navigation/result',
                                 lambda m: self.put('result', ('経路計画', RESULT_TEXT.get(m.data, m.data))), 10)
        self.create_subscription(PoseStamped, '/pickup_pose',
                                 lambda m: self.put('result', ('把持計画', f'物品あり ({m.pose.position.x:.2f}, {m.pose.position.y:.2f})')), 10)
        self.create_subscription(String, '/mechanism/result',
                                 lambda m: self.put('result', ('機構部', RESULT_TEXT.get(m.data, m.data))), 10)
        self.create_subscription(String, '/mechanism/command',
                                 lambda m: self.put('request', ('機構部', request_text(m.data))), 10)
        self.create_subscription(PointStamped, '/mechanism/grasp_point',
                                 lambda m: self.put('request', ('機構部', f'把持点 ({m.point.x:.2f}, {m.point.y:.2f}, {m.point.z:.2f}) を送信')), 10)
        # 異常用: 各ノードのログ (WARN 以上だけ渡す)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(BatteryState, '/battery_state', lambda m: self.put('battery', m), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.put('speed', ('指令', m)), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.put('speed', ('オドメトリ', m.twist.twist)), 10)
        self.start_cli = self.create_client(Trigger, '/master_node/start')
        self.create_timer(1.0, lambda: self.put('nodes', set(self.get_node_names())))

    def on_log(self, m):
        if m.level >= WARN_LEVEL and m.name in WATCH_LOGS:
            self.put('warn', (m.name, m.msg, m.level >= ERROR_LEVEL))  # 最後 = ERROR 以上か (色を変える)

    def put(self, kind, value):
        self.events.put((kind, value))

    def call_start(self):
        if not self.start_cli.service_is_ready():
            self.put('log', '開始できない: /master_node/start が見つからない')
            return
        fut = self.start_cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self.put('log', f'開始: {f.result().message}'))


# ===== 部品 =====
class Panel(QWidget):
    """区切り: 上に細い線を 1 本引き、title があれば左上に小さく書く (枠・塗り・角丸はなし)"""

    def __init__(self, title=None, margins=(0, 12, 0, 6), line=True):
        super().__init__()
        self.title, self.line = title, line
        self.body = QVBoxLayout(self)
        l, t, r, b = margins
        self.body.setContentsMargins(l, t + (24 if title else 0), r, b)
        self.body.setSpacing(6)

    def paintEvent(self, e):
        p = QPainter(self)
        if self.line:
            p.setPen(QPen(LINE, 1))
            p.drawLine(0, 0, self.width(), 0)
        if self.title:
            p.setPen(TEXT3)
            p.setFont(font(JP, 13, QFont.Medium))
            p.drawText(0, 28, self.title)


class Steps(QWidget):
    """段階の中の流れ: 済み=灰、今=白、まだ=暗い線。下に名前"""

    def __init__(self):
        super().__init__()
        self.setFixedHeight(46)
        self.steps, self.cur = [], -1

    def set(self, steps, cur):
        self.steps, self.cur = steps, cur
        self.update()

    def paintEvent(self, e):
        if len(self.steps) < 2:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        gap = 8
        seg = (self.width() - gap * (len(self.steps) - 1)) / len(self.steps)
        for i, name in enumerate(self.steps):
            x = i * (seg + gap)
            col = TEXT if i == self.cur else (DONE if i < self.cur else LINE)
            p.setPen(Qt.NoPen)
            p.setBrush(col)
            p.drawRect(QRectF(x, 4, seg, 3))
            p.setPen(TEXT if i == self.cur else TEXT3)
            p.setFont(font(JP, 13, QFont.Medium if i == self.cur else QFont.Normal))
            p.drawText(QRectF(x, 16, seg, 26), Qt.AlignLeft | Qt.AlignVCenter, name)


class Timeline(QWidget):
    """段階を点と線で横に並べる。今の段階の点を白く塗る"""

    def __init__(self):
        super().__init__()
        self.setFixedHeight(56)
        self.cur = -1

    def set(self, cur):
        if cur != self.cur:
            self.cur = cur
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        n, m = len(STAGES), 46
        xs = [m + (self.width() - 2 * m) * i / (n - 1) for i in range(n)]
        y = 16
        p.setPen(QPen(LINE, 2))
        p.drawLine(int(xs[0]), y, int(xs[-1]), y)
        for i, (x, name) in enumerate(zip(xs, STAGES)):
            on = i == self.cur
            if on:
                p.setPen(Qt.NoPen)
                p.setBrush(TEXT)
                p.drawEllipse(QRectF(x - 5.5, y - 5.5, 11, 11))
            else:
                p.setPen(QPen(TEXT3, 1.5))
                p.setBrush(QColor(BG))
                p.drawEllipse(QRectF(x - 4, y - 4, 8, 8))
            p.setPen(TEXT if on else TEXT3)
            p.setFont(font(JP, 13 if on else 12, QFont.Medium if on else QFont.Normal))
            p.drawText(QRectF(x - 60, y + 16, 120, 24), Qt.AlignHCenter | Qt.AlignVCenter, name)


class Button(QPushButton):
    """枠線だけのボタン。押すと枠の色で塗る"""

    def __init__(self, text, col, fill=False):
        super().__init__(text)
        self.setCursor(Qt.PointingHandCursor)
        self.setFont(font(JP, 14, QFont.Medium, 2))
        dark = '#111317'
        self.setStyleSheet(f'''
            QPushButton {{ color: {dark if fill else col}; background: {col if fill else 'transparent'};
                           border: 1px solid {col}; border-radius: 2px; padding: 7px 28px; }}
            QPushButton:hover {{ color: {dark}; background: {col}; }}
            QPushButton:pressed {{ background: {css(color(col, 0.7))}; }}''')


class Window(QWidget):
    """背景 (暗い灰の一色)"""

    def paintEvent(self, e):
        QPainter(self).fillRect(self.rect(), QColor(BG))


# ===== 画面 =====
class Ui:
    def __init__(self, node, events, ros_thread):
        self.node, self.events, self.ros_thread = node, events, ros_thread
        self.pose, self.pose_rx = None, 0.0
        self.goal = None
        self.phase = None
        self.warn_events = {}   # (ノード, 内容, ERROR か) → [最初の時刻, 最後の時刻, 回数]  「確認」で消す
        self.nodes = None       # ROS のノード一覧 (まだ来ていなければ None)
        self.alarm_shown = None  # 異常欄に今出している内容 (変わった時だけ書き直す)

        w = self.win = Window()
        w.setWindowTitle('マスター')
        self.place(node.geometry)
        root = QVBoxLayout(w)
        root.setContentsMargins(26, 16, 26, 16)
        root.setSpacing(9)

        # ===== 異常 (いちばん上に固定) =====
        #   通常は灰で「異常なし」。異常は 1 件 1 行で「何が起きたか — どうすればよいか」を黄 (注意) / 赤 (異常) で
        arow = QHBoxLayout()
        arow.setSpacing(14)
        self.alarm_body = label('', font(JP, 15, QFont.Medium), TEXT3)
        self.alarm_body.setWordWrap(True)
        arow.addWidget(self.alarm_body, 1, Qt.AlignTop)
        self.ack = Button('確認', ACTION)
        self.ack.clicked.connect(self.acknowledge)
        arow.addWidget(self.ack, 0, Qt.AlignTop)
        self.clock = label('', font(JP, 13), TEXT3, Qt.AlignRight | Qt.AlignTop)
        arow.addWidget(self.clock, 0, Qt.AlignTop)
        root.addLayout(arow)

        # ===== 今の状態: 段階 (いちばん大きく) / フェーズと進み / 段階の中の流れ / 段階の並び =====
        hero = Panel()
        row = QHBoxLayout()
        hero.body.addLayout(row)
        col = QVBoxLayout()
        col.setSpacing(0)
        self.stage_label = label('—', font(JP, 46, QFont.Medium), TEXT)
        # フェーズ名と、移動中は目標までの距離・それ以外 (把持中など) はそのフェーズの経過時間 (把持中に m が出ると混乱するため)
        self.progress = label('', font(JP, 18), TEXT2)
        col.addWidget(self.stage_label)
        col.addWidget(self.progress)
        row.addLayout(col, 1)
        self.start = Button('開始', WHITE)  # 押せるとき (待機・終了) だけ出す
        self.start.clicked.connect(node.call_start)
        row.addWidget(self.start, 0, Qt.AlignTop)
        hero.body.addSpacing(10)
        self.steps = Steps()
        hero.body.addWidget(self.steps)
        self.timeline = Timeline()
        hero.body.addWidget(self.timeline)
        root.addWidget(hero)

        # ===== 位置 と 速度 (左右に並べる): 見出し → 列の名前 → 2 行 =====
        mid = QHBoxLayout()
        mid.setSpacing(28)
        self.pos_cells, self.speed_cells, self.speed_rx = {}, {}, {}

        def table(title, columns, names, cells):
            box = Panel(title)
            pg = QGridLayout()
            pg.setHorizontalSpacing(6)
            pg.setVerticalSpacing(4)
            for c, (axis, u) in enumerate(columns, start=1):
                pg.addWidget(label(f'{axis} {unit(u)}', font(MONO, 15), TEXT2, Qt.AlignRight | Qt.AlignVCenter), 0, c)
                pg.setColumnStretch(c, 1)
            for r, name in enumerate(names, start=1):
                pg.addWidget(label(name, font(JP, 14), TEXT2), r, 0)
                cells[name] = []
                for c in range(1, 4):
                    v = label('—', font(MONO, 20), TEXT, Qt.AlignRight | Qt.AlignVCenter)
                    pg.addWidget(v, r, c)
                    cells[name].append(v)
            box.body.addLayout(pg)
            mid.addWidget(box, 1)

        table('位置  map', [('x', 'm'), ('y', 'm'), ('θ', '°')], ['現在', '目標'], self.pos_cells)
        table('速度  機体座標', [('x', 'm/s'), ('y', 'm/s'), ('ω', '°/s')], ['指令', 'オドメトリ'], self.speed_cells)
        root.addLayout(mid)

        # ===== システム: 通常は小さく灰。異常な相手の行だけ色を付ける =====
        sysp = Panel('システム')
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(2)
        grid.setColumnStretch(2, 1)
        self.names, self.status, self.last = {}, {}, {}
        # 1 行に: 名前 / 接続 (名前のすぐ隣) / 最後の結果・依頼 (長い文は切る)
        for r, (name, _) in enumerate(PARTNERS):
            self.names[name] = label(name, font(JP, 13), TEXT2)
            grid.addWidget(self.names[name], r, 0)
            self.status[name] = label('', font(JP, 13), TEXT3)
            grid.addWidget(self.status[name], r, 1)
            self.last[name] = label('', font(JP, 12), TEXT3)
            self.last[name].setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            grid.addWidget(self.last[name], r, 2)
        sysp.body.addLayout(grid)
        root.addWidget(sysp)

        # ===== 電力 =====
        powp = Panel('電力')
        nums = QHBoxLayout()
        nums.setSpacing(26)
        self.power_vals = {}
        for key, u in (('今', 'W'), ('使用電力量', 'Wh'), ('電圧', 'V')):
            box = QVBoxLayout()
            box.setSpacing(0)
            box.addWidget(label(key, font(JP, 13), TEXT2))
            val = QHBoxLayout()
            val.setSpacing(4)
            self.power_vals[key] = label('—', font(MONO, 26), TEXT)
            val.addWidget(self.power_vals[key])
            val.addWidget(label(u, font(MONO, 13), TEXT3), 0, Qt.AlignBottom)
            val.addStretch()
            box.addLayout(val)
            nums.addLayout(box)
        nums.addStretch()
        powp.body.addLayout(nums)
        root.addWidget(powp)
        self.energy_wh, self.batt_rx = 0.0, None

        # ===== 記録: 直近 LOG_LINES 行だけ見せる (運転中はあまり見ないため)。前の分はスクロールで見られる =====
        logp = Panel('記録')
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFrameStyle(0)
        self.log.document().setMaximumBlockCount(500)
        self.log.setStyleSheet(f'''
            QTextEdit {{ background: transparent; color: {css(TEXT2)}; }}
            QScrollBar:vertical {{ background: transparent; width: 6px; }}
            QScrollBar::handle:vertical {{ background: {css(LINE)}; }}
            QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}''')
        self.log.setFont(font(JP, 12))
        self.log.setFixedHeight(QFontMetrics(self.log.font()).lineSpacing() * LOG_LINES
                                + int(2 * self.log.document().documentMargin()))
        logp.body.addWidget(self.log)
        root.addWidget(logp)
        root.addStretch()

        self.set_phase(None)
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)
        w.show()

    def place(self, geometry):
        """launch が決めた "幅x高さ+X+Y" に置く"""
        try:
            size, x, y = geometry.split('+')
            wd, ht = size.split('x')
            self.win.setGeometry(int(x), int(y), int(wd), int(ht))
        except ValueError:
            self.win.resize(740, 1000)

    # ===== 記録 =====
    def add_log(self, text, col=None):
        t = time.strftime('%H:%M:%S')
        self.log.append(f'<span style="font-family:{MONO}; color:{css(TEXT3)}">{t}</span>'
                        f'&nbsp;&nbsp;&nbsp;<span style="color:{css(col or TEXT2)}">{text}</span>')

    # ===== 状態 =====
    def acknowledge(self):
        """出来事の異常を見たことにして消す (続いている状態の異常は直るまで消えない)"""
        self.warn_events.clear()
        self.add_log('異常を確認')

    def conditions(self):
        """今続いている異常の一覧 [(色, 何が起きたか, どうすればよいか)]"""
        out = []
        if not self.ros_thread.is_alive():  # コールバックで例外が出ると受信が止まる (UI 起動ターミナルにエラー)
            out.append((ALERT, 'UI の ROS 受信が停止', 'UI を起動したターミナルのエラーを見て UI を起動し直す'))
        if self.nodes is not None:
            out += [(ALERT, f'{name}が切断', f'{ros_name} を起動し直す') for name, ros_name in PARTNERS
                    if ros_name and ros_name not in self.nodes]
        if self.pose is None:
            out.append((CAUTION, '自己位置がまだ来ていない', 'ランドマークが見えているか確かめる'))
        elif time.time() - self.pose_rx > POSE_TIMEOUT:
            out.append((CAUTION, f'自己位置が {time.time() - self.pose_rx:.0f} 秒古い', 'ランドマークの見え方と自己位置ノードを確かめる'))
        return out

    def update_alarm(self):
        items = self.conditions()
        for (name, text, error), (_, last, n) in sorted(self.warn_events.items(), key=lambda kv: -kv[1][1])[:4]:
            items.append((ALERT if error else CAUTION,
                          f'{time.strftime("%H:%M:%S", time.localtime(last))} {DISPLAY_NAME.get(name, name)}: {text}'
                          + (f' ×{n}' if n > 1 else ''), '記録を見て「確認」を押す'))
        if len(self.warn_events) > 4:
            items.append((CAUTION, f'ほか {len(self.warn_events) - 4} 件', '記録を見る'))
        items.sort(key=lambda it: it[0] != ALERT)  # 赤を先に
        shown = (tuple(items), bool(self.warn_events))
        if shown == self.alarm_shown:
            return
        self.alarm_shown = shown
        if items:
            self.alarm_body.setText('<br>'.join(
                f'<span style="color:{c}">{what}</span><span style="color:{css(TEXT2)}"> — {todo}</span>'
                for c, what, todo in items))
        else:
            self.alarm_body.setText(f'<span style="color:{css(TEXT3)}">異常なし</span>')
        self.ack.setVisible(bool(self.warn_events))

    # ===== 今の作業・段階 =====
    def set_phase(self, name):
        self.phase, self.phase_since = name, time.time()
        info = next((p for p in PHASES if p[0] == name), None)
        self.stage_label.setText(info[1] if info else (name or '—'))
        steps = [p for p in PHASES if info and p[1] == info[1]]
        self.steps.set([p[2] for p in steps], [p[0] for p in steps].index(name) if info else -1)
        self.steps.setVisible(len(steps) > 1)
        self.timeline.set(STAGES.index(info[1]) if info else -1)
        self.start.setVisible(name in START_PHASES)
        if name:
            self.add_log(f'→ {name}', TEXT)

    def on_battery(self, m):
        now = time.time()
        power = abs(m.voltage * m.current)  # [W] (放電中は電流が負)
        if self.batt_rx is not None:
            self.energy_wh += power * (now - self.batt_rx) / 3600.0
        self.batt_rx = now
        self.power_vals['今'].setText(f'{power:.0f}')
        self.power_vals['使用電力量'].setText(f'{self.energy_wh:.1f}')
        self.power_vals['電圧'].setText(f'{m.voltage:.1f}')

    def update_progress(self):
        """移動中は目標までの距離、それ以外はこのフェーズの経過時間 (分:秒)"""
        info = next((p for p in PHASES if p[0] == self.phase), None)
        if info and info[2] == '移動' and self.pose is not None and self.goal is not None:
            p, g = self.pose.pose.position, self.goal.position
            d = math.hypot(g.x - p.x, g.y - p.y)
            self.progress.setText(f'{info[2]} ・ 目標まで <span style="font-family:{MONO}">{d:.1f}</span> {unit("m", 14)}')
        elif self.phase is not None:
            sec = int(time.time() - self.phase_since)
            step = f'{info[2]} ・ ' if info and info[1] != info[2] else ''
            self.progress.setText(f'{step}経過 <span style="font-family:{MONO}">{sec // 60}:{sec % 60:02d}</span>')
        else:
            self.progress.setText('')

    def set_partner(self, name, text, col):
        """システムの 1 行: 正常 (col=None) は灰、異常は名前と状態を col で"""
        self.status[name].setText(text)
        set_color(self.status[name], QColor(col) if col else TEXT3)
        set_color(self.names[name], QColor(col) if col else TEXT2)

    def set_speed(self, name, tw):
        self.speed_rx[name] = time.time()
        for lb, text in zip(self.speed_cells[name], (f'{tw.linear.x:.2f}', f'{tw.linear.y:.2f}',
                                                     f'{math.degrees(tw.angular.z):.1f}')):
            lb.setText(text)

    def set_pos(self, name, pose):
        x, y, th = pose.position.x, pose.position.y, yaw_deg(pose.orientation)
        for lb, text in zip(self.pos_cells[name], (f'{x:.2f}', f'{y:.2f}', f'{th:.1f}')):  # 単位は列の名前に
            lb.setText(text)

    # ===== 0.05 秒ごと =====
    def poll(self):
        while not self.events.empty():
            kind, v = self.events.get()
            if kind == 'phase':
                self.set_phase(v)
            elif kind == 'pose':
                self.pose, self.pose_rx = v, time.time()
            elif kind == 'goal':
                self.goal = v.pose
                self.set_pos('目標', v.pose)
                self.add_log(f'目標 ({v.pose.position.x:.2f}, {v.pose.position.y:.2f})')
            elif kind in ('result', 'request'):
                name, text = v
                self.last[name].setText(text)
                self.add_log(f'{name}: {text}')
            elif kind == 'warn':
                now = time.time()
                if v in self.warn_events:
                    self.warn_events[v][1] = now
                    self.warn_events[v][2] += 1
                else:
                    self.warn_events[v] = [now, now, 1]
                    self.add_log(f'{DISPLAY_NAME.get(v[0], v[0])}: {v[1]}', QColor(ALERT if v[2] else CAUTION))
            elif kind == 'nodes':
                self.nodes = v
                for name, ros_name in PARTNERS:
                    if ros_name and name != '自己位置':
                        up = ros_name in v
                        self.set_partner(name, '接続' if up else '切断', None if up else ALERT)
            elif kind == 'log':
                self.add_log(v, TEXT)
            elif kind == 'speed':
                self.set_speed(*v)
            elif kind == 'battery':
                self.on_battery(v)
        # 速度: 0.5 s 以上来なければ薄く (止まっている)
        for name, cells in self.speed_cells.items():
            old = time.time() - self.speed_rx.get(name, 0.0) > 0.5
            for lb in cells:
                set_color(lb, TEXT3 if old else TEXT)
        # 自己位置: POSE_TIMEOUT 以上来なければ古いと表示
        if self.pose is not None:
            old = time.time() - self.pose_rx > POSE_TIMEOUT
            self.set_pos('現在', self.pose.pose)
            for lb in self.pos_cells['現在']:
                set_color(lb, TEXT3 if old else TEXT)
            self.set_partner('自己位置', '古い' if old else '受信中', CAUTION if old else None)
        now = time.localtime()
        self.clock.setText(f'{now.tm_mon}月{now.tm_mday}日 ({"月火水木金土日"[now.tm_wday]})'
                           f'  <span style="font-family:{MONO}; font-size:15px">{time.strftime("%H:%M", now)}</span>')
        self.update_progress()
        self.update_alarm()
        if not rclpy.ok():  # launch を止めたら窓も閉じる
            QApplication.quit()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *a: QApplication.quit())  # Ctrl+C で閉じる
    events = queue.Queue()
    node = UiNode(events)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    ui = Ui(node, events, ros_thread)  # noqa: F841 (窓が消えないように持っておく)
    app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
