#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PUMA560 正向運動學教學 GUI（PyQt5 + rclpy）

功能：
  1. θ1～θ6 拉桿（0.1° 解析度）＋ 數字輸入欄，雙向同步
  2. 即時發布 /forward_position_controller/commands（Float64MultiArray，rad）
  3. 即時顯示 ⁰₆T（可切換 ⁰₁T … ⁵₆T、¹₃T、³₆T、¹₆T），位置單位 mm / m
  4. DH 參數表（θ 欄即時更新）
  5. 預設姿態按鈕
  6. 驗證區：tf2 查詢 link_0 → link_6，與 DH 公式比較
  7. 工具中心點 TCP、θ5 奇異點提示
  8. 反向運動學面板：以目前 ⁰₆T 或「手動輸入的 ⁰₆T」反解全部 8 組解，點選即套用
"""
import math
import os
import re
import sys
from collections import deque

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from PyQt5 import QtCore, QtGui, QtWidgets

from puma560_teaching_gui.puma560_kinematics import (
    Puma560, fit_to_limits, load_dh_params, nearest_rotation, rotation_deviation,
    same_configuration)

JOINT_NAMES = ['joint_%d' % i for i in range(1, 7)]
SUP = '⁰¹²³⁴⁵⁶'
SUB = '₀₁₂₃₄₅₆'
POS_TOL_MM = 0.01      # 驗證門檻：位置誤差 < 0.01 mm
ROT_TOL = 1e-5         # 旋轉矩陣元素誤差門檻
ORTH_FIX_TOL = 1e-9   # 輸入的 R：max|RᵀR − I| 超過此值就用 SVD 修正
ORTH_REJECT_TOL = 0.05  # 超過此值視為不是旋轉矩陣（多半是打錯）

PRESETS = [
    ('全零姿態', [0, 0, 0, 0, 0, 0]),
    ('練習題 (60,−30,30,40,45,−20)', [60, -30, 30, 40, 45, -20]),
    ('奇異姿態 θ5=0 (30,−20,40,50,0,−30)', [30, -20, 40, 50, 0, -30]),
]

# 可切換顯示的轉換：(frm, to)
TRANSFORMS = [(0, 6)] + [(i, i + 1) for i in range(6)] + [(1, 3), (3, 6), (1, 6)]

MONO = None  # 等寬字型；必須在 QApplication 建立後才能產生，見 main()


def make_mono_font():
    f = QtGui.QFont('DejaVu Sans Mono')
    f.setStyleHint(QtGui.QFont.Monospace)
    f.setPointSize(11)
    return f

COLOR_R = QtGui.QColor('#dbe9ff')     # 旋轉部分底色
COLOR_P = QtGui.QColor('#ffe8cc')     # 位置欄底色
COLOR_LAST = QtGui.QColor('#eeeeee')  # 最後一列
TEXT = QtGui.QColor('#111111')


def tname(frm, to):
    """⁰₆T 這類記號。"""
    return '%s%sT' % (SUP[frm], SUB[to])


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def default_limits_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('puma560_description'),
                            'config', 'joint_limits.yaml')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(
            here, '..', '..', 'puma560_description', 'config', 'joint_limits.yaml'))


# =====================================================================
# ROS 節點：發布命令、訂閱 joint_states、tf2 查詢
# =====================================================================
class TeachingNode(Node):
    def __init__(self):
        super().__init__('puma560_teaching_gui')
        self.declare_parameter('joint_limits', 'default')
        self.declare_parameter('command_topic', '/forward_position_controller/commands')
        topic = self.get_parameter('command_topic').value
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.js_hist = deque(maxlen=50)   # (stamp, q[6])，依名稱對應後的關節角
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_js(self, msg: JointState):
        # 注意：/joint_states 的順序不一定是 joint_1..6，必須依「名稱」對應
        idx = {n: k for k, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        q = [msg.position[idx[n]] for n in JOINT_NAMES]
        self.js_hist.append((Time.from_msg(msg.header.stamp), q))

    def publish(self, q_rad):
        self.pub.publish(Float64MultiArray(data=[float(v) for v in q_rad]))

    def lookup_06(self, stamp=None):
        """查詢 link_0 → link_6，回傳 4×4（m）。stamp=None 表示最新。"""
        tf = self.tf_buffer.lookup_transform('link_0', 'link_6', stamp or Time())
        t, r = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix(r.x, r.y, r.z, r.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def lookup_synced(self):
        """找一筆 joint_states，並查詢「同一時間戳」的 TF，
        用來檢查 URDF 模型本身是否等於 DH（手臂移動中也成立）。"""
        for stamp, q in reversed(self.js_hist):
            try:
                return q, self.lookup_06(stamp)
            except Exception:
                continue
        return None, None


# =====================================================================
# GUI
# =====================================================================
class MatrixTable(QtWidgets.QTableWidget):
    def __init__(self):
        super().__init__(4, 4)
        self.setFont(MONO)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.verticalHeader().setVisible(False)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.setMinimumHeight(170)
        for r in range(4):
            for c in range(4):
                it = QtWidgets.QTableWidgetItem('')
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                it.setForeground(TEXT)
                it.setBackground(COLOR_LAST if r == 3 else (COLOR_P if c == 3 else COLOR_R))
                self.setItem(r, c, it)

    def set_matrix(self, T, scale):
        for r in range(4):
            for c in range(4):
                v = T[r, c] * (scale if (c == 3 and r < 3) else 1.0)
                if abs(v) < 5e-5 * (scale if c == 3 else 1):
                    v = 0.0   # 避免顯示 -0.0000
                if c == 3 and r < 3:
                    txt = '%.4f' % v
                else:
                    txt = '%.4f' % v if r < 3 else '%d' % round(v)
                self.item(r, c).setText(txt)


class TeachingWindow(QtWidgets.QWidget):
    def __init__(self, node: TeachingNode):
        super().__init__()
        self.node = node
        self.robot = Puma560(load_dh_params())
        self.q_deg = [0.0] * 6
        self.unit_mm = True
        self.view = (0, 6)
        self._load_limits()

        self.setWindowTitle('PUMA560 正向運動學教學（Craig 修正式 DH）')
        main = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        right = QtWidgets.QVBoxLayout()
        main.addLayout(left, 5)
        main.addLayout(right, 6)

        left.addWidget(self._build_sliders())
        left.addWidget(self._build_presets())
        # 左下：DH 參數表 / 反向運動學 兩個分頁
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_dh_table(), 'DH 參數表')
        self.tabs.addTab(self._build_ik(), '反向運動學（IK）')
        left.addWidget(self.tabs)
        right.addWidget(self._build_matrix())
        right.addWidget(self._build_tcp())
        right.addWidget(self._build_verify())
        right.addStretch(1)

        self.update_all(publish=True)

        # ROS spin 與驗證更新
        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.spin_timer.start(10)
        self.verify_timer = QtCore.QTimer(self)
        self.verify_timer.timeout.connect(self.update_verify)
        self.verify_timer.start(200)

    # ----------------------------------------------------------------
    def _load_limits(self):
        name = self.node.get_parameter('joint_limits').value
        try:
            with open(default_limits_path(), 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            lim = data[name]
            self.limits = [tuple(lim[n]) for n in JOINT_NAMES]
            self.limits_name = name
        except Exception as e:  # 讀不到就用 ±180°
            self.node.get_logger().warn('讀取 joint_limits 失敗（%s），改用 ±180°' % e)
            self.limits = [(-180.0, 180.0)] * 6
            self.limits_name = 'default'

    def _build_sliders(self):
        box = QtWidgets.QGroupBox('關節角 θ1～θ6（度，解析度 0.1°）'
                                  + ('　⚠ 實際關節範圍（僅供參考）' if self.limits_name == 'real' else ''))
        g = QtWidgets.QGridLayout(box)
        self.sliders, self.spins = [], []
        for i in range(6):
            lo, hi = self.limits[i]
            lab = QtWidgets.QLabel('θ%d' % (i + 1))
            lab.setFont(MONO)
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.setRange(int(round(lo * 10)), int(round(hi * 10)))   # 0.1° 為一格
            s.setSingleStep(1)
            s.setPageStep(50)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(1)
            sp.setSingleStep(0.1)
            sp.setSuffix(' °')
            sp.setFont(MONO)
            sp.setKeyboardTracking(False)   # 按 Enter 或離開欄位才套用
            sp.setMinimumWidth(100)
            s.valueChanged.connect(lambda v, k=i: self._on_slider(k, v))
            sp.valueChanged.connect(lambda v, k=i: self._on_spin(k, v))
            rng = QtWidgets.QLabel('[%g, %g]' % (lo, hi))
            rng.setStyleSheet('color: gray;')
            g.addWidget(lab, i, 0)
            g.addWidget(s, i, 1)
            g.addWidget(sp, i, 2)
            g.addWidget(rng, i, 3)
            self.sliders.append(s)
            self.spins.append(sp)
        return box

    def _build_presets(self):
        box = QtWidgets.QGroupBox('預設姿態')
        lay = QtWidgets.QGridLayout(box)
        for k, (name, q) in enumerate(PRESETS):
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(lambda _=False, qq=q: self.set_q(qq))
            lay.addWidget(b, k // 2, k % 2)
        b = QtWidgets.QPushButton('歸零（Reset）')
        b.setToolTip('關節全部歸零，並把單位、顯示矩陣、TCP 長度恢復預設')
        b.clicked.connect(self.reset)
        lay.addWidget(b, 1, 1)
        return box

    def _build_dh_table(self):
        box = QtWidgets.QGroupBox('DH 參數表（Craig 修正式 DH）')
        v = QtWidgets.QVBoxLayout(box)
        f = QtWidgets.QLabel('ⁱ⁻¹ᵢT = RotX(αᵢ₋₁) · TransX(aᵢ₋₁) · RotZ(θᵢ) · TransZ(dᵢ)')
        f.setFont(MONO)
        v.addWidget(f)
        self.dh_table = QtWidgets.QTableWidget(6, 5)
        self.dh_table.setFont(MONO)
        self.dh_table.setHorizontalHeaderLabels(['i', 'αᵢ₋₁ (°)', 'aᵢ₋₁', 'dᵢ', 'θᵢ (°)'])
        self.dh_table.verticalHeader().setVisible(False)
        self.dh_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        hh = self.dh_table.horizontalHeader()
        for c, mode in enumerate([QtWidgets.QHeaderView.ResizeToContents,
                                  QtWidgets.QHeaderView.ResizeToContents,
                                  QtWidgets.QHeaderView.Stretch,
                                  QtWidgets.QHeaderView.Stretch,
                                  QtWidgets.QHeaderView.ResizeToContents]):
            hh.setSectionResizeMode(c, mode)
        self.dh_table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.dh_table.setMinimumHeight(215)
        for r in range(6):
            for c in range(5):
                it = QtWidgets.QTableWidgetItem('')
                it.setTextAlignment(QtCore.Qt.AlignCenter)
                if c == 4:
                    it.setBackground(QtGui.QColor('#fff6bf'))
                    it.setForeground(TEXT)
                self.dh_table.setItem(r, c, it)
        v.addWidget(self.dh_table)
        return box

    def _build_ik(self):
        """反向運動學面板：以目前 ⁰₆T 或手動輸入的 ⁰₆T 反解 8 組解，點選任一列即套用到拉桿。"""
        self.ik_result = None
        box = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(box)
        top = QtWidgets.QHBoxLayout()
        btn = QtWidgets.QPushButton('以目前 ⁰₆T 反解')
        btn.setStyleSheet('font-weight: bold; padding: 4px 12px;')
        btn.clicked.connect(self.solve_ik)
        top.addWidget(btn)
        self.ik_in_toggle = QtWidgets.QPushButton('手動輸入 ⁰₆T ▸')
        self.ik_in_toggle.setCheckable(True)
        self.ik_in_toggle.setToolTip('展開輸入區：自行輸入（或貼上）⁰₆T 的數值來反解 θ1～θ6')
        top.addWidget(self.ik_in_toggle)
        self.ik_target_lab = QtWidgets.QLabel('尚未反解')
        self.ik_target_lab.setFont(MONO)
        # 目標座標可能很長，不要讓它把分頁撐寬
        self.ik_target_lab.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                         QtWidgets.QSizePolicy.Preferred)
        top.addWidget(self.ik_target_lab, 1)
        v.addLayout(top)
        self.ik_in_box = self._build_ik_input()
        self.ik_in_box.setVisible(False)
        self.ik_in_toggle.toggled.connect(self._on_ik_in_toggle)
        v.addWidget(self.ik_in_box)

        self.ik_msg_lab = QtWidgets.QLabel('')
        self.ik_msg_lab.setWordWrap(True)
        v.addWidget(self.ik_msg_lab)

        f = QtGui.QFont(MONO)
        f.setPointSize(10)
        self.ik_table = QtWidgets.QTableWidget(8, 9)
        self.ik_table.setFont(f)
        self.ik_table.setHorizontalHeaderLabels(
            ['#', 'θ1', 'θ2', 'θ3', 'θ4', 'θ5', 'θ6', '分支', 'FK 誤差'])
        self.ik_table.verticalHeader().setVisible(False)
        self.ik_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.ik_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.ik_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        hh = self.ik_table.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        hh.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(7, QtWidgets.QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(8, QtWidgets.QHeaderView.ResizeToContents)
        # 固定列高，讓 8 組解不用捲動就能全部看到
        vh = self.ik_table.verticalHeader()
        vh.setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        vh.setDefaultSectionSize(24)
        self.ik_table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ik_table.setMinimumHeight(self.ik_table.horizontalHeader().sizeHint().height() + 8 * 24 + 6)
        for r in range(8):
            for c in range(9):
                it = QtWidgets.QTableWidgetItem('')
                it.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ik_table.setItem(r, c, it)
        self.ik_table.cellClicked.connect(self._on_ik_row)
        v.addWidget(self.ik_table, 1)

        note = QtWidgets.QLabel(
            '點選任一列即套用（以完整精度送出）。分支 = θ1 的 ±√、θ3 的 ±√、'
            'F = 手腕翻轉 (θ4+180°, −θ5, θ6+180°)。★ = 目前姿態；灰字 = 超出關節範圍。')
        note.setWordWrap(True)
        note.setStyleSheet('color: gray;')
        v.addWidget(note)
        # 輸入區＋8 組解比較高，螢幕不夠高時改用捲動，避免元件互相重疊
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setWidget(box)
        return scroll

    def _build_ik_input(self):
        """手動輸入 ⁰₆T 的表格（上 3 列可編輯，最後一列固定 0 0 0 1）。"""
        box = QtWidgets.QGroupBox('輸入 ⁰₆T（位置單位跟隨右側的 mm / m 設定）')
        h = QtWidgets.QHBoxLayout(box)
        f = QtGui.QFont(MONO)
        f.setPointSize(10)
        t = QtWidgets.QTableWidget(4, 4)
        t.setFont(f)
        t.verticalHeader().setVisible(False)
        t.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        vh = t.verticalHeader()
        vh.setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        vh.setDefaultSectionSize(22)
        t.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        t.setFixedHeight(t.horizontalHeader().sizeHint().height() + 4 * 22 + 4)
        for r in range(4):
            for c in range(4):
                it = QtWidgets.QTableWidgetItem('')
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                it.setForeground(TEXT)
                it.setBackground(COLOR_LAST if r == 3 else (COLOR_P if c == 3 else COLOR_R))
                if r == 3:
                    it.setText('1' if c == 3 else '0')
                    it.setFlags(it.flags() & ~QtCore.Qt.ItemIsEditable)
                t.setItem(r, c, it)
        self.ik_in = t
        self._set_ik_input_headers()
        h.addWidget(t, 1)

        btns = QtWidgets.QVBoxLayout()
        b1 = QtWidgets.QPushButton('填入目前 ⁰₆T')
        b1.setToolTip('把拉桿目前姿態的 ⁰₆T 填進來（內部保留完整精度；改過的格子以你輸入的值為準）')
        b1.clicked.connect(self.fill_ik_input_from_current)
        b2 = QtWidgets.QPushButton('從剪貼簿貼上')
        b2.setToolTip('貼上 12 個（前 3 列）或 16 個數字，依列優先排列；\n'
                      '空白、逗號、分號、方括號都可以，例如 MATLAB / numpy 印出的矩陣')
        b2.clicked.connect(self.paste_ik_input)
        b3 = QtWidgets.QPushButton('以輸入的 ⁰₆T 反解')
        b3.setStyleSheet('font-weight: bold;')
        b3.clicked.connect(self.solve_ik_input)
        btns.addWidget(b1)
        btns.addWidget(b2)
        btns.addStretch(1)
        btns.addWidget(b3)
        h.addLayout(btns)
        self.fill_ik_input_from_current()
        return box

    def _set_ik_input_headers(self):
        unit = 'mm' if self.unit_mm else 'm'
        self.ik_in.setHorizontalHeaderLabels(['X̂6', 'Ŷ6', 'Ẑ6', '⁰P₆ORG (%s)' % unit])

    def _on_ik_in_toggle(self, on):
        self.ik_in_box.setVisible(on)
        self.ik_in_toggle.setText('手動輸入 ⁰₆T ' + ('▾' if on else '▸'))

    def _ik_cell_fmt(self, c):
        return '%.6f' if c < 3 else ('%.4f' if self.unit_mm else '%.7f')

    def _set_ik_cell(self, r, c, v):
        """顯示固定位數，完整精度的值存在 UserRole；之後文字沒被改動就用完整值。"""
        v = float(v)
        txt = self._ik_cell_fmt(c) % v
        if float(txt) == 0.0:
            txt = txt.lstrip('-')     # 避免顯示 -0.000000
        it = self.ik_in.item(r, c)
        it.setText(txt)
        it.setData(QtCore.Qt.UserRole, (txt, v))

    def _ik_cell_value(self, r, c):
        it = self.ik_in.item(r, c)
        txt = it.text().strip()
        stored = it.data(QtCore.Qt.UserRole)
        if stored is not None and stored[0] == txt:
            return stored[1]
        return float(txt.replace('−', '-'))

    def _write_ik_input(self, T, scale):
        """T 的位置單位為 m；scale = 1000 時以 mm 顯示。"""
        for r in range(3):
            for c in range(4):
                self._set_ik_cell(r, c, T[r, c] * (scale if c == 3 else 1.0))

    def fill_ik_input_from_current(self):
        self._write_ik_input(self.robot.fk(self.q_rad()), 1000.0 if self.unit_mm else 1.0)

    def paste_ik_input(self):
        text = QtWidgets.QApplication.clipboard().text()
        nums = re.findall(r'[-+−]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', text)
        if len(nums) not in (12, 16):
            self._ik_error('剪貼簿裡找到 %d 個數字，需要 12 個（前 3 列）或 16 個（完整 4×4）。' % len(nums))
            return
        for k in range(12):
            it = self.ik_in.item(k // 4, k % 4)
            it.setData(QtCore.Qt.UserRole, None)
            it.setText(nums[k].replace('−', '-'))
        self.ik_msg_lab.setText('已貼上，按「以輸入的 ⁰₆T 反解」開始計算。')
        self.ik_msg_lab.setStyleSheet('color: #1e6e2e;')

    def _read_ik_input(self):
        """讀取輸入表格，回傳位置單位為 m 的 4×4 矩陣；有格子無法解析時丟 ValueError。"""
        scale = 1000.0 if self.unit_mm else 1.0
        T = np.eye(4)
        for r in range(3):
            for c in range(4):
                try:
                    v = self._ik_cell_value(r, c)
                except ValueError:
                    raise ValueError('第 %d 列第 %d 欄「%s」不是數字'
                                     % (r + 1, c + 1, self.ik_in.item(r, c).text()))
                T[r, c] = v / scale if c == 3 else v
        return T

    def _ik_error(self, msg):
        self.ik_msg_lab.setText(msg)
        self.ik_msg_lab.setStyleSheet('color: #b00000;')

    def solve_ik(self):
        """用目前拉桿角度的 ⁰₆T 反解（手腕奇異時 θ4 取目前值）。"""
        q = self.q_rad()
        self._run_ik(self.robot.fk(q), q, [])

    def solve_ik_input(self):
        """用輸入表格中的 ⁰₆T 反解。R 不完全正交時先用 SVD 修正。"""
        try:
            T_in = self._read_ik_input()
        except ValueError as e:
            self._ik_error('輸入有誤：%s' % e)
            return
        orth, det = rotation_deviation(T_in[:3, :3])
        if det < 0 or orth > ORTH_REJECT_TOL:
            self._ik_error('輸入的 3×3 部分不是旋轉矩陣（max|RᵀR − I| = %.3g，det R = %.4f），'
                           '請檢查是否打錯或欄列顛倒。' % (orth, det))
            return
        T = T_in.copy()
        notes = []
        if orth > ORTH_FIX_TOL:
            T[:3, :3] = nearest_rotation(T_in[:3, :3])
            notes.append('輸入的 R 不完全正交（max|RᵀR − I| = %.1e，常見於只抄 4 位小數），'
                         '已用 SVD 換成最接近的旋轉矩陣再反解（各元素最多改動 %.1e）。'
                         % (orth, float(np.max(np.abs(T[:3, :3] - T_in[:3, :3])))))
        self._run_ik(T, self.q_rad(), notes)

    def _run_ik(self, T, q_ref, notes):
        self.ik_result = self.robot.ik(T, q_ref=q_ref)
        res = self.ik_result
        msgs = notes + list(res.messages)
        if res.reachable:
            worst = max(s.max_error for s in res.solutions)
            msgs.insert(0, '共 %d 組解，全部代回 FK 驗算，最大誤差 %.1e（位置單位 m）'
                        % (len(res.solutions), worst))
            if not any(s.singular for s in res.solutions) and \
                    min(abs(math.sin(s.q[4])) for s in res.solutions) < 1e-3:
                msgs.append('θ5 非常接近 0（接近手腕奇異）：θ4、θ6 對輸入的微小誤差非常敏感，'
                            '只有 θ4 + θ6 比較可靠。')
        self.ik_msg_lab.setText('\n'.join(msgs))
        self.ik_msg_lab.setStyleSheet('color: #b00000;' if any('奇異' in m or '無解' in m for m in msgs)
                                      else 'color: #1e6e2e;')
        self.refresh_ik_table()

    def refresh_ik_table(self):
        if self.ik_result is None:
            return
        scale = 1000.0 if self.unit_mm else 1.0
        unit = 'mm' if self.unit_mm else 'm'
        fmt = '%.2f' if self.unit_mm else '%.5f'
        self.ik_target_lab.setText('目標 p = [' + ', '.join(
            fmt % (x * scale) for x in self.ik_result.target[:3, 3]) + '] ' + unit)
        lim_rad = [(math.radians(lo), math.radians(hi)) for lo, hi in self.limits]
        q_now = self.q_rad()
        sols = self.ik_result.solutions
        for r in range(8):
            items = [self.ik_table.item(r, c) for c in range(9)]
            if r >= len(sols):
                for it in items:
                    it.setText('')
                    it.setBackground(QtGui.QColor('white'))
                    it.setToolTip('')
                continue
            sol = sols[r]
            fitted = fit_to_limits(sol.q, lim_rad)
            shown = fitted if fitted is not None else sol.q
            current = same_configuration(sol.q, q_now, 1e-6)
            texts = (['★%d' % (r + 1) if current else str(r + 1)]
                     + ['%.2f' % math.degrees(v) for v in shown]
                     + ['%s %s%s' % ('+' if sol.sign1 > 0 else '−', '+' if sol.sign3 > 0 else '−',
                                     ' F' if sol.flip else '  '),
                        '%.0e' % sol.max_error])
            bg = QtGui.QColor('#c8f0c8') if current else QtGui.QColor('white')
            fg = QtGui.QColor('#999999') if fitted is None else TEXT
            tip = ('超出目前的關節範圍，無法套用' if fitted is None else '點選以套用這組解')
            if sol.singular:
                tip += '（θ5 = 0 手腕奇異：θ4 取目前值）'
            for it, t in zip(items, texts):
                it.setText(t)
                it.setBackground(bg)
                it.setForeground(fg)
                it.setToolTip(tip)

    def _on_ik_row(self, row, _col):
        if self.ik_result is None or row >= len(self.ik_result.solutions):
            return
        sol = self.ik_result.solutions[row]
        lim_rad = [(math.radians(lo), math.radians(hi)) for lo, hi in self.limits]
        fitted = fit_to_limits(sol.q, lim_rad)
        if fitted is None:
            self.ik_msg_lab.setText('第 %d 組解超出目前的關節範圍（joint_limits:=%s），無法套用。'
                                    % (row + 1, self.limits_name))
            self.ik_msg_lab.setStyleSheet('color: #b00000;')
            return
        self.set_q([math.degrees(v) for v in fitted])

    def _build_matrix(self):
        box = QtWidgets.QGroupBox('齊次轉換矩陣')
        v = QtWidgets.QVBoxLayout(box)
        top = QtWidgets.QHBoxLayout()
        self.title_lab = QtWidgets.QLabel()
        tf = QtGui.QFont(MONO)
        tf.setPointSize(16)
        tf.setBold(True)
        self.title_lab.setFont(tf)
        top.addWidget(self.title_lab)
        top.addStretch(1)
        top.addWidget(QtWidgets.QLabel('顯示：'))
        self.view_combo = QtWidgets.QComboBox()
        for frm, to in TRANSFORMS:
            extra = ''
            if (frm, to) == (0, 6):
                extra = '（完整）'
            elif to - frm > 1:
                extra = '（中間乘積）'
            self.view_combo.addItem(tname(frm, to) + extra, (frm, to))
        self.view_combo.currentIndexChanged.connect(self._on_view)
        top.addWidget(self.view_combo)
        top.addWidget(QtWidgets.QLabel('　位置單位：'))
        self.unit_combo = QtWidgets.QComboBox()
        self.unit_combo.addItems(['mm', 'm'])
        self.unit_combo.currentIndexChanged.connect(self._on_unit)
        top.addWidget(self.unit_combo)
        v.addLayout(top)

        self.mat = MatrixTable()
        v.addWidget(self.mat)
        self.meaning_lab = QtWidgets.QLabel()
        self.meaning_lab.setWordWrap(True)
        self.meaning_lab.setFont(MONO)
        v.addWidget(self.meaning_lab)
        self.sing_lab = QtWidgets.QLabel('')
        self.sing_lab.setWordWrap(True)
        v.addWidget(self.sing_lab)
        return box

    def _build_tcp(self):
        box = QtWidgets.QGroupBox('工具中心點（TCP）')
        h = QtWidgets.QHBoxLayout(box)
        formula = QtWidgets.QLabel('P_TCP = p + d6·Ẑ6　')
        formula.setFont(MONO)
        h.addWidget(formula)
        h.addWidget(QtWidgets.QLabel('工具長度 d6：'))
        self.d6_spin = QtWidgets.QDoubleSpinBox()
        self.d6_spin.setRange(0.0, 1000.0)
        self.d6_spin.setDecimals(2)
        self.d6_spin.setSuffix(' mm')
        self.d6_spin.valueChanged.connect(lambda _: self.update_all(publish=False))
        h.addWidget(self.d6_spin)
        self.tcp_lab = QtWidgets.QLabel()
        self.tcp_lab.setFont(MONO)
        h.addWidget(self.tcp_lab, 1)
        return box

    def _build_verify(self):
        box = QtWidgets.QGroupBox('驗證區：tf2（link_0 → link_6）vs DH 公式')
        g = QtWidgets.QGridLayout(box)
        self.v_cmd_lab = QtWidgets.QLabel('等待 TF…')
        self.v_cmd_lab.setFont(MONO)
        self.v_cmd_status = QtWidgets.QLabel('—')
        self.v_model_lab = QtWidgets.QLabel('等待 TF…')
        self.v_model_lab.setFont(MONO)
        self.v_model_status = QtWidgets.QLabel('—')
        for w in (self.v_cmd_status, self.v_model_status):
            w.setAlignment(QtCore.Qt.AlignCenter)
            w.setMinimumWidth(90)
        g.addWidget(QtWidgets.QLabel('① TF vs DH(拉桿角度)：'), 0, 0)
        g.addWidget(self.v_cmd_lab, 0, 1)
        g.addWidget(self.v_cmd_status, 0, 2)
        g.addWidget(QtWidgets.QLabel('② TF vs DH(實際關節角)：'), 1, 0)
        g.addWidget(self.v_model_lab, 1, 1)
        g.addWidget(self.v_model_status, 1, 2)
        note = QtWidgets.QLabel(
            '① 就是「Gazebo 手臂的 {6}」與「GUI 用 DH 算出的 ⁰₆T」的比較；手臂移動途中會暫時變紅，到位後應恢復綠色。\n'
            '② 用 /joint_states 的實際角度代入 DH，與同一時間戳的 TF 比較，用來檢查 URDF 座標系本身是否等於 DH（移動中也應為綠色）。')
        note.setWordWrap(True)
        note.setStyleSheet('color: gray;')
        g.addWidget(note, 2, 0, 1, 3)
        g.setColumnStretch(1, 1)
        return box

    # ----------------------------------------------------------------
    # 事件
    def _on_slider(self, k, v):
        self.q_deg[k] = v / 10.0
        self.spins[k].blockSignals(True)
        self.spins[k].setValue(self.q_deg[k])
        self.spins[k].blockSignals(False)
        self.update_all(publish=True)

    def _on_spin(self, k, v):
        self.q_deg[k] = round(v, 1)
        self.sliders[k].blockSignals(True)
        self.sliders[k].setValue(int(round(v * 10)))
        self.sliders[k].blockSignals(False)
        self.update_all(publish=True)

    def _on_view(self, _):
        self.view = self.view_combo.currentData()
        self.update_all(publish=False)

    def _on_unit(self, idx):
        unit_mm = (idx == 0)
        if unit_mm != self.unit_mm:
            # 輸入表格的位置欄跟著換算（無法解析的格子保留原文字）
            k = 1000.0 if unit_mm else 0.001
            vals = []
            for r in range(3):
                try:
                    vals.append(self._ik_cell_value(r, 3) * k)
                except ValueError:
                    vals.append(None)
            self.unit_mm = unit_mm
            for r, val in enumerate(vals):
                if val is not None:
                    self._set_ik_cell(r, 3, val)
        self.unit_mm = unit_mm
        self._set_ik_input_headers()
        self.update_all(publish=False)

    def set_q(self, q_deg):
        for k, v in enumerate(q_deg):
            lo, hi = self.limits[k]
            v = min(max(float(v), lo), hi)
            self.q_deg[k] = v
            for w, val in ((self.sliders[k], int(round(v * 10))), (self.spins[k], v)):
                w.blockSignals(True)
                w.setValue(val)
                w.blockSignals(False)
        self.update_all(publish=True)

    def reset(self):
        self.view_combo.setCurrentIndex(0)
        self.unit_combo.setCurrentIndex(0)
        self.d6_spin.setValue(0.0)
        self.set_q([0] * 6)
        self.ik_result = None
        self.ik_target_lab.setText('尚未反解')
        self.ik_msg_lab.setText('')
        self.fill_ik_input_from_current()
        for r in range(8):
            for c in range(9):
                self.ik_table.item(r, c).setText('')
                self.ik_table.item(r, c).setBackground(QtGui.QColor('white'))

    # ----------------------------------------------------------------
    # 更新顯示
    def q_rad(self):
        return [math.radians(v) for v in self.q_deg]

    def update_all(self, publish):
        q = self.q_rad()
        if publish:
            self.node.publish(q)
        scale = 1000.0 if self.unit_mm else 1.0
        unit = 'mm' if self.unit_mm else 'm'
        fmt = '%.2f' if self.unit_mm else '%.5f'

        # --- 矩陣 ---
        frm, to = self.view
        T = self.robot.chain(q, frm, to)
        self.title_lab.setText('%s =' % tname(frm, to))
        self.mat.setHorizontalHeaderLabels(
            ['X̂%d' % to, 'Ŷ%d' % to, 'Ẑ%d' % to, '%sP%s (%s)' % (SUP[frm], SUB[to] + 'ORG', unit)])
        self.mat.set_matrix(T, scale)
        self.meaning_lab.setText(
            '第 1～3 欄（藍底）：{%d} 的 X̂、Ŷ、Ẑ 軸在 {%d} 中的方向（旋轉矩陣 %s%sR）\n'
            '第 4 欄（橘底）：{%d} 原點在 {%d} 中的位置%s，單位 %s'
            % (to, frm, SUP[frm], SUB[to], to, frm, '（腕心）' if to == 6 else '', unit))

        # --- 奇異點 ---
        if Puma560.is_wrist_singular(q):
            self.sing_lab.setText('⚠ θ5 ≈ 0：Ẑ4 與 Ẑ6 共線，只能決定 θ4+θ6（手腕奇異）')
            self.sing_lab.setStyleSheet('color: #b00000; font-weight: bold;')
        else:
            self.sing_lab.setText('')

        # --- TCP ---
        T06 = self.robot.fk(q)
        p_tcp = Puma560.tcp_position(T06, self.d6_spin.value() / 1000.0) * scale
        self.tcp_lab.setText('P_TCP = [' + ', '.join(fmt % v for v in p_tcp) + '] ' + unit)

        # --- DH 表 ---
        for r, row in enumerate(self.robot.params.rows):
            a = (row.a_sym + ' = ' if row.a_sym else '') + (fmt % (row.a * scale))
            d = (row.d_sym + ' = ' if row.d_sym else '') + (fmt % (row.d * scale))
            vals = [str(row.i), '%g' % math.degrees(row.alpha), a, d, '%.1f' % self.q_deg[r]]
            for c, txt in enumerate(vals):
                self.dh_table.item(r, c).setText(txt)
        if getattr(self, 'ik_table', None) is not None:
            self.refresh_ik_table()
        self.dh_table.setHorizontalHeaderLabels(
            ['i', 'αᵢ₋₁ (°)', 'aᵢ₋₁ (%s)' % unit, 'dᵢ (%s)' % unit, 'θᵢ (°)'])

    @staticmethod
    def _diff(T_a, T_b):
        pos_mm = float(np.max(np.abs(T_a[:3, 3] - T_b[:3, 3]))) * 1000.0
        rot = float(np.max(np.abs(T_a[:3, :3] - T_b[:3, :3])))
        return pos_mm, rot

    @staticmethod
    def _set_status(lab, ok):
        if ok:
            lab.setText('一致')
            lab.setStyleSheet('background:#1e8e3e; color:white; font-weight:bold; padding:4px;')
        else:
            lab.setText('不一致')
            lab.setStyleSheet('background:#c5221f; color:white; font-weight:bold; padding:4px;')

    def update_verify(self):
        # ① 最新 TF vs 拉桿角度的 DH
        try:
            T_tf = self.node.lookup_06()
            pos, rot = self._diff(T_tf, self.robot.fk(self.q_rad()))
            self.v_cmd_lab.setText('最大位置誤差 %.6f mm　旋轉誤差 %.2e' % (pos, rot))
            self._set_status(self.v_cmd_status, pos < POS_TOL_MM and rot < ROT_TOL)
        except Exception as e:
            self.v_cmd_lab.setText('TF 查不到：%s' % type(e).__name__)
            self.v_cmd_status.setText('—')
            self.v_cmd_status.setStyleSheet('')
        # ② 同一時間戳：TF vs 實際關節角的 DH
        q_js, T_sync = self.node.lookup_synced()
        if q_js is None:
            self.v_model_lab.setText('等待 /joint_states 與 TF…')
            self.v_model_status.setText('—')
            self.v_model_status.setStyleSheet('')
        else:
            pos, rot = self._diff(T_sync, self.robot.fk(q_js))
            self.v_model_lab.setText('最大位置誤差 %.6f mm　旋轉誤差 %.2e' % (pos, rot))
            self._set_status(self.v_model_status, pos < POS_TOL_MM and rot < ROT_TOL)


def main(args=None):
    rclpy.init(args=args)
    node = TeachingNode()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    global MONO
    MONO = make_mono_font()
    w = TeachingWindow(node)
    w.resize(1350, 860)
    w.show()
    # 讓 Ctrl+C 能結束 Qt 事件迴圈
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    ret = app.exec_()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
