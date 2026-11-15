#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PUMA560 運動學模組（純 Python + numpy，不依賴 ROS）

嚴格依照 Craig《Introduction to Robotics》的「修正式 DH（modified DH）」：

    ^{i-1}_i T = RotX(α_{i-1}) · TransX(a_{i-1}) · RotZ(θ_i) · TransZ(d_i)

    = [ cθ       -sθ       0     a_{i-1}  ]
      [ sθ·cα    cθ·cα    -sα   -sα·d_i   ]
      [ sθ·sα    cθ·sα     cα    cα·d_i   ]
      [ 0         0        0     1        ]

注意：這和 Spong 等書的「標準 DH」(RotZ·TransZ·TransX·RotX) 不同，
座標系 {i} 是固定在「連桿 i 的近端（靠近基座那端）」的關節軸 i 上。

內部長度單位一律使用公尺 (m)，角度使用弳度 (rad)；
顯示用的 mm 換算由 GUI 處理。
"""

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# ---------------------------------------------------------------------
# DH 參數讀取
# ---------------------------------------------------------------------
@dataclass
class DHRow:
    """DH 表的一列（對應 joint_i）。"""
    i: int            # 關節編號 1..6
    alpha: float      # α_{i-1}（rad）
    a: float          # a_{i-1}（m）
    d: float          # d_i（m）
    a_sym: str = ''   # 顯示用符號，例如 'a2'；數字則為空字串
    d_sym: str = ''   # 顯示用符號，例如 'd3'


@dataclass
class DHParams:
    rows: List[DHRow]
    symbols: Dict[str, float] = field(default_factory=dict)


def default_dh_path() -> str:
    """找出 dh_params.yaml 的位置。

    搜尋順序：
      1. 環境變數 PUMA560_DH_YAML
      2. 已安裝的 ROS 套件 puma560_description 的 share 目錄
      3. 原始碼目錄（src/puma560_description/config/），方便不 source ROS 也能跑 pytest
    """
    env = os.environ.get('PUMA560_DH_YAML')
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('puma560_description'),
                         'config', 'dh_params.yaml')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.normpath(os.path.join(
        here, '..', '..', 'puma560_description', 'config', 'dh_params.yaml'))
    return p


def load_dh_params(path: Optional[str] = None) -> DHParams:
    """讀取 dh_params.yaml，把符號（a2, d3…）代換成數值。"""
    if yaml is None:
        raise RuntimeError('需要 python3-yaml')
    path = path or default_dh_path()
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    symbols = {k: float(v) for k, v in (data.get('symbols') or {}).items()}

    def resolve(v):
        # 欄位可以是數字，也可以是 symbols 中的名稱
        if isinstance(v, str):
            return symbols[v], v
        return float(v), ''

    rows = []
    for r in data['dh_table']:
        a, a_sym = resolve(r['a'])
        d, d_sym = resolve(r['d'])
        rows.append(DHRow(i=int(r['i']), alpha=math.radians(float(r['alpha_deg'])),
                          a=a, d=d, a_sym=a_sym, d_sym=d_sym))
    rows.sort(key=lambda x: x.i)
    return DHParams(rows=rows, symbols=symbols)


# ---------------------------------------------------------------------
# 基本轉換
# ---------------------------------------------------------------------
def craig_link_transform(alpha: float, a: float, theta: float, d: float) -> np.ndarray:
    """Craig 修正式 DH 的單一連桿轉換 ^{i-1}_i T。

    參數：alpha = α_{i-1}、a = a_{i-1}、theta = θ_i、d = d_i
    """
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct,      -st,       0.0,  a],
        [st * ca,  ct * ca, -sa,  -sa * d],
        [st * sa,  ct * sa,  ca,   ca * d],
        [0.0,      0.0,      0.0,  1.0],
    ])


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


# ---------------------------------------------------------------------
# PUMA560 正向運動學
# ---------------------------------------------------------------------
class Puma560:
    """PUMA560 運動學。q 一律為 6 個關節角（rad）。"""

    def __init__(self, params: Optional[DHParams] = None):
        self.params = params or load_dh_params()
        if len(self.params.rows) != 6:
            raise ValueError('PUMA560 需要 6 列 DH 參數')
        s = self.params.symbols
        # IK 會用到的尺寸（m）
        self.a2 = s.get('a2', self.params.rows[2].a)
        self.a3 = s.get('a3', self.params.rows[3].a)
        self.d3 = s.get('d3', self.params.rows[2].d)
        self.d4 = s.get('d4', self.params.rows[3].d)

    # 單一連桿 ^{i-1}_i T，i = 1..6
    def link_transform(self, i: int, theta: float) -> np.ndarray:
        r = self.params.rows[i - 1]
        return craig_link_transform(r.alpha, r.a, theta, r.d)

    def link_transforms(self, q: Sequence[float]) -> List[np.ndarray]:
        """回傳 [⁰₁T, ¹₂T, ..., ⁵₆T]。"""
        return [self.link_transform(i + 1, q[i]) for i in range(6)]

    def chain(self, q: Sequence[float], frm: int, to: int) -> np.ndarray:
        """^{frm}_{to} T = ^{frm}_{frm+1}T · ... · ^{to-1}_{to}T（frm < to）。

        例如 chain(q, 1, 3) = ¹₃T、chain(q, 3, 6) = ³₆T。
        """
        T = np.eye(4)
        for i in range(frm + 1, to + 1):
            T = T @ self.link_transform(i, q[i - 1])
        return T

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """⁰₆T（位置單位 m）。{6} 原點位於腕心。"""
        return self.chain(q, 0, 6)

    @staticmethod
    def tcp_position(T06: np.ndarray, d6: float) -> np.ndarray:
        """工具中心點：P_TCP = p + d6 · Ẑ6（d6 與 p 單位需一致）。"""
        return T06[:3, 3] + d6 * T06[:3, 2]

    @staticmethod
    def is_wrist_singular(q: Sequence[float], tol: float = 0.01) -> bool:
        """θ5 ≈ 0（或 180°）時 Ẑ4 與 Ẑ6 共線 → 手腕奇異。"""
        return abs(math.sin(q[4])) < tol

    # -----------------------------------------------------------------
    # 反向運動學（PUMA560 閉合解，講義公式「已修正」版本）
    # -----------------------------------------------------------------
    def ik(self, T: np.ndarray, q_ref: Optional[Sequence[float]] = None,
           sing_tol: float = 1e-9) -> 'IKResult':
        """由 ⁰₆T（位置單位 m）求出全部 8 組解。

        分支：θ1 取 ±√（2 種）× θ3 取 ±√（2 種）× 手腕是否翻轉（2 種）= 8 組
        q_ref：手腕奇異（θ5 = 0）時 θ4 無法決定，改用 q_ref[3]（通常是目前的 θ4）
        每組解都會代回 FK 驗算，誤差存在 fk_error。
        """
        a2, a3, d3, d4 = self.a2, self.a3, self.d3, self.d4
        r = T[:3, :3]
        px, py, pz = T[:3, 3]
        r11, r12, r13 = r[0]
        r21, r22, r23 = r[1]
        r31, r32, r33 = r[2]
        res = IKResult(target=np.array(T, dtype=float))

        # ---- θ1 = atan2(py, px) − atan2(d3, ±√(px² + py² − d3²)) ----
        rho2 = px * px + py * py - d3 * d3
        if rho2 < -1e-12:
            res.messages.append('無解：px² + py² < d3²（腕心離 Z0 軸太近，在肩部偏移 d3 的圓柱內）')
            return res
        rho = math.sqrt(max(rho2, 0.0))

        # ---- K = (px² + py² + pz² − a2² − a3² − d3² − d4²) / (2·a2) ----
        K = (px * px + py * py + pz * pz - a2 * a2 - a3 * a3 - d3 * d3 - d4 * d4) / (2.0 * a2)
        k2 = a3 * a3 + d4 * d4 - K * K
        if k2 < -1e-12:
            res.messages.append('無解：a3² + d4² < K²（腕心超出手臂可達範圍）')
            return res
        sk = math.sqrt(max(k2, 0.0))

        wrist_singular_any = False
        for sign1 in (+1, -1):
            th1 = math.atan2(py, px) - math.atan2(d3, sign1 * rho)
            c1, s1 = math.cos(th1), math.sin(th1)
            for sign3 in (+1, -1):
                # ---- θ3 = atan2(a3, d4) − atan2(K, ±√(a3² + d4² − K²)) ----
                th3 = math.atan2(a3, d4) - math.atan2(K, sign3 * sk)
                c3, s3 = math.cos(th3), math.sin(th3)

                # ---- θ23（令 u = c1·px + s1·py），θ2 = θ23 − θ3 ----
                # 注意第二個參數中間是「+」(a3 + a2·c3)·u（講義原文印成「−」是錯的）
                u = c1 * px + s1 * py
                th23 = math.atan2((-a3 - a2 * c3) * pz - u * (d4 - a2 * s3),
                                  (a2 * s3 - d4) * pz + (a3 + a2 * c3) * u)
                th2 = th23 - th3
                c23, s23 = math.cos(th23), math.sin(th23)

                # ---- θ4 = atan2(−r13·s1 + r23·c1, −r13·c1·c23 − r23·s1·c23 + r33·s23) ----
                y4 = -r13 * s1 + r23 * c1
                x4 = -r13 * c1 * c23 - r23 * s1 * c23 + r33 * s23
                singular = math.hypot(x4, y4) < sing_tol
                if singular:
                    # θ5 = 0：Ẑ4 與 Ẑ6 共線，只能決定 θ4 + θ6 → θ4 取參考值（目前角度）
                    wrist_singular_any = True
                    th4 = q_ref[3] if q_ref is not None else 0.0
                else:
                    th4 = math.atan2(y4, x4)
                c4, s4 = math.cos(th4), math.sin(th4)

                # ---- θ5：由 s5、c5 求出 ----
                #   −s5 = r13(c1c23c4 + s1s4) + r23(s1c23c4 − c1s4) − r33(s23c4)
                #    c5 = −r13·c1·s23 − r23·s1·s23 − r33·c23
                s5 = -(r13 * (c1 * c23 * c4 + s1 * s4)
                       + r23 * (s1 * c23 * c4 - c1 * s4)
                       - r33 * (s23 * c4))
                c5 = -r13 * c1 * s23 - r23 * s1 * s23 - r33 * c23
                th5 = math.atan2(s5, c5)

                # ---- θ6：由 s6、c6 求出 ----
                #   s6 = −r11(c1c23s4 − s1c4) − r21(s1c23s4 + c1c4) + r31(s23s4)
                #   c6 = r11[(c1c23c4 + s1s4)c5 − c1s23s5] + r21[(s1c23c4 − c1s4)c5 − s1s23s5]
                #        − r31(s23c4c5 + c23s5)
                s6 = (-r11 * (c1 * c23 * s4 - s1 * c4)
                      - r21 * (s1 * c23 * s4 + c1 * c4)
                      + r31 * (s23 * s4))
                c6 = (r11 * ((c1 * c23 * c4 + s1 * s4) * c5 - c1 * s23 * s5)
                      + r21 * ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5)
                      - r31 * (s23 * c4 * c5 + c23 * s5))
                th6 = math.atan2(s6, c6)

                q = [th1, th2, th3, th4, th5, th6]
                # ---- 手腕翻轉：(θ4 + 180°, −θ5, θ6 + 180°) ----
                q_flip = [th1, th2, th3, th4 + math.pi, -th5, th6 + math.pi]
                for flip, qq in ((False, q), (True, q_flip)):
                    qq = [wrap_angle(v) for v in qq]
                    err_p, err_r = fk_error(self.fk(qq), T)
                    res.solutions.append(IKSolution(
                        q=qq, sign1=sign1, sign3=sign3, flip=flip,
                        singular=singular, pos_error=err_p, rot_error=err_r))

        if wrist_singular_any:
            res.messages.append('θ5 = 0（手腕奇異）：θ4 無法單獨決定，已取目前的 θ4，'
                                'θ6 則依此算出（只有 θ4 + θ6 是確定的）')
        if rho2 < 1e-10:
            res.messages.append('px² + py² ≈ d3²：θ1 的兩組 ±√ 重合')
        if k2 < 1e-10:
            res.messages.append('a3² + d4² ≈ K²：θ3 的兩組 ±√ 重合（手臂伸直，位於工作空間邊界）')
        return res


@dataclass
class IKSolution:
    q: List[float]          # 6 個關節角（rad），已正規化到 (−π, π]
    sign1: int              # θ1 公式中 √ 取 +1 或 −1
    sign3: int              # θ3 公式中 √ 取 +1 或 −1
    flip: bool              # 是否為手腕翻轉解 (θ4+180°, −θ5, θ6+180°)
    singular: bool          # 是否在手腕奇異（θ5 = 0）
    pos_error: float        # 代回 FK 的最大位置誤差（m）
    rot_error: float        # 代回 FK 的最大旋轉矩陣元素誤差

    @property
    def max_error(self) -> float:
        return max(self.pos_error, self.rot_error)


@dataclass
class IKResult:
    target: np.ndarray
    solutions: List[IKSolution] = field(default_factory=list)
    messages: List[str] = field(default_factory=list)

    @property
    def reachable(self) -> bool:
        return len(self.solutions) > 0


def wrap_angle(a: float) -> float:
    """正規化到 (−π, π]。"""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def fk_error(T_a: np.ndarray, T_b: np.ndarray):
    """回傳 (最大位置誤差, 最大旋轉矩陣元素誤差)。"""
    return (float(np.max(np.abs(T_a[:3, 3] - T_b[:3, 3]))),
            float(np.max(np.abs(T_a[:3, :3] - T_b[:3, :3]))))


def rotation_deviation(R: np.ndarray):
    """回傳 (max|RᵀR − I|, det R)，用來判斷手打的 R 是不是旋轉矩陣。"""
    R = np.asarray(R, dtype=float)
    return float(np.max(np.abs(R.T @ R - np.eye(3)))), float(np.linalg.det(R))


def nearest_rotation(R: np.ndarray) -> np.ndarray:
    """用 SVD 求最接近 R 的旋轉矩陣（R = UΣVᵀ → UVᵀ，並保證 det = +1）。

    從講義或計算機抄來的 ⁰₆T 通常只有 4 位小數，RᵀR 不會剛好等於 I，
    先投影回旋轉矩陣再反解，θ4～θ6 才不會被捨入誤差帶偏。
    """
    U, _, Vt = np.linalg.svd(np.asarray(R, dtype=float))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


def same_configuration(q_a: Sequence[float], q_b: Sequence[float], tol: float = 1e-6) -> bool:
    """兩組關節角是否相同（以 2π 為週期比較）。"""
    return all(abs(wrap_angle(a - b)) < tol for a, b in zip(q_a, q_b))


def fit_to_limits(q: Sequence[float], limits_rad) -> Optional[List[float]]:
    """把每個關節角加減 2π，試著放進關節範圍；放不進去回傳 None。

    例如 real 範圍的 θ2 是 −225°～45°，IK 給出 170° 時可改用 −190°。
    """
    out = []
    for v, (lo, hi) in zip(q, limits_rad):
        cands = [v + k * 2.0 * math.pi for k in (0, -1, 1, -2, 2)]
        ok = [c for c in cands if lo - 1e-9 <= c <= hi + 1e-9]
        if not ok:
            return None
        out.append(ok[0])
    return out


def deg2rad_list(q_deg):
    return [math.radians(v) for v in q_deg]
