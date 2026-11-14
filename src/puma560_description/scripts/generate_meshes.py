#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
產生 PUMA560 外觀用的 STL mesh（上臂、前臂）。

- 尺寸 a2、a3、d3、d4 從 dh_params.yaml 讀取，所以 mesh 永遠和 DH 參數一致。
- 每個 mesh 已經放在「該 link 的 DH 座標系」中，URDF 裡 visual 的 origin 直接用 0 即可。
- 形狀 = 一組點的 3D 凸包（convex hull）：例如上臂 = 肩端大圓盤與肘端小圓盤的凸包，
  就會得到 PUMA 那種「肩端大、肘端小、厚度也收窄」的錐形厚板。
- 由 CMake 在 colcon build 時自動執行（見 CMakeLists.txt），產物不放進 git。

用法：generate_meshes.py <dh_params.yaml> <輸出目錄>
"""
import os
import struct
import sys

import numpy as np
import yaml
from scipy.spatial import ConvexHull

N = 64  # 圓周取樣點數


def disc(center, radius, z0, z1, n=N):
    """在 xy 平面上、圓心 center、厚度 z0~z1 的圓盤（取上下兩圈點）。"""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = []
    for z in (z0, z1):
        pts.append(np.c_[center[0] + radius * np.cos(t),
                         center[1] + radius * np.sin(t),
                         np.full(n, z)])
    return np.vstack(pts)


def ring_y(center, radius, n=N):
    """垂直於 Y 軸的圓（在 xz 平面上），圓心 center=(x, y, z)。"""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[center[0] + radius * np.cos(t),
                 np.full(n, center[1]),
                 center[2] + radius * np.sin(t)]


def write_hull_stl(points, path, name):
    """對點集合取凸包，輸出 binary STL（法向量一律朝外）。"""
    hull = ConvexHull(points)
    c = points[hull.vertices].mean(axis=0)
    tris = []
    for simplex in hull.simplices:
        a, b, d = points[simplex]
        nrm = np.cross(b - a, d - a)
        if np.dot(nrm, (a + b + d) / 3.0 - c) < 0:   # 法向量朝內 → 交換頂點順序
            b, d = d, b
            nrm = -nrm
        nrm = nrm / (np.linalg.norm(nrm) + 1e-15)
        tris.append((nrm, a, b, d))
    with open(path, 'wb') as f:
        f.write(name.encode('ascii').ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for nrm, a, b, d in tris:
            f.write(struct.pack('<12fH', *nrm, *a, *b, *d, 0))
    return len(tris)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1], 'r', encoding='utf-8') as f:
        sym = yaml.safe_load(f)['symbols']
    a2, a3, d3, d4 = (float(sym[k]) for k in ('a2', 'a3', 'd3', 'd4'))
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)

    # ---------------- 上臂（link_2 座標系）----------------
    # X2 沿上臂，肘關節在 (a2, 0, d3)；上臂厚板放在肩部（z2≈0）與前臂（z2≈d3）之間
    upper = np.vstack([
        disc((0.0, 0.0), 0.125, 0.015, 0.125),   # 肩端：大圓盤
        disc((a2, 0.0), 0.085, 0.030, 0.112),    # 肘端：小圓盤、較薄
    ])
    n1 = write_hull_stl(upper, os.path.join(out, 'upper_arm.stl'), 'puma560 upper arm')

    # ---------------- 前臂（link_3 座標系）----------------
    # 前臂由肘部沿 +Y3 延伸到腕心 (a3, d4, 0)；Z3 = 0 的平面即前臂中心面
    # 肘端是扁平的圓盤（z3 偏向遠離上臂的一側），腕端收成圓形截面
    fore = np.vstack([
        disc((0.0, 0.0), 0.080, -0.018, 0.070),        # 肘端
        ring_y((a3, 0.16, 0.020), 0.050),              # 中段
        ring_y((a3, d4 - 0.070, 0.0), 0.043),          # 腕端（接 link_4 的手腕外殼）
    ])
    n2 = write_hull_stl(fore, os.path.join(out, 'forearm.stl'), 'puma560 forearm')
    print('generate_meshes: upper_arm.stl (%d 三角形), forearm.stl (%d 三角形) -> %s'
          % (n1, n2, out))


if __name__ == '__main__':
    main()
