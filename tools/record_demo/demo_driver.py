#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示範腳本：依時間軸自動操作教學 GUI，同時擷取 GUI 影格、字幕時間軸與 README 截圖。

輸出（<輸出目錄> 下）：
  gui/<時間戳記>.jpg   GUI 影格（約 15 fps）
  shots/*.png          README 用的 GUI 截圖
  timeline.json        字幕與時間點；以 "__cam_shot__<名稱>" 開頭的項目表示
                       compose.py 要在該時間點從相機影格擷取一張 Gazebo 截圖

修改示範內容：編輯下方 run_demo() 即可。
需要先 source install/setup.bash，並且 Gazebo 與控制器已在執行（gui:=false）。

用法：demo_driver.py <輸出目錄>
"""
import json
import os
import sys
import time

import rclpy
from PyQt5 import QtWidgets

from puma560_teaching_gui import gui_node as g

FPS = 15.0


class Demo:
    def __init__(self, out):
        self.out = out
        os.makedirs(os.path.join(out, 'gui'), exist_ok=True)
        os.makedirs(os.path.join(out, 'shots'), exist_ok=True)
        rclpy.init(args=['--ros-args', '-r', '__node:=demo_driver', '-p', 'use_sim_time:=true'])
        self.node = g.TeachingNode()
        self.app = QtWidgets.QApplication([])
        self.app.setStyle('Fusion')
        g.MONO = g.make_mono_font()
        self.w = g.TeachingWindow(self.node)
        self.w.resize(1350, 860)
        self.w.show()
        self.captions = []
        self.next_grab = 0.0

    # ---------------- 基本動作 ----------------
    def tick(self, sec):
        """處理事件 sec 秒，期間以 FPS 擷取 GUI 影格。"""
        t_end = time.time() + sec
        while True:
            now = time.time()
            if now >= self.next_grab:
                self.w.grab().save(os.path.join(self.out, 'gui', '%.4f.jpg' % now), 'JPG', 92)
                self.next_grab = now + 1.0 / FPS
            if now >= t_end:
                break
            self.app.processEvents()
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def caption(self, text):
        self.captions.append((time.time(), text))

    def cam_shot(self, name, dt=0.0):
        """請 compose.py 在此時間點擷取一張 Gazebo 截圖 docs/images/<name>.png。"""
        self.captions.append((time.time() + dt, '__cam_shot__' + name))

    def gui_shot(self, name):
        self.w.grab().save(os.path.join(self.out, 'shots', name + '.png'))

    def move_to(self, q_target, sec):
        """拉桿以 smoothstep 平滑移動（看起來像在拖曳）。"""
        q0 = list(self.w.q_deg)
        steps = max(1, int(sec * 20))
        for k in range(1, steps + 1):
            s = k / steps
            s = s * s * (3 - 2 * s)
            self.w.set_q([a + (b - a) * s for a, b in zip(q0, q_target)])
            self.tick(sec / steps)

    def drag_joint(self, i, value, sec):
        q = list(self.w.q_deg)
        q[i] = value
        self.move_to(q, sec)

    # ---------------- 示範時間軸 ----------------
    def run_demo(self):
        w = self.w
        exercise = [60, -30, 30, 40, 45, -20]
        self.tick(1.0)
        self.t0 = time.time()

        self.caption('PUMA560 正向運動學教學模擬（Craig 修正式 DH，ROS 2 Humble + Gazebo Fortress）')
        self.tick(3.0)

        self.caption('拖曳拉桿 θ1～θ6 → Gazebo 手臂即時跟隨，⁰₆T 即時更新')
        for i, v in enumerate(exercise):
            self.drag_joint(i, v, 1.1)

        self.caption('練習題姿態 (60, −30, 30, 40, 45, −20)：⁰₆T 與講義答案一致；驗證區 TF = DH（綠色）')
        self.tick(3.5)
        self.gui_shot('gui_fk')
        self.cam_shot('gazebo_exercise')

        self.caption('切換顯示各連桿轉換與中間乘積：¹₃T、³₆T（講義的分組乘法）')
        for idx in (8, 9):            # TRANSFORMS 中 (1,3)、(3,6) 的位置
            w.view_combo.setCurrentIndex(idx)
            self.tick(2.2)
        w.view_combo.setCurrentIndex(0)
        self.tick(0.8)

        self.caption('奇異姿態 θ5 = 0：Ẑ4 與 Ẑ6 共線，GUI 顯示奇異點提示')
        self.move_to([30, -20, 40, 50, 0, -30], 3.0)
        self.tick(3.0)
        self.gui_shot('gui_singular')

        self.caption('工具中心點：輸入工具長度 d6，顯示 P_TCP = p + d6·Ẑ6')
        for k in range(1, 21):
            w.d6_spin.setValue(5.0 * k)
            self.tick(0.06)
        self.tick(2.0)
        w.d6_spin.setValue(0.0)

        self.caption('反向運動學：以目前 ⁰₆T 反解 → 列出全部 8 組解，每組都代回 FK 驗算')
        self.move_to(exercise, 2.5)
        w.tabs.setCurrentIndex(1)
        self.tick(0.8)
        w.solve_ik()
        self.tick(3.0)
        self.gui_shot('gui_ik')

        for row in (2, 4, 6, 1):
            self.caption('點選第 %d 組解 → 手臂換成另一種構型，但 ⁰₆T（末端位置與姿態）完全不變' % (row + 1))
            w.ik_table.selectRow(row)
            w._on_ik_row(row, 0)
            self.tick(3.6)
            if row == 6:
                self.cam_shot('gazebo_ik_solution7', dt=-0.3)   # 已到位、尚未切到下一組

        self.caption('歸零（Reset）')
        w.tabs.setCurrentIndex(0)
        self.move_to([0] * 6, 3.0)
        w.reset()
        self.tick(2.5)
        self.t1 = time.time()

    def save(self):
        with open(os.path.join(self.out, 'timeline.json'), 'w', encoding='utf-8') as f:
            json.dump({'t0': self.t0, 't1': self.t1, 'captions': self.captions},
                      f, ensure_ascii=False, indent=1)
        print('示範長度 %.1f 秒' % (self.t1 - self.t0))


def main():
    demo = Demo(sys.argv[1])
    demo.run_demo()
    demo.save()
    demo.node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
