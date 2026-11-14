#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把相機影格、GUI 影格與字幕依時間戳記對齊，合成影片影格（之後由 ffmpeg 編碼）。

版面（1920×720）：
  ┌──────────────────────── 字幕列 ────────────────────────┐
  │  Gazebo（相機 1280×960 縮放）  │  教學 GUI（1350×860 縮放）  │
  └───────────────────────────────────────────────────────┘
每個輸出影格取「該時間點之前最新的一張」相機影格與 GUI 影格。

另外依 timeline.json 中的 "__cam_shot__<名稱>"，從相機影格存出 Gazebo 截圖到 <run>/shots/。

用法：compose.py <run 目錄> <輸出影格目錄>
"""
import bisect
import glob
import json
import os
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

W, H, BAR = 1920, 720, 64     # 輸出大小與字幕列高度
FPS = 15.0
FONT = 'Noto Sans CJK TC'


def load_seq(run, d):
    files = sorted(glob.glob(os.path.join(run, d, '*.jpg')))
    if not files:
        raise SystemExit('沒有影格：%s' % os.path.join(run, d))
    return [float(os.path.basename(f)[:-4]) for f in files], files


def latest(ts, fs, t):
    return fs[max(0, bisect.bisect_right(ts, t) - 1)]


def main():
    run, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    app = QtWidgets.QApplication([])   # noqa: F841  QFont 需要 QApplication，且必須保留參考以免被回收

    cam_t, cam_f = load_seq(run, 'cam')
    gui_t, gui_f = load_seq(run, 'gui')
    with open(os.path.join(run, 'timeline.json'), encoding='utf-8') as f:
        tl = json.load(f)
    caps = [(t, c) for t, c in tl['captions'] if not c.startswith('__')]
    cap_times = [c[0] for c in caps]

    # Gazebo 截圖（README 用）
    os.makedirs(os.path.join(run, 'shots'), exist_ok=True)
    for t, c in tl['captions']:
        if c.startswith('__cam_shot__'):
            QtGui.QImage(latest(cam_t, cam_f, t)).save(
                os.path.join(run, 'shots', c[len('__cam_shot__'):] + '.png'))

    cam_h = H - BAR
    cam_w = int(1280 * cam_h / 960)
    gui_w = W - cam_w
    gui_h = int(860 * gui_w / 1350)
    cap_font = QtGui.QFont(FONT)
    cap_font.setPixelSize(27)
    cap_font.setBold(True)
    tag_font = QtGui.QFont(FONT)
    tag_font.setPixelSize(18)
    tag_font.setBold(True)
    smooth = QtCore.Qt.SmoothTransformation

    t, n = tl['t0'] - 0.3, 0
    while t <= tl['t1']:
        frame = QtGui.QImage(W, H, QtGui.QImage.Format_RGB32)
        frame.fill(QtGui.QColor('#eef1f5'))
        p = QtGui.QPainter(frame)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.TextAntialiasing)
        p.drawImage(0, BAR, QtGui.QImage(latest(cam_t, cam_f, t)).scaled(
            cam_w, cam_h, QtCore.Qt.IgnoreAspectRatio, smooth))
        p.drawImage(cam_w, BAR + (cam_h - gui_h) // 2, QtGui.QImage(latest(gui_t, gui_f, t)).scaled(
            gui_w, gui_h, QtCore.Qt.IgnoreAspectRatio, smooth))

        # 字幕列
        p.fillRect(0, 0, W, BAR, QtGui.QColor('#1f2937'))
        i = bisect.bisect_right(cap_times, t) - 1
        if i >= 0:
            p.setFont(cap_font)
            p.setPen(QtGui.QColor('white'))
            p.drawText(QtCore.QRect(24, 0, W - 48, BAR),
                       QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, caps[i][1])

        # 角落標籤（放在下方，避免擋到 GUI 拉桿）
        p.setFont(tag_font)
        for r, label in ((QtCore.QRect(12, H - 44, 190, 32), 'Gazebo Fortress'),
                         (QtCore.QRect(W - 292, H - 44, 280, 32), '教學 GUI（PyQt5 + rclpy）')):
            p.fillRect(r, QtGui.QColor(31, 41, 55, 190))
            p.setPen(QtGui.QColor('white'))
            p.drawText(r, QtCore.Qt.AlignCenter, label)
        p.end()

        frame.save(os.path.join(out, 'f_%05d.jpg' % n), 'JPG', 93)
        n += 1
        t += 1.0 / FPS
    print('合成 %d 張影格（%.1f 秒）' % (n, n / FPS))


if __name__ == '__main__':
    main()
