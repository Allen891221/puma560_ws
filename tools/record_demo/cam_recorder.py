#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""訂閱 /demo_cam（sensor_msgs/Image），把每張影像以 wall-clock 時間戳記存成 JPEG。

檔名就是時間戳記（例如 1794634464.6200.jpg），compose.py 依此和 GUI 影格對齊。
收到 SIGINT / SIGTERM 時，會把佇列中尚未寫完的影格寫完再結束。

用法：cam_recorder.py <輸出目錄>
"""
import os
import queue
import signal
import sys
import threading
import time

import rclpy
from PyQt5 import QtGui
from sensor_msgs.msg import Image


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    q = queue.Queue()

    def writer():
        # 存檔放在另一條執行緒，避免 JPEG 編碼拖慢訂閱
        while True:
            t, w, h, data = q.get()
            if t is None:
                break
            img = QtGui.QImage(data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
            img.save(os.path.join(out, '%.4f.jpg' % t), 'JPG', 92)

    th = threading.Thread(target=writer, daemon=True)
    th.start()

    rclpy.init()
    node = rclpy.create_node('demo_cam_recorder')
    node.create_subscription(
        Image, '/demo_cam',
        lambda m: q.put((time.time(), m.width, m.height, bytes(m.data))), 10)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    while not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)

    q.put((None, 0, 0, None))
    th.join(timeout=30)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
