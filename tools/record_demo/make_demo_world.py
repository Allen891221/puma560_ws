#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""以 teaching.sdf 為基礎，產生錄影用的 world：加上 Sensors 系統與一台固定視角相機。

相機影像發布在 Gazebo topic `demo_cam`（1280×960，15 Hz），
由 ros_gz_bridge 轉成 ROS 2 的 sensor_msgs/Image 後給 cam_recorder.py 錄製。

用法：make_demo_world.py <teaching.sdf> <輸出.sdf>
"""
import math
import sys

# 相機位置與注視點（world 座標，m）；手臂底座在原點，{0} 在 z = 1.0
CAM_POS = (1.55, -1.25, 1.42)
CAM_TARGET = (0.12, 0.10, 0.78)
HFOV = 1.10           # 水平視角（rad）
WIDTH, HEIGHT = 1280, 960
RATE = 15             # Hz


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    src = open(sys.argv[1], encoding='utf-8').read()
    dx, dy, dz = (t - c for t, c in zip(CAM_TARGET, CAM_POS))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))   # Gazebo 相機看向 +X；pitch > 0 為往下看
    cam = f'''
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- 錄影用相機（固定視角），由 tools/record_demo/make_demo_world.py 產生 -->
    <model name="demo_camera">
      <static>true</static>
      <pose>{CAM_POS[0]} {CAM_POS[1]} {CAM_POS[2]} 0 {pitch:.5f} {yaw:.5f}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <topic>demo_cam</topic>
          <update_rate>{RATE}</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>{HFOV}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
  </world>'''
    if src.count('</world>') != 1:
        raise SystemExit('找不到唯一的 </world>：%s' % sys.argv[1])
    i = src.index('</world>')
    out = src[:i].rstrip() + '\n' + cam + src[i + len('</world>'):]
    open(sys.argv[2], 'w', encoding='utf-8').write(out)


if __name__ == '__main__':
    main()
