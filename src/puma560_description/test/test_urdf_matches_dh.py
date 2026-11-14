# -*- coding: utf-8 -*-
"""驗證：xacro 展開後的 URDF，link_0 → link_6 的 joint 鏈 ≡ Craig DH 的 ⁰₆T。

此測試完全依照 URDF 規格自己計算（不使用 DH 公式）：
    T_joint = Trans(origin.xyz) · R_rpy(origin.rpy) · Rot(axis, θ)
    R_rpy = Rz(yaw) · Ry(pitch) · Rx(roll)
再與 puma560_kinematics.Puma560.fk() 比較，並逐一比對每個 link frame。

需要先 source ROS（使用 xacro 指令）：
    python3 -m pytest src/puma560_description/test -v -p no:anyio
"""
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'puma560_teaching_gui'))
from puma560_teaching_gui.puma560_kinematics import Puma560, load_dh_params  # noqa

XACRO = os.path.join(HERE, '..', 'urdf', 'puma560.urdf.xacro')


def axis_angle(axis, t):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def rpy_matrix(r, p, y):
    return (axis_angle([0, 0, 1], y) @ axis_angle([0, 1, 0], p)
            @ axis_angle([1, 0, 0], r))


def homog(R=np.eye(3), p=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


@pytest.fixture(scope='module', params=['default', 'real'])
def urdf_joints(request):
    try:
        xml = subprocess.check_output(
            ['xacro', XACRO, 'joint_limits:=' + request.param], text=True)
    except FileNotFoundError:
        pytest.skip('找不到 xacro 指令，請先 source /opt/ros/humble/setup.bash')
    root = ET.fromstring(xml)
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        ax = j.find('axis')
        axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [1, 0, 0]
        joints[j.get('name')] = dict(parent=j.find('parent').get('link'),
                                     child=j.find('child').get('link'),
                                     xyz=xyz, rpy=rpy, axis=axis, type=j.get('type'))
    return joints


def urdf_chain(joints, q):
    """依 URDF 規則計算每個 link_i 在 link_0 中的姿態。"""
    frames = {0: np.eye(4)}
    T = np.eye(4)
    for i in range(1, 7):
        j = joints['joint_%d' % i]
        assert j['parent'] == 'link_%d' % (i - 1) and j['child'] == 'link_%d' % i
        T = T @ homog(rpy_matrix(*j['rpy']), j['xyz']) @ homog(axis_angle(j['axis'], q[i - 1]))
        frames[i] = T
    return frames


def test_urdf_chain_equals_dh(urdf_joints):
    robot = Puma560(load_dh_params())
    rng = np.random.default_rng(7)
    qs = [np.zeros(6), np.radians([60, -30, 30, 40, 45, -20])] + \
        [rng.uniform(-np.pi, np.pi, 6) for _ in range(200)]
    for q in qs:
        frames = urdf_chain(urdf_joints, q)
        # 每個中間座標系 {i} 都要一致，不只最後的 {6}
        for i in range(1, 7):
            np.testing.assert_allclose(frames[i], robot.chain(q, 0, i), atol=1e-9)


def test_all_joints_rotate_about_child_z(urdf_joints):
    for i in range(1, 7):
        assert urdf_joints['joint_%d' % i]['axis'] == [0, 0, 1]
        assert urdf_joints['joint_%d' % i]['type'] == 'revolute'
