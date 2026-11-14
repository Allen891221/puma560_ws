# -*- coding: utf-8 -*-
"""PUMA560 正向運動學測試（pytest，不需 ROS）。

執行：
    cd ~/puma560_ws/src/puma560_teaching_gui
    python3 -m pytest test/test_kinematics.py -v
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from puma560_teaching_gui.puma560_kinematics import (  # noqa: E402
    Puma560, craig_link_transform, deg2rad_list, load_dh_params,
    rot_x, rot_z, trans)

MM = 1000.0


@pytest.fixture(scope='module')
def robot():
    return Puma560(load_dh_params())


def fk_mm(robot, q_deg):
    T = robot.fk(deg2rad_list(q_deg)).copy()
    T[:3, 3] *= MM
    return T


# ---------- 參數檔本身 ----------
def test_dh_yaml_matches_textbook():
    """yaml 中的數值必須等於指示中的 PUMA560 參數。"""
    p = load_dh_params()
    expect = [(0, 0, 0), (-90, 0, 0), (0, 431.8, 149.09),
              (-90, 20.32, 433.07), (90, 0, 0), (-90, 0, 0)]
    for row, (al, a, d) in zip(p.rows, expect):
        assert math.degrees(row.alpha) == pytest.approx(al)
        assert row.a * MM == pytest.approx(a)
        assert row.d * MM == pytest.approx(d)


# ---------- 公式本身 ----------
def test_link_formula_equals_rotx_transx_rotz_transz():
    """展開矩陣 = RotX(α)·TransX(a)·RotZ(θ)·TransZ(d)。"""
    rng = np.random.default_rng(0)
    for _ in range(50):
        al, th = rng.uniform(-np.pi, np.pi, 2)
        a, d = rng.uniform(-1, 1, 2)
        ref = rot_x(al) @ trans(a, 0, 0) @ rot_z(th) @ trans(0, 0, d)
        np.testing.assert_allclose(craig_link_transform(al, a, th, d), ref, atol=1e-12)


def test_urdf_joint_decomposition():
    """URDF joint：origin xyz=[a, −sα·d, cα·d]、rpy=[α,0,0]，再繞 z 轉 θ，
    必須等於 Craig 的連桿轉換（這是 URDF 巨集的數學依據）。"""
    rng = np.random.default_rng(1)
    for _ in range(50):
        al, th = rng.uniform(-np.pi, np.pi, 2)
        a, d = rng.uniform(-1, 1, 2)
        urdf = trans(a, -math.sin(al) * d, math.cos(al) * d) @ rot_x(al) @ rot_z(th)
        np.testing.assert_allclose(urdf, craig_link_transform(al, a, th, d), atol=1e-12)


# ---------- 第 5 節指定測試 ----------
def test_zero_pose(robot):
    T = fk_mm(robot, [0, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(T[:3, 3], [452.12, 149.09, -433.07], atol=1e-3)
    np.testing.assert_allclose(T[:3, :3], np.diag([1, -1, -1]), atol=1e-9)


def test_exercise_pose(robot):
    T = fk_mm(robot, [60, -30, 30, 40, 45, -20])
    expect = np.array([
        [0.5074, 0.5487, -0.6645, 68.0192],
        [0.5487, -0.8003, -0.2418, 415.9926],
        [-0.6645, -0.2418, -0.7071, -217.1700],
        [0, 0, 0, 1]])
    # 題目給的旋轉部分只有 4 位小數，容許四捨五入誤差 0.5e-4
    np.testing.assert_allclose(T[:3, :3], expect[:3, :3], atol=0.5e-4 + 1e-9)
    np.testing.assert_allclose(T[:3, 3], expect[:3, 3], atol=1e-3)


def test_pose_90_0_m90(robot):
    T = fk_mm(robot, [90, 0, -90, 0, 0, 0])
    np.testing.assert_allclose(T[:3, 3], [-149.09, 864.87, 20.32], atol=1e-3)


def test_random_rotation_is_so3(robot):
    rng = np.random.default_rng(42)
    for _ in range(100):
        q = rng.uniform(-np.pi, np.pi, 6)
        R = robot.fk(q)[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)


# ---------- 輔助功能 ----------
def test_chain_products(robot):
    """⁰₆T = ⁰₁T · ¹₃T · ³₆T（講義的分組乘法）。"""
    q = deg2rad_list([10, -20, 30, -40, 50, -60])
    T = robot.chain(q, 0, 1) @ robot.chain(q, 1, 3) @ robot.chain(q, 3, 6)
    np.testing.assert_allclose(T, robot.fk(q), atol=1e-12)
    np.testing.assert_allclose(robot.chain(q, 0, 1) @ robot.chain(q, 1, 6),
                               robot.fk(q), atol=1e-12)


def test_wrist_center_independent_of_wrist(robot):
    """{6} 原點在腕心：改變 θ4~θ6 位置不變。"""
    p1 = robot.fk(deg2rad_list([20, -30, 40, 0, 0, 0]))[:3, 3]
    p2 = robot.fk(deg2rad_list([20, -30, 40, 70, -50, 120]))[:3, 3]
    np.testing.assert_allclose(p1, p2, atol=1e-12)


def test_tcp_and_singularity(robot):
    T = robot.fk([0] * 6)
    np.testing.assert_allclose(Puma560.tcp_position(T, 0.1), T[:3, 3] + [0, 0, -0.1])
    assert Puma560.is_wrist_singular(deg2rad_list([30, -20, 40, 50, 0, -30]))
    assert not Puma560.is_wrist_singular(deg2rad_list([60, -30, 30, 40, 45, -20]))
