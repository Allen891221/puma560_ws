# -*- coding: utf-8 -*-
"""PUMA560 反向運動學測試（第 5 節第 5 項，以及奇異點、無解等邊界情況）。"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from puma560_teaching_gui.puma560_kinematics import (  # noqa: E402
    Puma560, deg2rad_list, fit_to_limits, load_dh_params, same_configuration, wrap_angle)


@pytest.fixture(scope='module')
def robot():
    return Puma560(load_dh_params())


def assert_all_solutions_valid(robot, T, res, tol=1e-6):
    assert len(res.solutions) == 8
    for s in res.solutions:
        assert s.pos_error < tol and s.rot_error < tol, (s.q, s.pos_error, s.rot_error)
        # 自己再算一次，不只相信 IKSolution 裡記錄的誤差
        np.testing.assert_allclose(robot.fk(s.q), T, atol=tol)


def test_random_poses_8_solutions(robot):
    """第 5 節第 5 項：任意隨機姿態 → 8 組解代回 FK 都和原 ⁰₆T 一致（< 1e-6）。"""
    rng = np.random.default_rng(2024)
    for _ in range(1000):
        q = rng.uniform(-np.pi, np.pi, 6)
        T = robot.fk(q)
        res = robot.ik(T)
        assert_all_solutions_valid(robot, T, res)
        # 原本的關節角必須是 8 組解之一
        assert any(same_configuration(s.q, q, 1e-6) for s in res.solutions)


def test_exercise_pose(robot):
    q = deg2rad_list([60, -30, 30, 40, 45, -20])
    T = robot.fk(q)
    res = robot.ik(T)
    assert_all_solutions_valid(robot, T, res)
    assert any(same_configuration(s.q, q) for s in res.solutions)


def test_solutions_are_distinct(robot):
    """一般姿態下 8 組解彼此不同。"""
    T = robot.fk(deg2rad_list([60, -30, 30, 40, 45, -20]))
    sols = robot.ik(T).solutions
    for i in range(8):
        for j in range(i + 1, 8):
            assert not same_configuration(sols[i].q, sols[j].q, 1e-6)


def test_wrist_flip_relation(robot):
    """手腕翻轉解 = (θ4+180°, −θ5, θ6+180°)，前三軸相同。"""
    sols = robot.ik(robot.fk(deg2rad_list([10, -40, 20, 30, 60, -70]))).solutions
    for a, b in zip(sols[0::2], sols[1::2]):
        assert not a.flip and b.flip
        assert np.allclose(a.q[:3], b.q[:3])
        assert abs(wrap_angle(b.q[3] - a.q[3] - math.pi)) < 1e-12
        assert abs(wrap_angle(b.q[4] + a.q[4])) < 1e-12
        assert abs(wrap_angle(b.q[5] - a.q[5] - math.pi)) < 1e-12


def test_wrist_singular_keeps_theta4(robot):
    """θ5 = 0：θ4 取參考值，θ4+θ6 仍正確，8 組解代回 FK 仍一致。"""
    q = deg2rad_list([30, -20, 40, 50, 0, -30])
    T = robot.fk(q)
    res = robot.ik(T, q_ref=q)
    assert_all_solutions_valid(robot, T, res)
    assert any(s.singular for s in res.solutions)
    assert any('奇異' in m for m in res.messages)
    # 用目前的 θ4 當參考 → 應該能找回原本的關節角
    assert any(same_configuration(s.q, q, 1e-9) for s in res.solutions)


def test_zero_pose(robot):
    """全零姿態同時也是 θ5 = 0 的奇異姿態。"""
    T = robot.fk([0] * 6)
    res = robot.ik(T, q_ref=[0] * 6)
    assert_all_solutions_valid(robot, T, res)
    assert any(same_configuration(s.q, [0] * 6, 1e-9) for s in res.solutions)


def test_unreachable(robot):
    T = np.eye(4)
    T[:3, 3] = [2.0, 0.0, 0.0]          # 遠超過手臂長度
    res = robot.ik(T)
    assert not res.reachable and any('可達' in m for m in res.messages)

    T[:3, 3] = [0.05, 0.05, 0.0]        # 落在 d3 圓柱內
    res = robot.ik(T)
    assert not res.reachable and any('d3' in m for m in res.messages)


def test_fit_to_limits():
    lim = [(math.radians(a), math.radians(b)) for a, b in
           [(-160, 160), (-225, 45), (-45, 225), (-110, 170), (-100, 100), (-266, 266)]]
    q = deg2rad_list([0, 170, -170, 0, 0, 0])
    out = fit_to_limits(q, lim)
    assert out is not None
    assert math.degrees(out[1]) == pytest.approx(-190)
    assert math.degrees(out[2]) == pytest.approx(190)
    assert fit_to_limits(deg2rad_list([170, 0, 0, 0, 0, 0]), lim) is None
