#!/usr/bin/env python3
"""
6020 SCARA 4-DOF robot simulation, using the real DH parameters from
robot_model/6020_scara.txt (not the synthetic demo geometry in
simulate_scara.py) -- for regression-testing scara.cpp against a second,
independently-sourced parameter set.

DH parameters (loaded via robot_model_loader.py):
  alpha = [0, 0, 0, 0]
  a     = [0, 0.325, 0.275, 0]   upper arm 325 mm, forearm 275 mm (600 mm reach)
  theta = [0, 0, 0, 0]
  d     = [0, 0, 0, 0]           no base-height/tool offset in this data file
                                  (shoulder axis sits at world Z=0)

Demonstrates:
  - FK/IK round-trip verification (100 random samples)
  - MoveLine Cartesian trajectory
  - MovePTPJ joint-space motion

Run (ROS2 sourced, with rviz2 already launched via 6020_scara_rviz.launch.py):
    python3 simulate_6020_scara.py
Or as a ROS2 node:
    ros2 run robnux_arm_sim simulate_6020_scara.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m
from robot_model_loader import build_para

PARA = build_para("6020_scara.txt", 4)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF = 4
HOME_JNT = np.zeros((DOF, 1))


def wait_done(rob: m.Robot, poll_s: float = 0.1) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def print_fk(rob: m.Robot, label: str, jnt: np.ndarray) -> None:
    ok, cart = rob.GetCartFromJnt(jnt.flatten(), 7)
    if ok:
        print(f"  {label}: x={cart[0]:.4f} y={cart[1]:.4f} z={cart[2]:.4f}")
    else:
        print(f"  {label}: FK failed")


def fk_ik_round_trip(rob: m.Robot, n: int, rng: np.random.Generator) -> None:
    """100 random joint configs -> FK -> IK -> FK, checking the recovered
    pose matches (mirrors test_fk_ik_all_arms.py's methodology)."""
    bounds = [(-math.pi * 0.9, math.pi * 0.9), (-math.pi * 0.9, math.pi * 0.9),
              (-0.14, 0.04), (-math.pi * 0.9, math.pi * 0.9)]
    n_ok, n_ik_fail, max_err = 0, 0, 0.0
    for _ in range(n):
        q = np.array([rng.uniform(lo, hi) for lo, hi in bounds])
        ok_fk, loc = rob.ForwardKin(q)
        if not ok_fk:
            continue
        ok_ik, q_rec = rob.InverseKin(loc, DOF)
        if not ok_ik:
            n_ik_fail += 1
            continue
        ok_fk2, loc2 = rob.ForwardKin(q_rec)
        if not ok_fk2:
            n_ik_fail += 1
            continue
        err = math.dist((loc.x, loc.y, loc.z), (loc2.x, loc2.y, loc2.z))
        max_err = max(max_err, err)
        n_ok += 1
    print(f"  {n_ok}/{n} round-tripped, {n_ik_fail} IK failures, "
          f"max position error = {max_err:.3e} m "
          f"({'PASS' if max_err < 1e-6 and n_ik_fail == 0 else 'CHECK'})")


def main() -> int:
    print("=" * 60)
    print("6020 SCARA simulation -- real robot_model/6020_scara.txt geometry")
    print("=" * 60)

    pf = m.Profile(0.3, 1.0, 10.0, 0.3, 1.0, 10.0)
    jpf = m.JntProfile(1.0, 5.0, 50.0)
    rob = m.Robot("scara", PARA, BASE_OFF, BASE_OFF, pf)

    ok, home_loc = rob.ForwardKin(HOME_JNT.flatten())
    if not ok:
        print("ERROR: FK at home position failed -- check DH parameters.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"      branch={home_loc.G}  turn={home_loc.T}")
    cfg, turns = home_loc.G, home_loc.T

    print("\n--- FK/IK round-trip verification (100 random samples) ---")
    fk_ik_round_trip(rob, 100, np.random.default_rng(0))

    rob.SetFeedback(HOME_JNT)
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(70, 70, 70))

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)
    Z = home_loc.z
    R = 0.12

    try:
        print("\n--- Demo: Cartesian square path (MoveLine) ---")
        corners = [
            m.LocData(0.45 + R, R, Z, 0, 0, 0, cfg, turns),
            m.LocData(0.45 - R, R, Z, 0, 0, 0, cfg, turns),
            m.LocData(0.45 - R, -R, Z, 0, 0, 0, cfg, turns),
            m.LocData(0.45 + R, -R, Z, 0, 0, 0, cfg, turns),
        ]
        for i, corner in enumerate(corners):
            rob.MoveLine(corner, fd, 10)
            print(f"  MoveLine to corner {i}: queued")
        wait_done(rob)
        print_fk(rob, "after square", HOME_JNT)

        print("\n--- Demo: Joint-space sweep (MovePTPJ) ---")
        rob.SetSpeed(m.Percent(50, 50, 50))
        targets_deg = [
            [30, -30, -0.05, 45],
            [-30, 30, -0.08, -45],
            [0, 0, 0.00, 0],
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([[
                math.radians(tgt[0]), math.radians(tgt[1]),
                tgt[2], math.radians(tgt[3]),
            ]]).T
            rob.MovePTPJ(jnt_tgt, 0)
            wait_done(rob)
            print_fk(rob, f"  joint target {tgt}", jnt_tgt)

        print("\n6020 SCARA simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
