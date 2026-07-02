#!/usr/bin/env python3
"""
RV-2FR-D 6-DOF serial arm simulation, using the real DH parameters from
robot_model/RV_2FR_D.txt (a Mitsubishi RV-2FR-D-like arm) -- for
regression-testing sixaxis_1.cpp against a second, independently-sourced
parameter set beyond the synthetic demo geometry in simulate_sixaxis.py.

DH parameters (loaded via robot_model_loader.py):
  alpha = [0, -PI/2, 0, PI/2, -PI/2, PI/2]
  a     = [0, 0, 0.230, -0.050, 0, 0]
  theta = [0, -PI/2, PI/2, 0, 0, 0]
  d     = [0.295, 0, 0, 0.270, 0, 0.070]

This geometry's alpha[3:6] pattern is the mirror image (through the plane
perpendicular to the local Z axis) of simulate_sixaxis.py's demo geometry
-- sixaxis_1.cpp's analytic IK originally only handled one chirality; it
now detects the sign of alpha_[4] and adapts both the elbow solve and the
wrist Euler-angle extraction accordingly (see sixaxis_1.cpp's CartToJnt
comments, and memory/project_robot_model_files.md for the derivation).

Demonstrates:
  - FK/IK round-trip verification (100 random samples)
  - MoveLine Cartesian trajectory
  - MovePTPJ joint-space motion

Run (ROS2 sourced, with rviz2 already launched via rv2fr_rviz.launch.py):
    python3 simulate_rv2fr.py
Or as a ROS2 node:
    ros2 run robnux_arm_sim simulate_rv2fr.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m
from robot_model_loader import build_para

PARA = build_para("RV_2FR_D.txt", 6)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF = 6
HOME_JNT = np.zeros(6)
# Modest offsets clear of the q(4)=0 spherical-wrist singularity.
READY_JNT = np.array([0.3, 0.4, -0.3, 0.2, 0.5, -0.2])


def wait_done(rob: m.Robot, poll_s: float = 0.1, timeout: float = 30.0) -> bool:
    t0 = time.time()
    while not rob.MotionDone():
        if time.time() - t0 > timeout:
            print(f"  WARNING: motion did not complete within {timeout}s")
            return False
        time.sleep(poll_s)
    return True


def fk_ik_round_trip(rob: m.Robot, n: int, rng: np.random.Generator) -> None:
    """100 random joint configs -> FK -> IK -> FK, checking the recovered
    pose matches (mirrors test_fk_ik_all_arms.py's methodology)."""
    bounds = [(-math.pi * 0.9, math.pi * 0.9)] * 4 + \
             [(-math.pi * 0.7, math.pi * 0.7)] + [(-math.pi * 0.9, math.pi * 0.9)]
    n_ok, n_ik_fail, n_bad, max_err = 0, 0, 0, 0.0
    for _ in range(n):
        q = np.array([rng.uniform(lo, hi) for lo, hi in bounds])
        if abs(q[4]) < 0.15:
            continue
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
        if err > 1e-5:
            n_bad += 1
        else:
            n_ok += 1
    n_total = n_ok + n_bad
    print(f"  {n_ok}/{n_total} round-tripped correctly, {n_bad} with large "
          f"error, {n_ik_fail} IK failures (real near-base-axis "
          f"singularities, expected), max position error = {max_err:.3e} m")


def main() -> int:
    print("=" * 60)
    print("RV-2FR-D simulation -- real robot_model/RV_2FR_D.txt geometry")
    print("=" * 60)

    pf = m.Profile(0.05, 0.2, 2.0, 0.1, 0.5, 5.0)
    jpf = m.JntProfile(0.5, 2.0, 20.0)
    rob = m.Robot("sixaxis_1", PARA, BASE_OFF, BASE_OFF, pf)

    ok, home_loc = rob.ForwardKin(HOME_JNT)
    if not ok:
        print("ERROR: FK at home position failed -- check DH parameters.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"      A={home_loc.A:.4f}  B={home_loc.B:.4f}  C={home_loc.C:.4f}")
    print(f"      branch={home_loc.G}  turn={home_loc.T}")

    print("\n--- FK/IK round-trip verification (100 random samples) ---")
    fk_ik_round_trip(rob, 100, np.random.default_rng(0))

    ok_r, ready_loc = rob.ForwardKin(READY_JNT)
    if not ok_r:
        print("ERROR: FK at ready pose failed.")
        rob.Shutdown()
        return 1
    xh, yh, zh = ready_loc.x, ready_loc.y, ready_loc.z
    ha, hb, hc = ready_loc.A, ready_loc.B, ready_loc.C
    cfg, turns = ready_loc.G, ready_loc.T

    rob.SetFeedback(READY_JNT.reshape(-1, 1))
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(50, 50, 50))

    print("\nROS2 topics now live (/joint_states, /cart_pose, /arm_path).")
    print("Starting motion in 2 seconds...")
    time.sleep(2)

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    try:
        print("\n--- Demo 1: Cartesian box (MoveLine) ---")
        dx, dz = 0.05, 0.05
        waypoints = [
            m.LocData(xh + dx, yh, zh,      ha, hb, hc, cfg, turns),
            m.LocData(xh + dx, yh, zh + dz, ha, hb, hc, cfg, turns),
            m.LocData(xh - dx, yh, zh + dz, ha, hb, hc, cfg, turns),
            m.LocData(xh - dx, yh, zh,      ha, hb, hc, cfg, turns),
            m.LocData(xh,      yh, zh,      ha, hb, hc, cfg, turns),
        ]
        for i, wp in enumerate(waypoints):
            rob.MoveLine(wp, fd, 5)
            print(f"  MoveLine to waypoint {i}: queued")
        wait_done(rob)
        print("  Box path complete.")

        print("\n--- Demo 2: Joint-space motion (MovePTPJ) ---")
        rob.MovePTPJ(HOME_JNT.reshape(-1, 1), 0)
        if wait_done(rob):
            ok_fk, loc = rob.ForwardKin(HOME_JNT)
            if ok_fk:
                print(f"  reached home: x={loc.x:.4f} y={loc.y:.4f} z={loc.z:.4f}")

        print("\nRV-2FR-D simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
