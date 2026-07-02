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

KNOWN BUG (found by this script, not yet fixed): sixaxis_1.cpp's analytic
CartToJnt round-trips correctly for the demo geometry's alpha sign pattern
(alpha[3:6] = [-PI/2, PI/2, -PI/2]) but NOT for this real robot's pattern
(alpha[3:6] = [PI/2, -PI/2, PI/2] -- every sign flipped). FK is unaffected
(any valid Craig-DH alpha works); only the closed-form IK breaks, with a
~0.54m position error on essentially every sample, across all 8 branch
codes. Isolated by swapping RV_2FR_D's alpha/a/theta/d arrays into the
default geometry one at a time: only swapping in `alpha` alone reproduces
the failure (verify_sixaxis_ik_math.py can reproduce this in ~10 lines).
This script's FK/IK round-trip section documents that failure quantitatively
rather than hiding it; the MovePTPJ demo below still works since it needs
no IK.

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
HOME_JNT = np.zeros(DOF)
# Modest offsets clear of the q(4)=0 spherical-wrist singularity, matching
# the guard used in arm_registry.py's rv2fr spec.
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
    """100 random joint configs -> FK -> IK -> FK. Reports the true pass
    rate rather than assuming success -- see the KNOWN BUG note above."""
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
        if err > 1e-6:
            n_bad += 1
        else:
            n_ok += 1
    n_total = n_ok + n_bad
    print(f"  {n_ok}/{n_total} round-tripped correctly, {n_bad} with large "
          f"error, {n_ik_fail} IK failures, max position error = {max_err:.3e} m")
    if n_bad:
        print("  ^ this is the known sixaxis_1.cpp alpha-sign bug documented "
              "in this script's docstring, not a new issue")


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

    rob.SetFeedback(HOME_JNT.reshape(-1, 1))
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(50, 50, 50))

    print("\nROS2 topics now live (/joint_states, /cart_pose, /arm_path).")
    print("Starting motion in 2 seconds...")
    time.sleep(2)

    try:
        print("\n--- Demo: Joint-space motion (MovePTPJ, no IK needed) ---")
        for tgt, label in [(READY_JNT, "ready"), (HOME_JNT, "home")]:
            rob.MovePTPJ(tgt.reshape(-1, 1), 0)
            if wait_done(rob):
                ok_fk, loc = rob.ForwardKin(tgt)
                if ok_fk:
                    print(f"  reached {label}: x={loc.x:.4f} y={loc.y:.4f} z={loc.z:.4f}")

        print("\nRV-2FR-D simulation complete (MoveLine/MoveArc skipped -- "
              "both need IK, which is not yet correct for this geometry; "
              "see the KNOWN BUG note in this script's docstring).")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
