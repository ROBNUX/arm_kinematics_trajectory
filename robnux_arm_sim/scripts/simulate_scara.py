#!/usr/bin/env python3
"""
SCARA 4-DOF robot simulation (Mitsubishi convention).

Demonstrates:
  - FK/IK with Scara kinematics
  - MoveLine Cartesian trajectory
  - MovePTPJ joint-space motion
  - All motions visualised in RViz2 via /joint_states

DH parameters (Craig convention):
  alpha = [0, 0, 0, 0]
  a     = [0, 0.30, 0.20, 0]   upper arm = 300 mm, forearm = 200 mm
  theta = [0, 0, 0, 0]          (zero offsets at home)
  d     = [0.30, 0, 0, -0.05]   base height 300 mm, tool -50 mm

Home TCP position: x=0.50, y=0.0, z=0.25

Run (ROS2 sourced, with rviz2 already launched via scara_rviz.launch.py):
    python3 simulate_scara.py
Or as a ROS2 node:
    ros2 run robnux_arm_sim simulate_scara.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

# ─────────────────────────────────────── geometry ────────────────────────────
# kine_para layout: [alpha × 4, a × 4, theta × 4, d × 4]
ALPHA = [0.0, 0.0, 0.0, 0.0]
A     = [0.0, 0.30, 0.20, 0.0]
THETA = [0.0, 0.0, 0.0, 0.0]
D     = [0.30, 0.0, 0.0, -0.05]

PARA = np.array([ALPHA + A + THETA + D]).T        # column vector (16×1)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF  = 4
HOME_JNT = np.zeros((DOF, 1))

# ─────────────────────────────────────── helpers ─────────────────────────────
def wait_done(rob: m.Robot, poll_s: float = 0.1) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def print_fk(rob: m.Robot, label: str, jnt: np.ndarray) -> None:
    ok, cart = rob.GetCartFromJnt(jnt.flatten(), 7)
    if ok:
        print(f"  {label}: x={cart[0]:.4f} y={cart[1]:.4f} z={cart[2]:.4f}")
    else:
        print(f"  {label}: FK failed")


def main() -> int:
    print("=" * 60)
    print("SCARA simulation — 4-DOF Mitsubishi convention")
    print("=" * 60)

    # ── build robot ──────────────────────────────────────────────────────────
    pf  = m.Profile(0.3, 1.0, 10.0, 0.3, 1.0, 10.0)   # Cartesian profile
    jpf = m.JntProfile(1.0, 5.0, 50.0)                  # joint profile
    rob = m.Robot("scara", PARA, BASE_OFF, BASE_OFF, pf)

    # ── home FK ──────────────────────────────────────────────────────────────
    ok, home_loc = rob.ForwardKin(HOME_JNT.flatten())
    if not ok:
        print("ERROR: FK at home position failed — check DH parameters.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"      branch={home_loc.G}  turn={home_loc.T}")
    cfg, turns = home_loc.G, home_loc.T

    rob.SetFeedback(HOME_JNT)
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(70, 70, 70))

    fd  = m.FrameData(1, 1, m.IpoMode.WORLD)
    Z   = home_loc.z   # keep same Z throughout Cartesian moves
    R   = 0.15          # square half-size (150 mm)

    # ── demo 1: Cartesian square path ────────────────────────────────────────
    print("\n--- Demo 1: Cartesian square path ---")
    corners = [
        m.LocData( R,  R, Z, 0, 0, 0, cfg, turns),
        m.LocData(-R,  R, Z, 0, 0, 0, cfg, turns),
        m.LocData(-R, -R, Z, 0, 0, 0, cfg, turns),
        m.LocData( R, -R, Z, 0, 0, 0, cfg, turns),
    ]
    try:
        for i, corner in enumerate(corners):
            rob.MoveLine(corner, fd, 10)
            print(f"  MoveLine to corner {i}")

        wait_done(rob)
        print_fk(rob, "after square", HOME_JNT)

        # ── demo 2: joint-space motion ────────────────────────────────────────
        print("\n--- Demo 2: Joint-space sweep ---")
        rob.SetSpeed(m.Percent(50, 50, 50))
        targets_deg = [
            [ 30, -30, -0.05, 45],
            [-30,  30, -0.08, -45],
            [  0,   0,  0.00,   0],
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([[
                math.radians(tgt[0]),
                math.radians(tgt[1]),
                tgt[2],              # prismatic joint [m]
                math.radians(tgt[3]),
            ]]).T
            rob.MovePTPJ(jnt_tgt, 0)
            wait_done(rob)
            print_fk(rob, f"  joint target {tgt}", jnt_tgt)

        # ── demo 3: IK round-trip ─────────────────────────────────────────────
        print("\n--- Demo 3: IK round-trip verification ---")
        test_jnts = [
            np.array([[ math.radians(20), math.radians(-40), -0.03, math.radians(60)]]).T,
            np.array([[-math.radians(15), math.radians( 30), -0.07, math.radians(-20)]]).T,
        ]
        for jnt in test_jnts:
            ok_fk, loc = rob.ForwardKin(jnt.flatten())
            if ok_fk:
                ok_ik, jnt_rec = rob.InverseKin(loc, DOF)
                err = np.linalg.norm(jnt_rec - jnt.flatten())
                print(f"  FK→IK round-trip error: {err:.2e} rad "
                      f"({'PASS' if err < 1e-4 else 'WARN'})")

        print("\nSCARA simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
