#!/usr/bin/env python3
"""
Quattro parallel robot simulation (3-DoF XYZ + optional yaw via Quattro_4).

Wraps the existing quattro_bot examples and provides:
  - 3-DoF Quattro (XYZ translation)
  - 4-DoF Quattro_4 (XYZ + yaw)

For RViz visualization use the existing quattro_bot launch files:
    ros2 launch quattro_bot quattro_rviz.launch.py       # 3-DoF
    ros2 launch quattro_bot quattro_4_rviz.launch.py     # 4-DoF

Run this script alongside the above launch:
    python3 simulate_quattro.py            # 3-DoF Quattro
    python3 simulate_quattro.py --four     # 4-DoF Quattro_4

Robot parameters (Quattro 3-DoF):
  [R1, alpha, b1, c1, d1, h, r1, m_platform]
  = [0.3, -PI/2, 0.4, 0.0, 1.02, 0.1, 0.0, 0.18*sqrt(2)]
"""

import argparse
import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi


# ─────────────── Quattro 3-DoF parameters ────────────────────────────────────
QUATTRO_PARA = np.array([[
    0.3, -PI / 2.0, 0.4, 0.0, 1.02, 0.1, 0.0, 0.18 * math.sqrt(2)
]]).T

# ─────────────── Quattro_4 4-DoF parameters ─────────────────────────────────
# Parameters matching quattro_4_macro.xacro defaults (see quattro_bot)
QUATTRO4_PARA = np.array([[
    0.3, -PI / 2.0, 0.4, 0.0, 1.02, 0.1,
    0.0,  0.0,                           # v1x, v1y offsets
    0.12, 0.12,                          # v2x, v2y offsets
]]).T

BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T


def wait_done(rob: m.Robot, poll_s: float = 0.1) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def run_quattro(dof4: bool) -> int:
    robot_name = "quattro_4" if dof4 else "quattro"
    dof        = 4 if dof4 else 3
    para       = QUATTRO4_PARA if dof4 else QUATTRO_PARA

    print("=" * 60)
    print(f"Quattro {'4-DoF' if dof4 else '3-DoF'} simulation")
    print("=" * 60)

    pf  = m.Profile(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0)
    jpf = m.JntProfile(20.0, 50.0, 200.0)
    rob = m.Robot(robot_name, para, BASE_OFF, BASE_OFF, pf)

    home_jnt = np.zeros((dof, 1))
    rob.SetFeedback(home_jnt)

    ok, home_loc = rob.ForwardKin(home_jnt.flatten())
    if not ok:
        print("ERROR: FK at home failed.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    if dof4:
        print(f"      yaw={math.degrees(home_loc.A):.2f}°")
    cfg, turns = home_loc.G, home_loc.T

    for i in range(dof):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(80, 80, 80))

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)
    Z  = home_loc.z

    def loc(x, y, z, yaw=0.0):
        return m.LocData(x, y, z, yaw if dof4 else 0, 0, 0, cfg, turns)

    try:
        # ── demo 1: XY square path ────────────────────────────────────────────
        print("\n--- Demo 1: Cartesian square path ---")
        R = 0.30
        square = [
            loc( R,  R, Z),
            loc(-R,  R, Z),
            loc(-R, -R, Z),
            loc( R, -R, Z),
            loc( 0,  0, Z),
        ]
        for i, wp in enumerate(square):
            rob.MoveLine(wp, fd, 10)
            print(f"  Step {i}: ({wp.x:.2f}, {wp.y:.2f})")
        wait_done(rob)

        # ── demo 2: Z-stroke ─────────────────────────────────────────────────
        print("\n--- Demo 2: Z stroke ---")
        rob.MoveLine(loc(0.0, 0.0, Z - 0.15), fd, 5)
        rob.MoveLine(loc(0.0, 0.0, Z + 0.10), fd, 5)
        rob.MoveLine(loc(0.0, 0.0, Z),        fd, 0)
        wait_done(rob)
        print("  Z stroke complete.")

        # ── demo 3 (4-DoF only): yaw rotation ────────────────────────────────
        if dof4:
            print("\n--- Demo 3: Yaw rotation (Quattro_4 only) ---")
            for yaw_deg in [30, -30, 60, -60, 0]:
                rob.MovePTP(loc(0, 0, Z, math.radians(yaw_deg)), fd, 5)
                wait_done(rob)
                print(f"  Yaw = {yaw_deg}°")

        # ── demo: IK round-trip ────────────────────────────────────────────────
        print("\n--- IK round-trip check ---")
        for jnt_vals in [
            np.array([0.1, -0.1, 0.1, -0.1] if dof4 else [0.1, -0.1, 0.1]),
            np.array([0.3,  0.3, 0.3,  0.3] if dof4 else [0.3,  0.3, 0.3]),
        ]:
            jnt = jnt_vals.reshape(-1, 1)
            ok_fk, loc_fk = rob.ForwardKin(jnt.flatten())
            if ok_fk:
                ok_ik, jnt_rec = rob.InverseKin(loc_fk, dof)
                err = np.linalg.norm(jnt_rec - jnt.flatten())
                print(f"  error = {err:.2e} rad ({'PASS' if err < 1e-4 else 'WARN'})")

        print(f"\nQuattro {'4-DoF' if dof4 else '3-DoF'} simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Quattro parallel robot simulation")
    parser.add_argument("--four", action="store_true",
                        help="Run 4-DoF Quattro_4 instead of 3-DoF Quattro")
    args = parser.parse_args()
    return run_quattro(dof4=args.four)


if __name__ == "__main__":
    raise SystemExit(main())
