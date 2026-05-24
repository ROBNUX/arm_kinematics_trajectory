#!/usr/bin/env python3
"""Example 06 — Cartesian motion: MoveLine, MovePTP, MoveArc.

Demonstrates a full motion sequence on a Quattro robot:
  - Set up robot, profiles, and speed.
  - Execute a square path with MoveLine and MovePTP.
  - Demonstrate MoveArc via an auxiliary via-point.
  - Show PauseMotion / ResumeMotion during execution.
  - Wait for completion and shut down cleanly.

Run from a sourced ROS 2 workspace (requires active DDS):

    python3 examples/06_cartesian_motion.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PARA = np.array([[0.3, -math.pi / 2.0, 0.4, 0.0, 1.02, 0.1, 0.0,
                  0.18 * math.sqrt(2)]]).T
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
DOF = 4


def wait_done(rob: m.Robot, poll_s: float = 0.5) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def main() -> int:
    print("=" * 60)
    print("Cartesian motion — MoveLine / MovePTP / MoveArc")
    print("=" * 60)

    # ---------------------------------------------------------------- setup
    pf = m.Profile(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0)
    jpf = m.JntProfile(20.0, 50.0, 200.0)
    rob = m.Robot("quattro", PARA, BASE_OFF, BASE_OFF, pf)

    home_jnt = np.zeros((DOF, 1))
    rob.SetFeedback(home_jnt)

    # Discover branch / turn flags from home position
    ok, cur_loc = rob.ForwardKin(home_jnt)
    if not ok:
        print("ForwardKin failed at home — aborting.")
        rob.Shutdown()
        return 1
    cfg, turns = cur_loc.G, cur_loc.T
    print(f"Home position: x={cur_loc.x:.4f} y={cur_loc.y:.4f} "
          f"z={cur_loc.z:.4f}  branch={cfg} turn={turns}")

    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()

    spd = m.Percent(80, 80, 80)
    rob.SetSpeed(spd)

    # ---------------------------------------------------------------- frames
    # base=1 (default), tool=1 (default), interpolation in world frame
    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    # ---------------------------------------------------------------- locations
    Z = -0.75
    loc_a = m.LocData( 0.4,  0.4, Z, 0.0, 0.0, 0.0, cfg, turns)
    loc_b = m.LocData(-0.4,  0.4, Z, 0.0, 0.0, 0.0, cfg, turns)
    loc_c = m.LocData(-0.4, -0.4, Z, 0.0, 0.0, 0.0, cfg, turns)
    loc_d = m.LocData( 0.4, -0.4, Z, 0.0, 0.0, 0.0, cfg, turns)

    # Arc via-point (midpoint of a circular arc) and end-point
    arc_via = m.LocData( 0.0,  0.5, Z, 0.0, 0.0, 0.0, cfg, turns)
    arc_end = m.LocData(-0.4,  0.4, Z, 0.0, 0.0, 0.0, cfg, turns)

    try:
        print("\n--- Square loop: 3 iterations with MoveLine / MovePTP ---")
        for i in range(3):
            print(f"  Iteration {i + 1}")
            rob.MoveLine(loc_a, fd, 10)   # 10 % approach blend
            rob.MoveLine(loc_b, fd, 10)
            rob.MovePTP( loc_c, fd, 10)
            rob.MovePTP( loc_d, fd, 0)    # 0 % = stop exactly at target

        print("\n--- Arc demo: from loc_a via arc_via to arc_end ---")
        rob.MoveLine(loc_a, fd, 10)
        rob.MoveArc(arc_via, arc_end, fd, 0)

        print("\n--- PauseMotion / ResumeMotion demo ---")
        rob.MoveLine(loc_b, fd, 10)
        rob.MoveLine(loc_c, fd, 10)
        time.sleep(0.5)
        rob.PauseMotion()
        print("  Robot paused — sleeping 2 s …")
        time.sleep(2.0)
        rob.ResumeMotion()
        print("  Robot resumed.")

        wait_done(rob)
        print("\nAll motions complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            code = int(str(e).rsplit(" ", 1)[1])
            sys.exit(128 + code)
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
