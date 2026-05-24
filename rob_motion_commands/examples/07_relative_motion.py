#!/usr/bin/env python3
"""Example 07 — relative motion: MoveLineRel and MovePTPRel.

Shows how to move the robot by incremental offsets in both WORLD and TOOL
frames using the relative command variants. A LocData passed to a Rel command
represents a delta (dx, dy, dz, dA, dB, dC) rather than an absolute target.
The IpoMode field in FrameData selects the frame in which the delta is
interpreted:

  IpoMode.WORLD — delta expressed in the world / base frame.
  IpoMode.TOOL  — delta expressed in the current tool frame.

Run from a sourced ROS 2 workspace:

    python3 examples/07_relative_motion.py
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
    print("Relative motion — MoveLineRel / MovePTPRel")
    print("=" * 60)

    pf = m.Profile(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0)
    jpf = m.JntProfile(20.0, 50.0, 200.0)
    rob = m.Robot("quattro", PARA, BASE_OFF, BASE_OFF, pf)

    home_jnt = np.zeros((DOF, 1))
    rob.SetFeedback(home_jnt)

    ok, cur_loc = rob.ForwardKin(home_jnt)
    if not ok:
        print("ForwardKin failed — aborting.")
        rob.Shutdown()
        return 1
    cfg, turns = cur_loc.G, cur_loc.T

    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(70, 70, 70))

    # Move to a well-defined starting position first (absolute)
    start = m.LocData(0.0, 0.0, -0.75, 0.0, 0.0, 0.0, cfg, turns)

    # FrameData for relative moves:
    #   IpoMode.WORLD — delta in world frame
    #   IpoMode.TOOL  — delta in tool frame
    fd_world = m.FrameData(1, 1, m.IpoMode.WORLD)
    fd_tool  = m.FrameData(1, 1, m.IpoMode.TOOL)

    # Small incremental steps (metres / radians)
    step_x = m.LocData(0.05, 0.0,  0.0, 0.0, 0.0, 0.0, cfg, turns)
    step_y = m.LocData(0.0,  0.05, 0.0, 0.0, 0.0, 0.0, cfg, turns)
    step_z = m.LocData(0.0,  0.0, -0.02, 0.0, 0.0, 0.0, cfg, turns)

    try:
        print(f"\nMoving to start position ({start.x}, {start.y}, {start.z}) …")
        rob.MoveLine(start, fd_world, 0)

        print("\n--- MoveLineRel: 4 steps in +X (world frame) ---")
        for i in range(4):
            print(f"  step {i + 1}: +{step_x.x} m in X")
            rob.MoveLineRel(step_x, fd_world, 5)

        print("\n--- MoveLineRel: 4 steps in +Y (world frame) ---")
        for i in range(4):
            rob.MoveLineRel(step_y, fd_world, 5)

        print("\n--- MovePTPRel: 3 steps in -Z (world frame) ---")
        for i in range(3):
            rob.MovePTPRel(step_z, fd_world, 5)

        print("\n--- MoveLineRel: staircase in tool frame ---")
        step_diag = m.LocData(0.04, 0.04, -0.01, 0.0, 0.0, 0.0, cfg, turns)
        for i in range(5):
            rob.MoveLineRel(step_diag, fd_tool, 5)

        wait_done(rob)
        print("\nAll relative motions complete.")

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
