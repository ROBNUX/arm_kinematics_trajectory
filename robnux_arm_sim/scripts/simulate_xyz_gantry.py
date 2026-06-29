#!/usr/bin/env python3
"""
XYZGantry 3-DOF Cartesian robot simulation.

IK mapping (from library source):
  q[0] = (p.z - d[0]) / pitch[0]   -> Z stroke
  q[1] = (-p.y - d[1]) / pitch[1]  -> -Y stroke
  q[2] = (p.x - d[2]) / pitch[2]   -> X stroke

DH parameters (Craig convention):
  alpha = [0,   PI/2, -PI/2]   sets Z-then-Y-then-X prismatic axes
  a     = [0,   0,     0   ]
  theta = [0,   0,     0   ]
  d     = [0,   0,     0   ]   (workspace centred at origin)

  pitch = [1, 1, 1]  (default)

Workspace used here:
  X: [-0.4, 0.4]   Y: [-0.4, 0.4]   Z: [-0.5, 0.0]

Run:
    python3 simulate_xyz_gantry.py
Or:
    ros2 run robnux_arm_sim simulate_xyz_gantry.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi

# ─────────────────────────────────────── geometry ────────────────────────────
ALPHA = [0.0,    PI/2,  -PI/2]
A     = [0.0,    0.0,    0.0 ]
THETA = [0.0,    0.0,    0.0 ]
D     = [0.0,    0.0,    0.0 ]

PARA = np.array([ALPHA + A + THETA + D]).T      # (12×1)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF  = 3
HOME_JNT = np.zeros((DOF, 1))   # TCP at (0, 0, 0) when all d_ref=0


def wait_done(rob: m.Robot, poll_s: float = 0.1) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def loc_xyz(x, y, z, cfg, turns):
    return m.LocData(x, y, z, 0, 0, 0, cfg, turns)


def main() -> int:
    print("=" * 60)
    print("XYZGantry simulation — 3-DOF Cartesian robot")
    print("=" * 60)

    pf  = m.Profile(0.1, 0.5, 5.0, 0.1, 0.5, 5.0)
    jpf = m.JntProfile(0.5, 2.0, 20.0)
    rob = m.Robot("xyz_gantry", PARA, BASE_OFF, BASE_OFF, pf)

    # ── home FK ──────────────────────────────────────────────────────────────
    ok, home_loc = rob.ForwardKin(HOME_JNT.flatten())
    if not ok:
        print("ERROR: FK at home failed.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    cfg, turns = home_loc.G, home_loc.T

    rob.SetFeedback(HOME_JNT)
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(70, 70, 70))

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    try:
        # ── demo 1: 3D rectangular path ───────────────────────────────────────
        print("\n--- Demo 1: 3D rectangular traverse ---")
        xs, xe = -0.3, 0.3
        y_pos  = 0.0
        z_top  = -0.1
        z_bot  = -0.4
        path = [
            loc_xyz(xs, y_pos, z_top, cfg, turns),
            loc_xyz(xe, y_pos, z_top, cfg, turns),
            loc_xyz(xe, y_pos, z_bot, cfg, turns),
            loc_xyz(xs, y_pos, z_bot, cfg, turns),
            loc_xyz(xs, y_pos, z_top, cfg, turns),
            loc_xyz(0.0, y_pos, 0.0, cfg, turns),   # back to home
        ]
        for i, wp in enumerate(path):
            rob.MoveLine(wp, fd, 5)
            print(f"  Step {i}: ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f})")
        wait_done(rob)

        # ── demo 2: raster scan pattern ───────────────────────────────────────
        print("\n--- Demo 2: Raster scan in XY plane (z=-0.25) ---")
        rob.SetSpeed(m.Percent(60, 60, 60))
        z_scan = -0.25
        y_rows = [-0.2, 0.0, 0.2]
        for j, y_r in enumerate(y_rows):
            x_start = -0.3 if (j % 2 == 0) else 0.3
            x_end   =  0.3 if (j % 2 == 0) else -0.3
            rob.MoveLine(loc_xyz(x_start, y_r, z_scan, cfg, turns), fd, 5)
            rob.MoveLine(loc_xyz(x_end,   y_r, z_scan, cfg, turns), fd, 0)
            print(f"  Row {j}: y={y_r:.2f}")
        rob.MoveLine(loc_xyz(0, 0, 0, cfg, turns), fd, 0)
        wait_done(rob)

        # ── demo 3: IK check ──────────────────────────────────────────────────
        print("\n--- Demo 3: IK verification ---")
        test_pts = [
            (0.2, -0.3, -0.2),
            (-0.1, 0.15, -0.35),
            (0.0, 0.0, -0.1),
        ]
        for px, py, pz in test_pts:
            pose = np.array([[px, py, pz, 0, 0, 0]]).T
            ok_ik, jnt = rob.GetJntFromPose(pose.flatten(), DOF)
            if ok_ik:
                ok_fk, cart = rob.GetCartFromJnt(jnt, 7)
                if ok_fk:
                    err = math.sqrt((cart[0]-px)**2 + (cart[1]-py)**2 + (cart[2]-pz)**2)
                    print(f"  ({px:.2f},{py:.2f},{pz:.2f}) -> IK+FK err={err:.2e} m "
                          f"({'PASS' if err < 1e-4 else 'WARN'})")
            else:
                print(f"  ({px:.2f},{py:.2f},{pz:.2f}) -> IK failed")

        print("\nXYZGantry simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
