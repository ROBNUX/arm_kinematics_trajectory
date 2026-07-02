#!/usr/bin/env python3
"""
XYZ_UR 5-DOF robot simulation: XYZ Gantry + UJNT (2-DOF tilt/pan head).

The XYZ_UR kinematics splits into two sub-modules:
  • XYZ sub-robot:  3 prismatic joints (Z, -Y, X)  [JOINT_0_ACT .. JOINT_2_ACT]
  • UJNT sub-robot: 2 revolute joints (tilt, pan)   [JOINT_3_ACT .. JOINT_4_ACT]

DH parameters fed to xyz_ur CreateRobot (concatenation of both sub-modules):
  XYZ part (3-DOF):
    alpha=[0, PI/2, -PI/2], a=[0,0,0], theta=[0,0,0], d=[0,0,0]
  UJNT part (2-DOF, R-R U-joint):
    alpha=[0, -PI/2], a=[0,0], theta=[0,0], d=[0.05, 0.05]
    (chosen so that at q3=q4=0, joint3's axis is world Z (pan) and
    joint4's axis is world Y (tilt) -- see UJNT::CartToJnt for the
    matching closed-form IK derivation)

Combined: 5-DOF vector (XYZ first, then UJNT appended).

Run:
    python3 simulate_xyz_ur.py
Or:
    ros2 run robnux_arm_sim simulate_xyz_ur.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi

# ─────────────────────────────────────── geometry ────────────────────────────
# XYZ_UR SetGeometry expects [xyz_params | ujnt_params] interleaved in DH format
# Since XYZ_UR internally builds two sub-robots, we pass the combined DH:
# alpha=[0, PI/2,-PI/2, 0, -PI/2]  (3 XYZ + 2 UJNT)
ALPHA = [0.0,    PI/2,  -PI/2,  0.0,   -PI/2]
A     = [0.0,    0.0,    0.0,    0.0,     0.0 ]
THETA = [0.0,    0.0,    0.0,    0.0,     0.0 ]
D     = [0.0,    0.0,    0.0,    0.05,    0.05]

PARA = np.array([ALPHA + A + THETA + D]).T   # (20×1)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF  = 5
HOME_JNT = np.zeros((DOF, 1))


def wait_done(rob: m.Robot, poll_s: float = 0.1) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def main() -> int:
    print("=" * 60)
    print("XYZ_UR simulation — 5-DOF XYZ gantry + tilt/pan head")
    print("=" * 60)

    pf  = m.Profile(0.1, 0.5, 5.0, 0.1, 0.5, 5.0)
    jpf = m.JntProfile(0.5, 2.0, 20.0)
    rob = m.Robot("xyz_ur", PARA, BASE_OFF, BASE_OFF, pf)

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
    rob.SetSpeed(m.Percent(60, 60, 60))

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    def loc(x, y, z, yaw=0.0, pitch=0.0, roll=0.0):
        return m.LocData(x, y, z, yaw, pitch, roll, cfg, turns)

    try:
        # ── demo 1: Cartesian path (XYZ only, head at neutral) ────────────────
        print("\n--- Demo 1: XY path at fixed Z (tilt/pan = 0) ---")
        z0 = home_loc.z
        path = [
            loc( 0.25,  0.0,   z0),
            loc( 0.25,  0.25,  z0),
            loc(-0.25,  0.25,  z0),
            loc(-0.25, -0.25,  z0),
            loc( 0.0,   0.0,   z0),
        ]
        for i, wp in enumerate(path):
            rob.MoveLine(wp, fd, 5)
            print(f"  Step {i}: ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f})")
        wait_done(rob)

        # ── demo 2: joint-space head tilt/pan sweep ────────────────────────────
        print("\n--- Demo 2: Tilt/pan head sweep (joints 3 and 4) ---")
        rob.SetSpeed(m.Percent(40, 40, 40))
        head_poses = [
            [0, 0, -0.3, math.radians( 30), math.radians( 0)],
            [0, 0, -0.3, math.radians(-30), math.radians( 0)],
            [0, 0, -0.3, math.radians(  0), math.radians(20)],
            [0, 0, -0.3, math.radians(  0), math.radians(-20)],
            [0, 0,  0.0, 0.0, 0.0],
        ]
        for jnts in head_poses:
            tgt = np.array([jnts]).T
            rob.MovePTPJ(tgt, 0)
            wait_done(rob)
            print(f"  Head jnts: {[f'{math.degrees(v):.1f}' for v in jnts[3:]]}")

        # ── demo 3: combined XYZ + head motion ─────────────────────────────────
        print("\n--- Demo 3: Combined XYZ traverse + head tracking ---")
        rob.SetSpeed(m.Percent(50, 50, 50))
        combo = [
            # [jnt0_Z, jnt1_Y, jnt2_X, jnt3_tilt_rad, jnt4_pan_rad]
            [-0.2, 0.1, 0.2, math.radians(15), math.radians(10)],
            [-0.3, -0.1, -0.2, math.radians(-15), math.radians(-10)],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        for jnts in combo:
            tgt = np.array([jnts]).T
            rob.MovePTPJ(tgt, 0)
            wait_done(rob)
            ok_fk, cart = rob.GetCartFromJnt(np.array(jnts), 7)
            if ok_fk:
                print(f"  TCP at ({cart[0]:.3f}, {cart[1]:.3f}, {cart[2]:.3f})")

        print("\nXYZ_UR simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
