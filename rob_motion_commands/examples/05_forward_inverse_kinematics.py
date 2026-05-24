#!/usr/bin/env python3
"""Example 05 — forward and inverse kinematics with a Quattro robot.

Shows the four kinematic helpers exposed on Robot without executing any
motion:

    GetCartFromJnt(jnt, cart_size)  -> (ok, flat_cart_vec)
    GetPoseFromJnt(jnt, pose_size)  -> (ok, flat_pose_vec)
    ForwardKin(jnt)                 -> (ok, LocData)
    InverseKin(pose, dof)           -> (ok, jnt)

Run from a sourced ROS 2 workspace:

    python3 examples/05_forward_inverse_kinematics.py
"""

import math

import numpy as np

import rob_motion_commands as m

# --------------------------------------------------------------------------
# Quattro robot geometry parameters
#   [R1, alpha, b1, c1, d1, h, r1, m_platform]
# --------------------------------------------------------------------------
PARA = np.array([[0.3, -math.pi / 2.0, 0.4, 0.0, 1.02, 0.1, 0.0,
                  0.18 * math.sqrt(2)]]).T

# Base offset: (tx, ty, tz, qw, qx, qy, qz) — identity quaternion at origin
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF = 4   # Quattro has 4 active joints


def main() -> int:
    print("=" * 60)
    print("Forward / inverse kinematics — quattro robot")
    print("=" * 60)

    pf = m.Profile(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0)
    rob = m.Robot("quattro", PARA, BASE_OFF, BASE_OFF, pf)

    # ----------------------------------------------------------------
    # Test several joint configurations
    # ----------------------------------------------------------------
    test_configs = [
        np.zeros((DOF, 1)),                                # home position
        np.array([[0.1, -0.1, 0.1, -0.1]]).T,
        np.array([[0.3,  0.3, 0.3,  0.3]]).T,
    ]

    for idx, jnt in enumerate(test_configs):
        print(f"\n--- Config {idx}: jnt = {jnt.T}")

        # ---- GetCartFromJnt: flat Cartesian vector (x,y,z,qw,qx,qy,qz) ----
        ok_c, cart = rob.GetCartFromJnt(jnt, 7)
        if ok_c:
            print(f"  GetCartFromJnt  -> x={cart[0]:.4f} y={cart[1]:.4f} "
                  f"z={cart[2]:.4f}")
        else:
            print("  GetCartFromJnt  -> FAILED (singular / out of workspace)")

        # ---- GetPoseFromJnt: flat pose vector (x,y,z, yaw,pitch,roll) ------
        ok_p, pose_vec = rob.GetPoseFromJnt(jnt, 6)
        if ok_p:
            print(f"  GetPoseFromJnt  -> x={pose_vec[0]:.4f} y={pose_vec[1]:.4f} "
                  f"z={pose_vec[2]:.4f}  "
                  f"yaw={math.degrees(pose_vec[3]):.2f}° "
                  f"pitch={math.degrees(pose_vec[4]):.2f}° "
                  f"roll={math.degrees(pose_vec[5]):.2f}°")
        else:
            print("  GetPoseFromJnt  -> FAILED")

        # ---- ForwardKin: returns LocData ------------------------------------
        ok_fk, loc = rob.ForwardKin(jnt)
        if ok_fk:
            print(f"  ForwardKin      -> x={loc.x:.4f} y={loc.y:.4f} "
                  f"z={loc.z:.4f}  branch={loc.G} turn={loc.T}")
        else:
            print("  ForwardKin      -> FAILED")
            continue

        # ---- InverseKin: recover joint angles from LocData -----------------
        ok_ik, jnt_recovered = rob.InverseKin(loc, DOF)
        if ok_ik:
            err = np.linalg.norm(jnt_recovered - jnt.flatten())
            print(f"  InverseKin      -> jnt = {jnt_recovered.round(4)}")
            print(f"  Round-trip error: {err:.2e} rad  "
                  f"({'PASS' if err < 1e-4 else 'WARN — large error'})")
        else:
            print("  InverseKin      -> FAILED (singular / out of workspace)")

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
