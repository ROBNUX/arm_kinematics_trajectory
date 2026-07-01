#!/usr/bin/env python3
"""
FK/IK round-trip verification for SixAxis_1 box waypoints.

For each Cartesian waypoint this script:
  1. Tries InverseKin with cfg=5 (not_overhead, original) and cfg=4 (overhead, fix)
  2. Runs ForwardKin on the recovered joints to measure the Cartesian round-trip error
  3. Prints joint angles (degrees) so you can see whether the IK returns a
     physically sensible configuration near BOX_JNT = [0,0,0,0,30,0] deg.

Run (from a sourced ROS2 workspace):
    python3 test_fk_ik_sixaxis.py
Or:
    ros2 run robnux_arm_sim test_fk_ik_sixaxis.py
"""

import math
import numpy as np
import rob_motion_commands as m

PI = math.pi

# ── DH parameters (same as simulate_sixaxis.py) ──────────────────────────────
ALPHA = [0,      -PI/2,  0,      -PI/2,   PI/2,  -PI/2]
A     = [0,       0.075, 0.25,    0.03,    0,      0   ]
THETA = [0,       PI/2,  0,       0,       0,      0   ]
D     = [0.33,    0,     0,       0.25,    0,      0.09]

PARA     = np.array([ALPHA + A + THETA + D]).T   # (24×1)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
DOF      = 6

BOX_JNT  = np.array([0.0, 0.0, 0.0, 0.0, math.radians(30), 0.0])


def ik_and_verify(rob, label, loc_in, cfg, dof):
    """Run IK with given cfg, then FK to verify round-trip. Prints one row."""
    test_loc = m.LocData(loc_in.x, loc_in.y, loc_in.z,
                         loc_in.A, loc_in.B, loc_in.C,
                         cfg, loc_in.T)
    ok_ik, jnt = rob.InverseKin(test_loc, dof)
    if not ok_ik:
        print(f"  {label:30s} cfg={cfg}  IK FAILED")
        return

    ok_fk, loc_back = rob.ForwardKin(jnt.reshape(-1, 1))
    if not ok_fk:
        print(f"  {label:30s} cfg={cfg}  FK after IK FAILED")
        return

    xyz_err = math.sqrt((loc_back.x - loc_in.x)**2 +
                        (loc_back.y - loc_in.y)**2 +
                        (loc_back.z - loc_in.z)**2)
    jnt_deg = np.degrees(jnt).round(1)
    print(f"  {label:30s} cfg={cfg}  FK_err={xyz_err:.4f}m  "
          f"jnt_deg={jnt_deg}  (G={loc_back.G})")


def main() -> int:
    print("=" * 70)
    print("SixAxis_1 FK/IK round-trip check — box waypoints")
    print("=" * 70)

    pf  = m.Profile(0.05, 0.2, 2.0, 0.1, 0.5, 5.0)
    rob = m.Robot("sixaxis_1", PARA, BASE_OFF, BASE_OFF, pf)

    # ── Step 1: FK at BOX_JNT ────────────────────────────────────────────────
    ok, box_loc = rob.ForwardKin(BOX_JNT)
    if not ok:
        print("ERROR: FK at BOX_JNT failed")
        rob.Shutdown()
        return 1

    xh, yh, zh   = box_loc.x, box_loc.y, box_loc.z
    ha, hb, hc   = box_loc.A, box_loc.B, box_loc.C
    turns        = box_loc.T
    dx, dz       = 0.08, 0.06

    print(f"\nFK(BOX_JNT=[0,0,0,0,30°,0]):")
    print(f"  x={xh:.4f}  y={yh:.4f}  z={zh:.4f}")
    print(f"  A={math.degrees(ha):.2f}°  B={math.degrees(hb):.2f}°  "
          f"C={math.degrees(hc):.2f}°")
    print(f"  branch(G)={box_loc.G}  turn(T)={turns}")

    # ── Step 2: IK round-trip on BOX_JNT pose ───────────────────────────────
    # SingleInt2RobnuxBranch unpacks MSB-first:
    #   cfg[0] = bit2  (0=overhead, 1=not_overhead)
    #   cfg[1] = bit1  (0=below,    1=above)
    #   cfg[2] = bit0  (0=flip,     1=non-flip)
    # So overhead branch requires in_flag < 4 (bit2 = 0).
    print("\n--- IK round-trip on BOX_JNT pose (expecting jnt≈[0,0,0,0,30,0]°) ---")
    print(f"  {'label':30s} {'cfg':>3}  {'branch':<22}  FK_err   jnt_deg")
    cfg_labels = {
        0: "overhead,  below,  flip",
        1: "overhead,  below,  no-flip",
        2: "overhead,  above,  flip",
        3: "overhead,  above,  no-flip",
        4: "not-ovhd,  below,  flip",
        5: "not-ovhd,  below,  no-flip",
        6: "not-ovhd,  above,  flip",
        7: "not-ovhd,  above,  no-flip",
    }
    for cfg in range(8):
        ik_and_verify(rob, cfg_labels[cfg], box_loc, cfg, DOF)

    # ── Step 3: All 5 box waypoints ─────────────────────────────────────────
    waypoints = [
        ("WP1 (+dx,    z   )", m.LocData(xh+dx, yh, zh,    ha, hb, hc, 0, turns)),
        ("WP2 (+dx,    z+dz)", m.LocData(xh+dx, yh, zh+dz, ha, hb, hc, 0, turns)),
        ("WP3 (-dx,    z+dz)", m.LocData(xh-dx, yh, zh+dz, ha, hb, hc, 0, turns)),
        ("WP4 (-dx,    z   )", m.LocData(xh-dx, yh, zh,    ha, hb, hc, 0, turns)),
        ("WP5 (center, z   )", m.LocData(xh,    yh, zh,    ha, hb, hc, 0, turns)),
    ]

    # Find which cfg gives the best IK (nearest to BOX_JNT configuration)
    print("\n--- All box waypoints: try cfg=1 (overhead,below,no-flip) vs cfg=5 ---")
    for label, base_loc in waypoints:
        for cfg in [1, 5]:
            ik_and_verify(rob, label, base_loc, cfg, DOF)
        print()

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
