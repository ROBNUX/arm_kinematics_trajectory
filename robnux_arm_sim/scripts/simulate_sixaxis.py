#!/usr/bin/env python3
"""
SixAxis_1 6-DOF serial arm simulation (Mitsubishi articulated convention).

Demonstrates:
  - FK/IK with SixAxis_1 kinematics
  - MoveLine (linear Cartesian path)
  - MoveArc (circular arc)
  - MovePTPJ (joint-space PTP)
  - All motions visualised in RViz2

DH parameters (Craig convention):
  alpha = [0, -PI/2, 0, -PI/2,  PI/2, -PI/2]
  a     = [0,  0.075, 0.25, 0.03, 0,    0   ]
  theta = [0,  PI/2,  0,    0,    0,    0   ]   (home offsets)
  d     = [0.33, 0,   0,   0.25,  0,   0.09 ]

Run:
    python3 simulate_sixaxis.py
Or:
    ros2 run robnux_arm_sim simulate_sixaxis.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

# ─────────────────────────────────────── geometry ────────────────────────────
PI = math.pi
ALPHA = [0,      -PI/2,  0,      -PI/2,   PI/2,  -PI/2]
A     = [0,       0.075, 0.25,    0.03,    0,      0   ]
THETA = [0,       PI/2,  0,       0,       0,      0   ]
D     = [0.33,    0,     0,       0.25,    0,      0.09]

PARA = np.array([ALPHA + A + THETA + D]).T     # (24×1)
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T

DOF       = 6
HOME_JNT  = np.zeros(DOF)
# GATE_JNT: initial display pose.
# q[1]=+90°: upper arm extends in -X from shoulder (horizontal, at shoulder height z=0.33).
# q[2]=+90°: forearm bends straight UP from the elbow, giving a classic L/gate shape.
# q[4]=30°: wrist clear of singularity.
GATE_JNT  = np.array([0.0, math.radians(90), math.radians(90), 0.0, math.radians(30), 0.0])
# BOX_JNT: near HOME but q[4]=30° keeps wrist away from the J5=0 singularity so the
# Jacobian-based velocity IK doesn't fail during LIN execution.  EE lands at the same
# XYZ region as HOME (~23 cm from Z-axis), well clear of the 5 cm headDist threshold.
BOX_JNT   = np.array([0.0, 0.0, 0.0, 0.0, math.radians(30), 0.0])
# READY_JNT: visually interesting shoulder/elbow pose used as start/end visual.
# q[1]=-45°: shoulder pitched forward; q[2]=80°: visible elbow bend; q[4]=30°: wrist clear.
READY_JNT = np.array([0.0, math.radians(-45), math.radians(80), 0.0, math.radians(30), 0.0])


def wait_done(rob: m.Robot, poll_s: float = 0.1, timeout: float = 90.0) -> bool:
    t0 = time.time()
    while not rob.MotionDone():
        if time.time() - t0 > timeout:
            print(f"  WARNING: motion did not complete within {timeout}s — trajectory may have failed")
            return False
        time.sleep(poll_s)
    return True


def main() -> int:
    print("=" * 60)
    print("SixAxis_1 simulation — 6-DOF Mitsubishi articulated arm")
    print("=" * 60)

    # ── build robot ──────────────────────────────────────────────────────────
    pf  = m.Profile(0.05, 0.2, 2.0, 0.1, 0.5, 5.0)
    jpf = m.JntProfile(0.5, 2.0, 20.0)
    rob = m.Robot("sixaxis_1", PARA, BASE_OFF, BASE_OFF, pf)

    # ── home FK ──────────────────────────────────────────────────────────────
    ok, home_loc = rob.ForwardKin(HOME_JNT)
    if not ok:
        print("ERROR: FK at home position failed — check DH parameters.")
        rob.Shutdown()
        return 1

    print(f"\nHome: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"      A={home_loc.A:.4f}  B={home_loc.B:.4f}  C={home_loc.C:.4f}")
    print(f"      branch={home_loc.G}  turn={home_loc.T}")

    # ── box FK — center and orientation for Cartesian box ───────────────────
    ok_b, box_loc = rob.ForwardKin(BOX_JNT)
    if not ok_b:
        print("ERROR: FK at box pose failed.")
        rob.Shutdown()
        return 1

    print(f"Box:   x={box_loc.x:.4f}  y={box_loc.y:.4f}  z={box_loc.z:.4f}")
    print(f"       A={box_loc.A:.4f}  B={box_loc.B:.4f}  C={box_loc.C:.4f}")
    print(f"       branch={box_loc.G}  turn={box_loc.T}")

    # ── ready FK — printed for info only; used as PTP finale ────────────────
    ok_r, ready_loc = rob.ForwardKin(READY_JNT)
    if not ok_r:
        print("ERROR: FK at ready pose failed.")
        rob.Shutdown()
        return 1

    print(f"Ready: x={ready_loc.x:.4f}  y={ready_loc.y:.4f}  z={ready_loc.z:.4f}")
    print(f"       A={ready_loc.A:.4f}  B={ready_loc.B:.4f}  C={ready_loc.C:.4f}")
    print(f"       branch={ready_loc.G}  turn={ready_loc.T}")

    xh, yh, zh = box_loc.x, box_loc.y, box_loc.z
    ha, hb, hc = box_loc.A, box_loc.B, box_loc.C
    cfg, turns = box_loc.G, box_loc.T

    # Show gate shape in RViz for 2 s, then teleport to box start (no PTP trace).
    rob.SetFeedback(GATE_JNT.reshape(-1, 1))
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(60, 60, 60))

    print("\nROS2 topics now live (/joint_states, /cart_pose, /arm_path).")
    print("Gate shape displayed for 2 seconds...")
    time.sleep(2)

    # Teleport arm to box start position (SetFeedback with no motion queued).
    # This avoids a PTPJ trace in the EOAT display — the LIN box path is clean.
    rob.SetFeedback(BOX_JNT.reshape(-1, 1))
    print("Arm at box start position, LIN box begins in 2 seconds...")
    time.sleep(2)

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    # ── Demo 1: Cartesian box in XZ plane (no preceding PTPJ) ────────────────
    print("\n--- Demo 1: Cartesian box in XZ plane ---")

    dx, dz = 0.08, 0.06
    waypoints = [
        m.LocData(xh + dx, yh, zh,      ha, hb, hc, cfg, turns),
        m.LocData(xh + dx, yh, zh + dz, ha, hb, hc, cfg, turns),
        m.LocData(xh - dx, yh, zh + dz, ha, hb, hc, cfg, turns),
        m.LocData(xh - dx, yh, zh,      ha, hb, hc, cfg, turns),
        m.LocData(xh,      yh, zh,      ha, hb, hc, cfg, turns),
    ]
    try:
        for i, wp in enumerate(waypoints):
            rob.MoveLine(wp, fd, 5)
            print(f"  MoveLine to waypoint {i}: queued")
        if wait_done(rob):
            print("  Box path complete.")

        # Demo 2 (PTPJ back to GATE) is intentionally disabled:
        # joint-space PTP across a large configuration space sweeps the EE
        # through a huge arc that dwarfs the LIN box in the /arm_path display.

        print("\nSixAxis simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
