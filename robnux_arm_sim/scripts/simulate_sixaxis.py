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
# J5 (index 4) = 30° keeps wrist away from singularity at J5=0
READY_JNT = np.array([0.0, 0.0, 0.0, 0.0, math.radians(30), 0.0])


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

    # ── ready FK — box center and orientation come from here ─────────────────
    ok_r, ready_loc = rob.ForwardKin(READY_JNT)
    if not ok_r:
        print("ERROR: FK at ready pose failed.")
        rob.Shutdown()
        return 1

    print(f"Ready: x={ready_loc.x:.4f}  y={ready_loc.y:.4f}  z={ready_loc.z:.4f}")
    print(f"       A={ready_loc.A:.4f}  B={ready_loc.B:.4f}  C={ready_loc.C:.4f}")
    print(f"       branch={ready_loc.G}  turn={ready_loc.T}")

    xh, yh, zh = ready_loc.x, ready_loc.y, ready_loc.z
    ha, hb, hc = ready_loc.A, ready_loc.B, ready_loc.C
    cfg, turns  = ready_loc.G, ready_loc.T

    rob.SetFeedback(HOME_JNT.reshape(-1, 1))
    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    rob.StartMotion()
    rob.SetSpeed(m.Percent(60, 60, 60))

    print("\nROS2 topics now live (/joint_states, /cart_pose, /arm_path).")
    print("Starting motion in 4 seconds...")
    time.sleep(4)

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    # ── Queue PTP to ready pose + Demo 1 box all at once ─────────────────────
    print("\n--- Moving to ready pose (PTP) then Cartesian box in XZ plane ---")
    rob.MovePTPJ(READY_JNT.reshape(-1, 1), 0)
    print("  PTP to ready pose: queued")

    dx, dz = 0.05, 0.05
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

        # ── Demo 2: arc in XY plane ───────────────────────────────────────────
        print("\n--- Demo 2: Arc in XY plane ---")
        rob.StartMotion()  # re-arm after wait_done
        rob.SetSpeed(m.Percent(60, 60, 60))
        r_arc = 0.05
        rob.MoveLine(m.LocData(xh - r_arc, yh, zh, ha, hb, hc, cfg, turns), fd, 5)
        arc_via = m.LocData(xh + r_arc, yh + r_arc, zh, ha, hb, hc, cfg, turns)
        arc_end = m.LocData(xh, yh + 2 * r_arc, zh, ha, hb, hc, cfg, turns)
        rob.MoveArc(arc_via, arc_end, fd, 0)
        if wait_done(rob):
            print("  Arc motion complete.")

        # ── Demo 3: joint-space PTP (all queued together) ─────────────────────
        print("\n--- Demo 3: Joint-space PTP sweep ---")
        rob.StartMotion()  # re-arm after wait_done
        rob.SetSpeed(m.Percent(40, 40, 40))
        targets_deg = [
            [  15, -15,  0,  0,  30,  0],
            [ -15,  15, -5,  0, -30,  0],
            [   0,   0,  0,  0,  30,  0],  # return close to ready pose (J5=30 non-singular)
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([math.radians(d) for d in tgt])
            ok_fk, cart = rob.ForwardKin(jnt_tgt)
            if ok_fk:
                print(f"  PTP to {tgt} → FK: x={cart.x:.3f} y={cart.y:.3f} z={cart.z:.3f}")
            rob.MovePTPJ(jnt_tgt.reshape(-1, 1), 0)
        if wait_done(rob):
            print("  PTP sweep complete.")

        # ── Demo 4: IK round-trip ─────────────────────────────────────────────
        print("\n--- Demo 4: FK→IK round-trip check ---")
        for q_deg in [[10, -5, 3, 0, 20, 0], [-10, 5, -3, 0, -20, 0]]:
            jnt = np.array([math.radians(d) for d in q_deg])
            ok_fk, loc = rob.ForwardKin(jnt)
            if ok_fk:
                ok_ik, jnt_rec = rob.InverseKin(loc, DOF)
                if ok_ik:
                    err = np.linalg.norm(np.array(jnt_rec) - jnt)
                    print(f"  round-trip error: {err:.2e} rad "
                          f"({'PASS' if err < 1e-3 else 'WARN'})")
                else:
                    print("  IK failed")
            else:
                print("  FK failed")

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
