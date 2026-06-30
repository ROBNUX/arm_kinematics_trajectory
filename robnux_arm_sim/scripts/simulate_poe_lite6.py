#!/usr/bin/env python3
"""
Lite6-like compact 6-DOF arm simulation using serialArmPOE (SphericalTwoIntersecting).

Simplified uFactory Lite6-like geometry (all dimensions in metres):
  Joint axes at home (q=0):
    J0: Z  (waist)
    J1: Y  (shoulder) — T_left[1]=0 → J0/J1 concurrent → SphericalTwoIntersecting
    J2: Z  (upper arm twist, NOT parallel to J1)
    J3: Y  (forearm pitch)
    J4: Z  (wrist roll)  ┐
    J5: Y  (wrist pitch) ┘ J3/J4/J5 concurrent → spherical wrist

  T_left translations (parent-to-joint at q=0):
    (0,     0, 0.300)   shoulder height
    (0,     0, 0    )   ZERO — J0 and J1 concurrent at (0,0,0.300)
                         axes[0](Z) ∩ axes[1](Y) → SphericalTwoIntersecting
    (0,     0, 0.270)   upper arm along Z
    (0.280, 0, 0    )   forearm extends along X to wrist pivot
    (0,     0, 0    )   concurrent with J3  ┐
    (0,     0, 0    )   concurrent with J4  ┘  spherical wrist at (0.280, 0, 0.570)
  T_right[5] tool offset: (0, 0, 0.070)

  Home FK (q=0): x≈0.280  y≈0  z≈0.640

Run:
    python3 simulate_poe_lite6.py
    ros2 run robnux_arm_sim simulate_poe_lite6.py

Launch RViz first:
    ros2 launch robnux_arm_sim poe_lite6_rviz.launch.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi
DOF = 6

# ---------------------------------------------------------------------------
# POE geometry — Lite6-like SphericalTwoIntersecting
# ---------------------------------------------------------------------------

_AXES = [
    [0, 0, 1],   # J0 waist: Z
    [0, 1, 0],   # J1 shoulder pitch: Y — concurrent with J0 (T_left[1]=0)
    [0, 0, 1],   # J2 upper arm twist: Z  (NOT parallel to J1=Y → not SphericalTwoParallel)
    [0, 1, 0],   # J3 forearm pitch: Y
    [0, 0, 1],   # J4 wrist roll: Z  ┐
    [0, 1, 0],   # J5 wrist pitch: Y ┘ J3/J4/J5 concurrent → spherical wrist
]

_TL_POS = [
    [0,     0, 0.300],   # J0: shoulder height
    [0,     0, 0    ],   # J1: ZERO — ||p[1]|| = 0 → J0/J1 concurrent = intersecting
    [0,     0, 0.270],   # J2: upper arm along Z
    [0.280, 0, 0    ],   # J3: forearm along X (prevents J2/J3/J4 concurrent detection)
    [0,     0, 0    ],   # J4: concurrent with J3  ┐ wrist pivot at (0.280, 0, 0.570)
    [0,     0, 0    ],   # J5: concurrent with J4  ┘
]

_TOOL_OFFSET = [0, 0, 0.070]   # wrist → flange

# q=0: arm extends in +X at (0.280, 0, 0.640). Small J2 bend avoids singularity.
HOME_JNT  = np.zeros(DOF)
READY_JNT = np.array([0.0, math.radians(-30), math.radians(45), math.radians(30), 0.0, 0.0])
BASE_OFF  = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T


def _make_poe_params(axes, tl_pos, tool_offset):
    N = len(axes)
    p = np.zeros(35 * N)
    for i, ax in enumerate(axes):
        p[3*i : 3*(i+1)] = ax
    base = 3 * N
    for i, t in enumerate(tl_pos):
        T = np.eye(4); T[:3, 3] = t
        p[base + 16*i : base + 16*(i+1)] = T.flatten()
    base += 16 * N
    for i in range(N):
        T = np.eye(4)
        if i == N - 1:
            T[:3, 3] = tool_offset
        p[base + 16*i : base + 16*(i+1)] = T.flatten()
    return p.reshape(-1, 1)


PARA = _make_poe_params(_AXES, _TL_POS, _TOOL_OFFSET)


def wait_done(rob, poll_s=0.1, timeout=90.0):
    t0 = time.time()
    while not rob.MotionDone():
        if time.time() - t0 > timeout:
            print(f"  WARNING: motion did not complete within {timeout}s — trajectory may have failed")
            return False
        time.sleep(poll_s)
    return True


def main() -> int:
    print("=" * 60)
    print("Lite6-like simulation — POE 6-DOF SphericalTwoIntersecting")
    print("=" * 60)

    pf  = m.Profile(0.3, 1.0, 10.0, 0.3, 1.0, 10.0)
    jpf = m.JntProfile(1.0, 5.0, 50.0)
    rob = m.Robot("poe_arm_6r", PARA, BASE_OFF, BASE_OFF, pf)

    # Pre-compute FK (pure computation — no motion required yet)
    ok_fk, ready_loc = rob.ForwardKin(READY_JNT)
    if not ok_fk:
        print("ERROR: FK at ready pose failed — check POE parameters.")
        rob.Shutdown()
        return 1
    xh, yh, zh = ready_loc.x, ready_loc.y, ready_loc.z
    ha, hb, hc = ready_loc.A, ready_loc.B, ready_loc.C
    cfg, turns = ready_loc.G, ready_loc.T

    ok, home_loc = rob.ForwardKin(HOME_JNT)
    if not ok:
        print("ERROR: FK at home failed — check POE parameters.")
        rob.Shutdown()
        return 1
    print(f"\nHome FK:  x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"Ready FK: x={xh:.4f}  y={yh:.4f}  z={zh:.4f}")

    if not rob.SetFeedback(HOME_JNT.reshape(-1, 1)):
        print("ERROR: SetFeedback failed — FK may have failed internally.")
        rob.Shutdown()
        return 1
    print("  SetFeedback: OK")

    for i in range(DOF):
        rob.SetJntProfile(i, jpf)

    if not rob.StartMotion():
        print("ERROR: StartMotion failed — robot not initialized or no feedback.")
        rob.Shutdown()
        return 1
    print("  StartMotion: OK")
    rob.SetSpeed(m.Percent(60, 60, 60))

    print("\nROS2 topics now live (/joint_states, /cart_pose, /arm_path).")
    print("If using PlotJuggler: click Stop then Start to refresh topic list.")
    print("Starting motion in 4 seconds...")
    time.sleep(4)

    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    # ── Queue PTP to ready pose + Demo 1 box all at once (no wait_done between) ──
    print("\n--- Moving to ready pose (PTP) then Cartesian box ---")
    ok = rob.MovePTPJ(READY_JNT.reshape(-1, 1), 0)
    if not ok:
        print("  ERROR: MovePTPJ to ready pose failed.")
        rob.Shutdown()
        return 1
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
            ok = rob.MoveLine(wp, fd, 5)
            print(f"  MoveLine to waypoint {i}: {'queued' if ok else 'FAILED'}")
            if not ok:
                print("  ERROR: MoveLine failed — stopping.")
                rob.Shutdown()
                return 1
        if wait_done(rob):
            print("  Box path complete.")

        # ── Demo 2: Joint-space PTP sweep (all queued together) ────────────
        print("\n--- Demo 2: Joint-space PTP sweep ---")
        rob.StartMotion()  # re-arm after wait_done
        rob.SetSpeed(m.Percent(40, 40, 40))
        targets_deg = [
            [ 20, -20,  35,  25,  20,  0],
            [-15,  10, -15, -20, -15,  0],
            [  0, -30,  45,  30,   0,  0],  # return to ready pose
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([math.radians(d) for d in tgt])
            ok_fk, cart = rob.ForwardKin(jnt_tgt)
            if ok_fk:
                print(f"  PTP to {tgt} → FK: x={cart.x:.3f} y={cart.y:.3f} z={cart.z:.3f}")
            rob.MovePTPJ(jnt_tgt.reshape(-1, 1), 0)
        wait_done(rob)
        print("  PTP sweep complete.")

        # ── Demo 3: FK→IK round-trip ─────────────────────────────────────
        print("\n--- Demo 3: FK→IK round-trip ---")
        for q_deg in [[20, -15, 25, 20, 10, 0], [-10, 10, -15, -10, -8, 0]]:
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

        print("\nLite6 simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
