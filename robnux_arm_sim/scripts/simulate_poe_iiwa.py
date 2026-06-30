#!/usr/bin/env python3
"""
iiwa-like 7-DOF arm simulation using serialArmPOE (SevenRSRS topology).

Singh-Kreutz SRS geometry (approximate KUKA iiwa LBR 7 R800):
  All axes alternate Z/Y in base frame at q=0:
    J0: Z  J1: Y  J2: Z  J3: Y  J4: Z  J5: Y  J6: Z

  T_left translations (parent → joint, expressed in world frame at q=0):
    J0: (0, 0, 0.340)   shoulder height
    J1: (0, 0, 0)       concurrent with J0  ┐
    J2: (0, 0, 0)       concurrent with J0  ┘  → shoulder pivot
    J3: (0, 0, 0.400)   shoulder → elbow
    J4: (0, 0, 0.400)   elbow → wrist pivot
    J5: (0, 0, 0)       concurrent with J4  ┐
    J6: (0, 0, 0)       concurrent with J4  ┘  → wrist pivot
  T_right[6] tool offset: (0, 0, 0.126)

  Home FK (q=0): (0, 0, 0.340 + 0.400 + 0.400 + 0.126) = (0, 0, 1.266)

Run:
    python3 simulate_poe_iiwa.py
    ros2 run robnux_arm_sim simulate_poe_iiwa.py

Launch RViz first:
    ros2 launch robnux_arm_sim poe_iiwa_rviz.launch.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi
DOF = 7

# ---------------------------------------------------------------------------
# POE geometry — iiwa-like SevenRSRS
# ---------------------------------------------------------------------------

_AXES = [
    [0, 0, 1],   # J0 waist: Z
    [0, 1, 0],   # J1 shoulder tilt: Y
    [0, 0, 1],   # J2 shoulder pan: Z   ┐ J0,J1,J2 concurrent → shoulder SRS
    [0, 1, 0],   # J3 elbow: Y
    [0, 0, 1],   # J4 forearm roll: Z
    [0, 1, 0],   # J5 wrist tilt: Y     ┐ J4,J5,J6 concurrent → wrist SRS
    [0, 0, 1],   # J6 wrist roll: Z
]

_TL_POS = [
    [0, 0, 0.340],   # J0: shoulder height
    [0, 0, 0     ],  # J1: concurrent with J0
    [0, 0, 0     ],  # J2: concurrent with J0 → shoulder pivot at (0,0,0.340)
    [0, 0, 0.400 ],  # J3: shoulder to elbow
    [0, 0, 0.400 ],  # J4: elbow to wrist pivot
    [0, 0, 0     ],  # J5: concurrent with J4
    [0, 0, 0     ],  # J6: concurrent with J4 → wrist pivot at (0,0,1.140)
]

_TOOL_OFFSET = [0, 0, 0.126]   # wrist → flange

# q=0: arm pointing straight up — singular. READY_JNT bends the arm to a
# working configuration. J1=45°, J3=−90°, J5=45° on Y-axes → accumulated 0° → identity orientation.
HOME_JNT  = np.zeros(DOF)
READY_JNT = np.array([0.0, math.radians(45), 0.0, math.radians(-90), 0.0, math.radians(45), 0.0])
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
    print("iiwa-like simulation — POE 7-DOF SevenRSRS arm")
    print("=" * 60)

    pf  = m.Profile(0.05, 0.2, 2.0, 0.1, 0.5, 5.0)
    jpf = m.JntProfile(0.5, 2.0, 20.0)
    rob = m.Robot("poe_arm_7r", PARA, BASE_OFF, BASE_OFF, pf)

    ok, home_loc = rob.ForwardKin(HOME_JNT)
    if not ok:
        print("ERROR: FK at home failed — check POE parameters.")
        rob.Shutdown()
        return 1

    print(f"\nHome FK: x={home_loc.x:.4f}  y={home_loc.y:.4f}  z={home_loc.z:.4f}")
    print(f"         A={home_loc.A:.4f}  B={home_loc.B:.4f}  C={home_loc.C:.4f}")

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

    # ── Move to ready pose first (PTP — no IK, always succeeds) ──────────
    print("\n--- Moving to ready pose (PTP) ---")
    ok = rob.MovePTPJ(READY_JNT.reshape(-1, 1), 0)
    if not ok:
        print("  ERROR: MovePTPJ to ready pose failed.")
        rob.Shutdown()
        return 1
    if not wait_done(rob):
        rob.Shutdown()
        return 1

    ok_fk, ready_loc = rob.ForwardKin(READY_JNT)
    if not ok_fk:
        print("  ERROR: FK at ready pose failed.")
        rob.Shutdown()
        return 1
    xh, yh, zh = ready_loc.x, ready_loc.y, ready_loc.z
    ha, hb, hc = ready_loc.A, ready_loc.B, ready_loc.C
    cfg, turns = ready_loc.G, ready_loc.T
    print(f"  Ready FK: x={xh:.4f}  y={yh:.4f}  z={zh:.4f}")

    # ── Demo 1: Cartesian box in XY plane ─────────────────────────────────
    print("\n--- Demo 1: Cartesian box in XY plane ---")
    dx, dy = 0.10, 0.10
    waypoints = [
        m.LocData(xh + dx, yh,      zh, ha, hb, hc, cfg, turns),
        m.LocData(xh + dx, yh + dy, zh, ha, hb, hc, cfg, turns),
        m.LocData(xh - dx, yh + dy, zh, ha, hb, hc, cfg, turns),
        m.LocData(xh - dx, yh,      zh, ha, hb, hc, cfg, turns),
        m.LocData(xh,      yh,      zh, ha, hb, hc, cfg, turns),
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

        # ── Demo 2: joint-space PTP sweep ─────────────────────────────────
        print("\n--- Demo 2: Joint-space PTP sweep ---")
        rob.SetSpeed(m.Percent(40, 40, 40))
        targets_deg = [
            [ 20, 25,  10, -60,  10,  25,  0],
            [-15, 60, -10, -80, -10,  60,  0],
            [  0, 45,   0, -90,   0,  45,  0],  # return to ready pose
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([math.radians(d) for d in tgt])
            rob.MovePTPJ(jnt_tgt.reshape(-1, 1), 0)
            wait_done(rob)
            ok_fk, cart = rob.GetCartFromJnt(jnt_tgt, 6)
            if ok_fk:
                print(f"  PTP {tgt}: x={cart[0]:.3f} y={cart[1]:.3f} z={cart[2]:.3f}")

        # ── Demo 3: IK round-trip ─────────────────────────────────────────
        print("\n--- Demo 3: FK→IK round-trip ---")
        for q_deg in [[20, -20, 10, 30, 15, 20, 0], [-15, 15, -5, -20, -10, -15, 0]]:
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

        print("\niiwa simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
