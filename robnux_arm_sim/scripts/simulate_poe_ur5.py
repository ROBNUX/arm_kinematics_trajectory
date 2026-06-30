#!/usr/bin/env python3
"""
UR5-like 6-DOF arm simulation using serialArmPOE (ThreeParallel topology).

POE parameter layout for SetGeometry (35*N floats, N=6):
  [axes 3*N | T_left 16*N row-major | T_right 16*N row-major]

Approximate UR5 geometry (all dimensions in metres):
  Joint axes at home (q=0):
    J0: Z  (waist)
    J1: Y  (shoulder)
    J2: Y  (elbow)      <- axes[1,2,3] all Y → ThreeParallel topology
    J3: Y  (wrist 1)
    J4: -Z (wrist 2)
    J5: Y  (wrist 3)
  T_left translations (parent-to-joint offsets at q=0):
    (0, 0, 0.0892)    shoulder height
    (0, 0.1358, 0)    shoulder lateral
    (0.4250, 0, 0)    upper arm
    (0.3922, 0, 0)    forearm
    (0, 0.1091, 0)    wrist-1 lateral
    (0, 0, 0.0946)    wrist-2 height
  T_right[5] = I, tool offset along local Z: (0, 0, 0.0823)

Run:
    python3 simulate_poe_ur5.py
    ros2 run robnux_arm_sim simulate_poe_ur5.py

Launch RViz first:
    ros2 launch robnux_arm_sim poe_ur5_rviz.launch.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PI = math.pi
DOF = 6

# ---------------------------------------------------------------------------
# POE geometry — UR5 approximate
# ---------------------------------------------------------------------------

_AXES = [
    [ 0,  0,  1],  # J0 waist: Z
    [ 0,  1,  0],  # J1 shoulder: Y
    [ 0,  1,  0],  # J2 elbow: Y   <- parallel to J1, J3 → ThreeParallel
    [ 0,  1,  0],  # J3 wrist1: Y
    [ 0,  0, -1],  # J4 wrist2: -Z
    [ 0,  1,  0],  # J5 wrist3: Y
]

_TL_POS = [          # T_left translations (parent → joint offset at home)
    [0,       0,      0.0892],
    [0,       0.1358, 0     ],
    [0.4250,  0,      0     ],
    [0.3922,  0,      0     ],
    [0,       0.1091, 0     ],
    [0,       0,      0.0946],
]

_TOOL_OFFSET = [0, 0, 0.0823]   # T_right[5] translation (wrist3 → flange)

HOME_JNT = np.zeros(DOF)
BASE_OFF  = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T


def _make_poe_params(axes, tl_pos, tool_offset):
    """Pack axes / T_left / T_right into the 35*N column vector."""
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
    print("UR5-like simulation — POE 6-DOF ThreeParallel arm")
    print("=" * 60)

    pf  = m.Profile(0.3, 1.0, 10.0, 0.3, 1.0, 10.0)
    jpf = m.JntProfile(1.0, 5.0, 50.0)
    rob = m.Robot("poe_arm_6r", PARA, BASE_OFF, BASE_OFF, pf)

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

    xh, yh, zh = home_loc.x, home_loc.y, home_loc.z
    ha, hb, hc = home_loc.A, home_loc.B, home_loc.C
    cfg, turns = home_loc.G, home_loc.T
    fd = m.FrameData(1, 1, m.IpoMode.WORLD)

    # ── Demo 1: Cartesian box in XY plane ────────────────────────────────
    print("\n--- Demo 1: Cartesian box in XY plane ---")
    dx, dy = 0.12, 0.10
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

        # ── Demo 2: joint-space PTP sweep (all queued together) ────────────
        print("\n--- Demo 2: Joint-space PTP sweep ---")
        rob.StartMotion()  # re-arm after wait_done
        rob.SetSpeed(m.Percent(40, 40, 40))
        targets_deg = [
            [ 30, -30,  60,  0,  45,  0],
            [-30,  10, -20,  0, -30,  0],
            [  0,   0,   0,  0,   0,  0],
        ]
        for tgt in targets_deg:
            jnt_tgt = np.array([math.radians(d) for d in tgt])
            ok_fk, loc = rob.ForwardKin(jnt_tgt)
            if ok_fk:
                print(f"  PTP to {tgt} → FK: x={loc.x:.3f} y={loc.y:.3f} z={loc.z:.3f}")
            rob.MovePTPJ(jnt_tgt.reshape(-1, 1), 0)
        wait_done(rob)
        print("  PTP sweep complete.")

        # ── Demo 3: IK round-trip verification ───────────────────────────
        print("\n--- Demo 3: FK→IK round-trip ---")
        for q_deg in [[20, -20, 40, 0, 30, 0], [-20, 10, -30, 0, -45, 0]]:
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

        print("\nUR5 simulation complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            sys.exit(128 + int(str(e).rsplit(" ", 1)[1]))
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
