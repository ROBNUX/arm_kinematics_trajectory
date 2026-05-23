#!/usr/bin/env python3
"""Example 03 — fit an S-curve trajectory segment and report its samples.

Drives kinematics_lib::SegmentPlanning::fit_traj_segment_samples through the
ScurvePlanner Python binding. The C++ method takes the sampling-output Eigen
references in-place, so this script pre-allocates numpy buffers and lets the
binding fill them.

Plots the result if matplotlib is available; otherwise just prints a summary.
"""

import numpy as np

import rob_motion_commands as m


def main() -> int:
    p_start, p_end = 0.0, 1.0
    v_start, v_end = 0.0, 0.0
    p_max, v_max, a_max, j_max = 10.0, 2.0, 4.0, 10.0
    n = 200

    t = np.zeros(n, dtype=np.float64)
    pos = np.zeros(n, dtype=np.float64)
    vel = np.zeros(n, dtype=np.float64)
    acc = np.zeros(n, dtype=np.float64)
    jrk = np.zeros(n, dtype=np.float64)

    boundaries = m.ScurvePlanner.FitScurveSegment(
        p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max,
        n, t, pos, vel, acc, jrk,
    )

    print(f"S-curve planning from {p_start} to {p_end} "
          f"(v_max={v_max}, a_max={a_max}, j_max={j_max})")
    print(f"  segment boundaries (s): {[round(b, 4) for b in boundaries]}")
    print(f"  duration: {t[-1]:.4f} s")
    print(f"  max |vel|: {np.max(np.abs(vel)):.4f}  (limit {v_max})")
    print(f"  max |acc|: {np.max(np.abs(acc)):.4f}  (limit {a_max})")
    print(f"  max |jerk|: {np.max(np.abs(jrk)):.4f}  (limit {j_max})")
    print(f"  final pos: {pos[-1]:.6f}  (target {p_end})")

    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
        ax[0].plot(t, pos); ax[0].set_ylabel("pos")
        ax[1].plot(t, vel); ax[1].set_ylabel("vel")
        ax[2].plot(t, acc); ax[2].set_ylabel("acc")
        ax[3].plot(t, jrk); ax[3].set_ylabel("jerk")
        ax[3].set_xlabel("time (s)")
        fig.suptitle("S-curve profile")
        plt.tight_layout()
        plt.savefig("/tmp/scurve_example.png", dpi=80)
        print("  plot saved: /tmp/scurve_example.png")
    except ImportError:
        print("  (matplotlib not installed; skipping plot)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
