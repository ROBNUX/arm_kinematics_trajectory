#!/usr/bin/env python3
"""
Random tests for the single-axis S-curve motion profile generator
(SegmentPlanning::fit_traj_segment_samples, exposed as m.ScurvePlanner.
FitScurveSegment) -- the primitive every trajectory type (LIN, ARC, PTP,
PTPJ) in this repo blends onto its path parameter.

Unlike a live MoveLine/MoveArc execution test (which depends on ROS timing,
publish rates, and IK correctness all at once), this is deterministic and
fast: same profile in, same samples out, every run. It isolates the motion
*profile* math from the Cartesian/IK layer, and is what to reach for first
when chasing a "trajectory looked wrong" report, since it either exonerates
or indicts the profile generator in milliseconds.

Each case is derived from `seed + case_index` alone (not a continuously
advancing RNG stream), so any single case can be reproduced or re-run in
isolation with --case. This matters because a boundary-velocity segment
opposing the displacement direction (e.g. p: 1.0->0.0 with v_start=0.5,
v_end=0.3, both positive) segfaults fit_traj_segment_samples outright --
see memory/reference this script leaves behind. The test driver runs all
cases in a single child subprocess and, if that child dies, records the
in-flight case as a crash and respawns a fresh child to keep going, so one
segfault doesn't take down the other 99 cases' results.

For each of N cases:
  1. Sample the profile at n_points, evenly spaced in time.
  2. Check boundary conditions: pos[0]==p_start, pos[-1]==p_end,
     vel[0]==v_start, vel[-1]==v_end.
  3. Check the reported limits are respected: max|vel|<=v_max, max|acc|<=a_max,
     max|jerk|<=j_max (small tolerance for sampling not landing exactly on a
     peak).
  4. Cross-check self-consistency by finite-differencing the position/
     velocity/acceleration samples and comparing against the returned
     velocity/acceleration/jerk samples (catches a profile whose pos/vel/acc
     arrays don't actually agree with each other).
  5. Check time_list is strictly increasing.

Run (ROS2 + /opt/robnux overlay sourced):
    python3 test_scurve_profile.py [N] [--seed S]
    python3 test_scurve_profile.py --case 2 --seed 7   # re-run one case in-process
"""
import argparse
import json
import subprocess
import sys

import numpy as np

DEFAULT_N = 100
# Central-difference error at a jerk-switch corner (acceleration continuous
# but not differentiable) scales ~O(1/N), not O(1/N^2) -- empirically ~5%
# at N=300 vs ~0.3% at N=3000 for the same profile. Use enough points that
# corner artifacts stay well under FD_REL_TOL, so a real mismatch still
# stands out.
N_POINTS = 3000
TOL_BOUNDARY = 1e-6
LIMIT_MARGIN = 1e-3          # relative slack for max|v|/max|a|/max|j| checks
FD_REL_TOL = 0.02            # finite-difference cross-check tolerance


def finite_diff(y, t):
    return np.gradient(y, t)


def case_params(seed: int, case_index: int):
    rng = np.random.default_rng(seed + case_index)
    v_max = rng.uniform(0.5, 5.0)
    a_max = rng.uniform(1.0, 20.0)
    j_max = rng.uniform(5.0, 100.0)
    v_start = rng.uniform(-0.8 * v_max, 0.8 * v_max)
    v_end = rng.uniform(-0.8 * v_max, 0.8 * v_max)
    p_start = rng.uniform(-2.0, 2.0)
    p_end = p_start + rng.uniform(-3.0, 3.0)
    # p_max is a *position* limit (symmetric +-p_max), not a velocity term --
    # keep it generous so it never binds; this test is about the velocity/
    # acceleration/jerk-limited profile shape, not position-limit clamping.
    p_max = max(abs(p_start), abs(p_end)) * 4 + 100.0
    return dict(p_start=p_start, p_end=p_end, v_start=v_start, v_end=v_end,
                p_max=p_max, v_max=v_max, a_max=a_max, j_max=j_max)


def run_case(m, params):
    t = np.zeros(N_POINTS)
    p = np.zeros(N_POINTS)
    v = np.zeros(N_POINTS)
    a = np.zeros(N_POINTS)
    j = np.zeros(N_POINTS)
    bounds = m.ScurvePlanner.FitScurveSegment(
        params["p_start"], params["p_end"], params["v_start"], params["v_end"],
        params["p_max"], params["v_max"], params["a_max"], params["j_max"],
        N_POINTS, t, p, v, a, j)

    problems = []
    if len(bounds) == 0:
        problems.append("FitScurveSegment returned empty bounds (internal failure)")
        return problems

    if not np.all(np.diff(t) > 0):
        problems.append("time_list is not strictly increasing")

    if abs(p[0] - params["p_start"]) > TOL_BOUNDARY:
        problems.append(f"pos[0]={p[0]:.6g} != p_start={params['p_start']:.6g}")
    if abs(p[-1] - params["p_end"]) > TOL_BOUNDARY:
        problems.append(f"pos[-1]={p[-1]:.6g} != p_end={params['p_end']:.6g}")
    if abs(v[0] - params["v_start"]) > TOL_BOUNDARY:
        problems.append(f"vel[0]={v[0]:.6g} != v_start={params['v_start']:.6g}")
    if abs(v[-1] - params["v_end"]) > TOL_BOUNDARY:
        problems.append(f"vel[-1]={v[-1]:.6g} != v_end={params['v_end']:.6g}")

    max_v, max_a, max_j = np.max(np.abs(v)), np.max(np.abs(a)), np.max(np.abs(j))
    if max_v > params["v_max"] * (1 + LIMIT_MARGIN):
        problems.append(f"max|vel|={max_v:.6g} exceeds v_max={params['v_max']:.6g}")
    if max_a > params["a_max"] * (1 + LIMIT_MARGIN):
        problems.append(f"max|acc|={max_a:.6g} exceeds a_max={params['a_max']:.6g}")
    if max_j > params["j_max"] * (1 + LIMIT_MARGIN):
        problems.append(f"max|jerk|={max_j:.6g} exceeds j_max={params['j_max']:.6g}")

    v_fd = finite_diff(p, t)
    a_fd = finite_diff(v, t)
    scale_v = max(max_v, 1e-6)
    scale_a = max(max_a, 1e-6)
    core = slice(2, -2)
    dv = np.max(np.abs(v_fd[core] - v[core])) / scale_v
    da = np.max(np.abs(a_fd[core] - a[core])) / scale_a
    if dv > FD_REL_TOL:
        problems.append(f"d(pos)/dt vs vel mismatch: rel_err={dv:.4f}")
    if da > FD_REL_TOL:
        problems.append(f"d(vel)/dt vs acc mismatch: rel_err={da:.4f}")

    return problems


def run_child(n: int, start_index: int, seed: int):
    """Runs cases [start_index, n) in-process. Prints CASE_PARAMS before each
    risky call and RESULT_JSON after, both flushed immediately, so the parent
    can tell exactly which case was in flight if this process dies."""
    import rob_motion_commands as m
    for i in range(start_index, n):
        params = case_params(seed, i)
        print(f"CASE_PARAMS:{i}:" + json.dumps(params), flush=True)
        problems = run_case(m, params)
        print(f"RESULT_JSON:{i}:" + json.dumps(problems), flush=True)


def run_parent(n: int, seed: int):
    """Spawns the child; if it dies mid-case, records that case as a crash
    and respawns a fresh child to continue with the next case."""
    results = {}
    start = 0
    last_params_by_case = {}
    while start < n:
        proc = subprocess.Popen(
            [sys.executable, __file__, "--_child", str(n), str(start), str(seed)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        last_case_seen = start - 1
        for line in proc.stdout:
            line = line.rstrip("\n")
            if line.startswith("CASE_PARAMS:"):
                _, idx, payload = line.split(":", 2)
                idx = int(idx)
                last_params_by_case[idx] = json.loads(payload)
                last_case_seen = idx
            elif line.startswith("RESULT_JSON:"):
                _, idx, payload = line.split(":", 2)
                idx = int(idx)
                results[idx] = json.loads(payload)
        proc.wait()
        next_start = max(results.keys(), default=start - 1) + 1
        if proc.returncode != 0 and next_start <= start:
            # crashed before completing (or even starting) any new case in
            # this batch -- attribute the crash to the case it never
            # finished (or the batch's first case, if it died before even
            # printing CASE_PARAMS) so the loop always makes progress.
            crashed_case = max(last_case_seen, start)
            stderr_tail = proc.stderr.read()
            tail = "\n".join(stderr_tail.strip().splitlines()[-4:])
            results[crashed_case] = [
                f"CRASHED (exit {proc.returncode}) with params="
                f"{last_params_by_case.get(crashed_case, case_params(seed, crashed_case))}; "
                f"stderr tail: {tail}"
            ]
            next_start = crashed_case + 1
        start = next_start
    return [results[i] for i in range(n)]


def main() -> int:
    # Child invocation uses its own fixed positional layout (see run_parent's
    # subprocess.Popen call) so it can't collide with the top-level parser's
    # "n" positional below.
    if "--_child" in sys.argv:
        idx = sys.argv.index("--_child")
        n, start, seed = (int(x) for x in sys.argv[idx + 1: idx + 4])
        run_child(n, start, seed)
        return 0

    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int, default=DEFAULT_N)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--case", type=int, default=None,
                     help="re-run a single case index in-process (for debugging a failure)")
    args = ap.parse_args()

    if args.case is not None:
        import rob_motion_commands as m
        params = case_params(args.seed, args.case)
        print(f"case {args.case}: {params}")
        problems = run_case(m, params)
        print("PASS" if not problems else f"FAIL: {problems}")
        return 1 if problems else 0

    print("=" * 78)
    print(f"S-curve profile test — {args.n} random segments, seed={args.seed}")
    print("=" * 78)
    all_problems = run_parent(args.n, args.seed)
    n_fail = 0
    for i, problems in enumerate(all_problems):
        if problems:
            n_fail += 1
            print(f"[FAIL] case {i}: {case_params(args.seed, i)}")
            for pr in problems:
                print(f"         {pr}")

    print("=" * 78)
    if n_fail:
        print(f"RESULT: {n_fail}/{args.n} random segment(s) FAILED")
    else:
        print(f"RESULT: all {args.n} random segment(s) PASSED")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
