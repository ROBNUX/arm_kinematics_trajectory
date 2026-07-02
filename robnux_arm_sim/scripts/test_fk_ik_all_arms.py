#!/usr/bin/env python3
"""
Random FK/IK round-trip test across every arm topology in robnux_arm_sim.

For each arm in arm_registry.build_all_specs():
  1. Draw a random joint vector q within the arm's safe joint bounds
     (redrawing wrist/shoulder-singular joints per spec.singular_guards).
  2. ForwardKin(q) -> loc.  If FK reports unreachable, redraw (doesn't count
     against the sample budget).
  3. InverseKin(loc) -> q_rec, using the branch/turn code FK just returned.
  4. ForwardKin(q_rec) -> loc2.
  5. Compare loc vs loc2 (position + orientation) -- NOT q vs q_rec, since
     redundant/parallel arms can have multiple joint solutions for one pose.

This is the same technique that found and confirmed the 4 sixaxis_1 IK bugs
(see memory: project_sixaxis_ik_bugs) -- FK(IK(FK(q))) should reproduce the
original Cartesian pose to numerical precision for every reachable sample.

Each arm is tested in its own subprocess. Some kinematics plugins are known
to hard-crash (native Eigen assertion abort, not a Python exception) on
certain inputs (see quattro below) -- without process isolation, one bad
arm would kill the results for the other ten.

Run (ROS2 + /opt/robnux overlay sourced):
    python3 test_fk_ik_all_arms.py [N] [--arm NAME] [--seed S]
"""
import argparse
import json
import math
import subprocess
import sys

import numpy as np

import arm_registry as reg

PI = math.pi
DEFAULT_N = 100
MAX_ATTEMPT_FACTOR = 40
# Some joint-space samples legitimately land on a real singularity that IK
# correctly refuses to solve (e.g. sixaxis_1's wrist center landing within
# SIXDOF_HEADDIST of the base Z-axis -> azimuth is ill-defined). That's
# correct behavior, not a bug, so a small IK-failure rate is tolerated;
# only a rate above this -- or any FK(IK(pose)) mismatch when IK *does*
# succeed -- fails the test.
IK_FAIL_TOLERANCE = 0.15


def euler_zyx_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rot_angle_diff(roll1, pitch1, yaw1, roll2, pitch2, yaw2):
    R1 = euler_zyx_to_R(roll1, pitch1, yaw1)
    R2 = euler_zyx_to_R(roll2, pitch2, yaw2)
    Rd = R1.T @ R2
    tr = np.clip((np.trace(Rd) - 1.0) / 2.0, -1.0, 1.0)
    return abs(math.acos(tr))


def sample_joint(spec: reg.ArmSpec, rng: np.random.Generator):
    q = np.array([rng.uniform(lo, hi) for lo, hi in spec.joint_bounds])
    for idx, eps in spec.singular_guards:
        lo, hi = spec.joint_bounds[idx]
        tries = 0
        while abs(q[idx]) < eps and tries < 25:
            q[idx] = rng.uniform(lo, hi)
            tries += 1
        if abs(q[idx]) < eps:
            return None
    return q


def run_arm_test(m, spec: reg.ArmSpec, n_samples: int, rng: np.random.Generator):
    rob = spec.build_robot(m)
    records = []
    attempts = 0
    unreachable = 0
    max_attempts = n_samples * MAX_ATTEMPT_FACTOR
    try:
        while len(records) < n_samples and attempts < max_attempts:
            attempts += 1
            q = sample_joint(spec, rng)
            if q is None:
                continue
            ok_fk, loc = rob.ForwardKin(q)
            if not ok_fk:
                unreachable += 1
                continue

            ik_dof = spec.ik_dof if spec.ik_dof is not None else spec.dof
            ok_ik, q_rec = rob.InverseKin(loc, ik_dof)
            rec = {"q": q.tolist(), "ok_ik": bool(ok_ik), "ok_fk2": False,
                   "pos_err": None, "rot_err": None}
            if ok_ik:
                ok_fk2, loc2 = rob.ForwardKin(q_rec)
                rec["ok_fk2"] = bool(ok_fk2)
                if ok_fk2:
                    rec["pos_err"] = math.dist((loc.x, loc.y, loc.z),
                                                (loc2.x, loc2.y, loc2.z))
                    rec["rot_err"] = rot_angle_diff(loc.A, loc.B, loc.C,
                                                     loc2.A, loc2.B, loc2.C)
            records.append(rec)
    finally:
        rob.Shutdown()
    return records, attempts, unreachable


def summarize(spec: reg.ArmSpec, records, attempts, unreachable):
    n = len(records)
    ik_fail = sum(1 for r in records if not r["ok_ik"])
    fk2_fail = sum(1 for r in records if r["ok_ik"] and not r["ok_fk2"])
    good = [r for r in records if r["pos_err"] is not None]
    bad = [r for r in good if r["pos_err"] > spec.pos_tol or r["rot_err"] > spec.rot_tol]
    max_pos = max((r["pos_err"] for r in good), default=float("nan"))
    max_rot = max((r["rot_err"] for r in good), default=float("nan"))
    # fk2_fail (IK reported success but its own joints don't FK back) and any
    # `bad` round-trip are always real bugs. A high ik_fail *rate* can also
    # indicate a bug (e.g. a branch that never solves), but a low rate is
    # expected -- singularities are real, finite-measure-zero-adjacent
    # regions that random sampling will occasionally land near.
    ik_fail_rate = ik_fail / n if n else 1.0
    tolerance = spec.ik_fail_tolerance if spec.ik_fail_tolerance is not None else IK_FAIL_TOLERANCE
    passed = (n > 0) and ik_fail_rate <= tolerance and not fk2_fail and not bad
    return {
        "name": spec.name, "n": n, "attempts": attempts, "unreachable": unreachable,
        "ik_fail": ik_fail, "ik_fail_rate": ik_fail_rate, "fk2_fail": fk2_fail, "bad": bad,
        "max_pos_err": max_pos, "max_rot_err": max_rot, "passed": passed,
        "crashed": False,
    }


def print_result_line(res, n_requested):
    status = "PASS" if res["passed"] else "FAIL"
    if res.get("crashed"):
        print(f"[{status}] {res['name']:12s} CRASHED: {res.get('error', 'unknown error')}")
        return
    print(f"[{status}] {res['name']:12s} n={res['n']:3d}/{n_requested} "
          f"attempts={res['attempts']:5d} unreachable={res['unreachable']:4d} "
          f"ik_fail={res['ik_fail']:3d} fk2_fail={res['fk2_fail']:3d} "
          f"max_pos_err={res['max_pos_err']:.3e}m max_rot_err={res['max_rot_err']:.3e}rad "
          f"(bad={len(res['bad'])})")
    for r in res["bad"][:5]:
        print(f"         bad sample: q={np.round(r['q'], 4)} "
              f"pos_err={r['pos_err']:.3e} rot_err={r['rot_err']:.3e}")


def run_child(n_samples: int, arm_name: str, seed: int) -> int:
    """Runs one arm's test in-process and prints a single JSON result line.
    Invoked by the parent as a subprocess so a native crash in one arm's
    kinematics plugin can't take down the other arms' results."""
    import rob_motion_commands as m
    spec = reg.spec_by_name(arm_name)
    rng = np.random.default_rng(seed)
    records, attempts, unreachable = run_arm_test(m, spec, n_samples, rng)
    res = summarize(spec, records, attempts, unreachable)
    print("RESULT_JSON:" + json.dumps(res))
    return 0 if res["passed"] else 1


def run_parent(n_samples: int, specs, seed: int):
    results = []
    for spec in specs:
        proc = subprocess.run(
            [sys.executable, __file__, str(n_samples), "--arm", spec.name,
             "--seed", str(seed), "--_child"],
            capture_output=True, text=True,
        )
        res = None
        for line in proc.stdout.splitlines():
            if line.startswith("RESULT_JSON:"):
                res = json.loads(line[len("RESULT_JSON:"):])
                break
        if res is None:
            tail = "\n".join(proc.stderr.strip().splitlines()[-6:])
            res = {
                "name": spec.name, "n": 0, "attempts": 0, "unreachable": 0,
                "ik_fail": 0, "ik_fail_rate": 1.0, "fk2_fail": 0, "bad": [],
                "max_pos_err": float("nan"), "max_rot_err": float("nan"),
                "passed": False, "crashed": True,
                "error": f"exit code {proc.returncode}; stderr tail:\n{tail}",
            }
        print_result_line(res, n_samples)
        results.append(res)
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int, default=DEFAULT_N,
                     help="round-trip samples per arm (default 100)")
    ap.add_argument("--arm", default=None, help="only test this arm name")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--_child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args._child:
        return run_child(args.n, args.arm, args.seed)

    specs = reg.build_all_specs()
    if args.arm:
        specs = [s for s in specs if s.name == args.arm]
        if not specs:
            print(f"unknown --arm {args.arm!r}; known: "
                  f"{[s.name for s in reg.build_all_specs()]}")
            return 2

    print("=" * 78)
    print(f"FK/IK round-trip test — {args.n} samples/arm, seed={args.seed}")
    print("=" * 78)
    results = run_parent(args.n, specs, args.seed)

    print("=" * 78)
    n_fail = sum(1 for r in results if not r["passed"])
    if n_fail:
        print(f"RESULT: {n_fail}/{len(results)} arm(s) FAILED FK/IK round-trip")
    else:
        print(f"RESULT: all {len(results)} arm(s) PASSED FK/IK round-trip")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
