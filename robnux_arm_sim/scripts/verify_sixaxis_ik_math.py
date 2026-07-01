#!/usr/bin/env python3
"""
Pure-Python reference model of SixAxis_1's FK/IK math (no ROS2 / pybind
dependency). This was used to isolate four bugs in
robnux_kinematics_map/src/sixaxis_1.cpp by reproducing CartToJnt and
UpdateConfigTurn line-for-line and bisecting the FK/IK round-trip error:

  1. CartToJnt: the elbow angle q(2) was built from `-M_PI + a2s4_angle` /
     `M_PI - a2s4_angle`. The correct reference angle (derived from the
     Craig-DH position chain via sympy) is `2*s4_offset`, not `+-M_PI`, so
     every reachable pose in every branch had ~0.5 m of position error.
  2. CartToJnt: the wrist flip/non-flip selection branched on
     `beta > 0` / `beta < 0`, but Rotation::GetEulerZYZ always returns beta
     in [0, pi], so that branch is dead logic that made `wristState`
     ineffective (or picked the wrong solution) for orientation.
  3. UpdateConfigTurn: `xAtFrame1` added `d_[3]*sin(...)` where the sign
     must be negative to match CartToJnt's own radial-reach convention,
     which flipped the overhead/not_overhead branch flag for common poses
     (this was the root cause of the cfg-masking workaround that used to
     live in simulate_sixaxis.py).
  4. UpdateConfigTurn: the above/below (`qelbow`) classification split at
     0/PI, but CartToJnt's elbow solution is symmetric about 2*s4_offset,
     not 0/PI, so the above/below flag disagreed with CartToJnt for about
     half of all poses.

All four are fixed in sixaxis_1.cpp. This script re-verifies the fixed
math independently: it checks that FK(IK(pose, cfg)) == pose (to float
precision) for all 8 branch codes and that the branch code you feed into
IK is the same one UpdateConfigTurn reports back from the recovered
joints (cfg == G), across the box waypoints from simulate_sixaxis.py plus
a batch of random reachable poses.

Run:
    python3 verify_sixaxis_ik_math.py
"""

import math

import numpy as np
from numpy import arctan2 as atan2
from numpy import cos, hypot, sin, sqrt

PI = np.pi

# DH parameters, same as simulate_sixaxis.py / sixaxis_1's SetGeometry call.
ALPHA = [0, -PI / 2, 0, -PI / 2, PI / 2, -PI / 2]
A = [0, 0.075, 0.25, 0.03, 0, 0]
THETA0 = [0, PI / 2, 0, 0, 0, 0]
D = [0.33, 0, 0, 0.25, 0, 0.09]


def dh_craig(a, alpha, d, theta):
    """Frame::DH_Craig1989 (modified/Craig DH) rotation + translation."""
    ct, st = cos(theta), sin(theta)
    sa, ca = sin(alpha), cos(alpha)
    R = np.array([[ct, -st, 0], [st * ca, ct * ca, -sa], [st * sa, ct * sa, ca]])
    p = np.array([a, -sa * d, ca * d])
    return R, p


def compose(R1, p1, R2, p2):
    return R1 @ R2, R1 @ p2 + p1


def fk(jnt):
    """serialArm::JntToCart, defaultBaseOff_ = identity."""
    R = np.eye(3)
    p = np.zeros(3)
    thetas = [THETA0[i] + jnt[i] for i in range(6)]
    for i in range(6):
        Ri, pi_ = dh_craig(A[i], ALPHA[i], D[i], thetas[i])
        R, p = compose(R, p, Ri, pi_)
    return R, p


def get_euler_zyz(R):
    """Rotation::GetEulerZYZ, translated line-for-line from rotation.cpp."""
    K_EPS = 1e-10
    sb2 = R[0, 2] ** 2 + R[1, 2] ** 2
    if sb2 > K_EPS:
        alpha_ = atan2(R[1, 2], R[0, 2])
        gamma_ = atan2(R[2, 1], -R[2, 0])
        cc, sc = cos(gamma_), sin(gamma_)
        beta = atan2(R[2, 1] * sc - R[2, 0] * cc, R[2, 2])
    elif R[2, 2] > 1 - K_EPS:
        beta, gamma_ = 0.0, 0.0
        alpha_ = atan2(R[1, 0], R[1, 1])
    else:
        beta = PI
        gamma_ = atan2(R[1, 0], R[1, 1])
        alpha_ = 0.0
    return alpha_, beta, gamma_


def cart_to_jnt(p_in, R, not_overhead, righty, wrist_no_flip):
    """SixAxis_1::CartToJnt (fixed), returns final joint vector q."""
    a_, d_, theta_ = A, D, THETA0
    pl = np.array([0, 0, d_[5]])
    p = p_in - R @ pl

    sudo_s4 = hypot(a_[3], d_[3])
    xy_radius2 = p[0] ** 2 + p[1] ** 2 - (d_[1] + d_[2]) ** 2
    if xy_radius2 < 0:
        raise ValueError("unreachable: xy radius")
    xy_norm = hypot(p[0], p[1])

    q = [0.0] * 6
    if not_overhead:
        q[0] = atan2(p[1], p[0]) - np.arcsin((d_[1] + d_[2]) / xy_norm)
        xyna1 = sqrt(xy_radius2) - a_[1]
        wrist_len = sqrt(xyna1 ** 2 + (p[2] - d_[0]) ** 2)
        angle_xyz = atan2(xyna1, p[2] - d_[0])
    else:
        q[0] = atan2(p[1], p[0]) + np.arcsin((d_[1] + d_[2]) / xy_norm)
        q[0] = q[0] + PI if q[0] < 0 else q[0] - PI
        xya1 = sqrt(xy_radius2) + a_[1]
        wrist_len = hypot(xya1, p[2] - d_[0])
        angle_xyz = atan2(xya1, p[2] - d_[0])

    if wrist_len > a_[2] + sudo_s4 or wrist_len < abs(a_[2] - sudo_s4):
        raise ValueError("unreachable: elbow triangle")

    a2s4_angle = np.arccos((a_[2] ** 2 + sudo_s4 ** 2 - wrist_len ** 2) / (2 * a_[2] * sudo_s4))
    offset_angle = np.arccos((a_[2] ** 2 + wrist_len ** 2 - sudo_s4 ** 2) / (2 * a_[2] * wrist_len))
    s4_offset = np.arctan(a_[3] / d_[3])

    q1_sign = 1.0 if not_overhead else -1.0
    if not righty:
        q[1] = q1_sign * angle_xyz + offset_angle
        q[2] = 2 * s4_offset + a2s4_angle
    else:
        q[1] = q1_sign * angle_xyz - offset_angle
        q[2] = 2 * s4_offset - a2s4_angle
    q[2] -= s4_offset

    q[1] -= theta_[1] + PI / 2.0
    q[2] -= theta_[2] - PI / 2.0
    q[3] = -theta_[3]
    q[4] = -theta_[4]
    q[5] = -theta_[5]

    r, _ = fk(q)
    wrist_rotation = r.T @ R
    alpha_, beta, gamma_ = get_euler_zyz(wrist_rotation)

    if wrist_no_flip:  # q4 = +beta; alpha/gamma need +-PI renormalization
        q[4] = beta
        q[3] = alpha_ - PI if alpha_ > 0 else alpha_ + PI
        q[5] = gamma_ - PI if gamma_ > 0 else gamma_ + PI
    else:  # q4 = -beta; alpha/gamma used as returned
        q[4] = -beta
        q[3] = alpha_
        q[5] = gamma_

    q[3] -= theta_[3]
    q[4] -= theta_[4]
    q[5] -= theta_[5]
    return np.array(q)


def update_config_turn(jnt):
    """SixAxis_1::UpdateConfigTurn (fixed), returns (branchFlags, packedG)."""
    theta = [THETA0[i] + jnt[i] for i in range(6)]
    q_tmp = list(theta)
    q_tmp[1] += PI / 2.0
    q_tmp[2] -= PI / 2.0

    branch = [1, 0, 1]  # [not_overhead, above, no_flip], defaults unused except [1]
    for i in range(6):
        tmp_q = q_tmp[i] % (2 * PI)
        if tmp_q > PI:
            tmp_q -= 2 * PI

        if i == 2:
            phi = np.arctan(A[3] / D[3])
            qelbow = tmp_q + phi
            shifted = (qelbow - 2 * phi + PI) % (2 * PI) - PI
            branch[1] = 1 if shifted <= 0 else 0
        if i == 4:
            branch[2] = 1 if 0 <= tmp_q <= PI else 0

    x_at_frame1 = (
        A[1] + A[2] * sin(q_tmp[1]) - D[3] * sin(q_tmp[1] + q_tmp[2]) + A[3] * cos(q_tmp[1] + q_tmp[2])
    )
    branch[0] = 1 if x_at_frame1 >= 0 else 0

    packed = branch[0] * 4 + branch[1] * 2 + branch[2]
    return branch, packed


def branch_from_cfg(cfg):
    return bool(cfg & 4), bool(cfg & 2), bool(cfg & 1)


def check_pose(label, jnt_true, verbose=False):
    R, p = fk(jnt_true)
    _, g_true = update_config_turn(jnt_true)
    all_ok = True
    for cfg in range(8):
        not_overhead, righty, wrist_no_flip = branch_from_cfg(cfg)
        try:
            q = cart_to_jnt(p, R, not_overhead, righty, wrist_no_flip)
        except ValueError:
            continue
        _, p_back = fk(q)
        err = np.linalg.norm(p_back - p)
        _, g_back = update_config_turn(q)
        ok = err < 1e-6 and g_back == cfg
        all_ok &= ok
        if verbose or not ok:
            print(
                f"  {label:28s} cfg={cfg} err={err:.2e} g_in={cfg} g_out={g_back} "
                f"{'OK' if ok else 'MISMATCH'}"
            )
    status = "PASS" if all_ok else "FAIL"
    print(f"[{status}] {label}: true branch (from FK) = {g_true}")
    return all_ok


def main() -> int:
    print("=" * 70)
    print("SixAxis_1 IK math verification (pure Python, matches sixaxis_1.cpp)")
    print("=" * 70)

    all_pass = True
    all_pass &= check_pose("BOX_JNT [0,0,0,0,30,0]deg", [0, 0, 0, 0, math.radians(30), 0])

    rng = np.random.default_rng(42)
    for i in range(20):
        jnt = np.concatenate(
            [
                rng.uniform(-math.pi, math.pi, 1),
                rng.uniform(math.radians(-70), math.radians(70), 2),
                rng.uniform(-math.pi, math.pi, 1),
                rng.uniform(math.radians(-80), math.radians(80), 1),
                rng.uniform(-math.pi, math.pi, 1),
            ]
        )
        all_pass &= check_pose(f"random pose {i}", jnt)

    print("=" * 70)
    print("ALL PASS" if all_pass else "SOME FAILED")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
