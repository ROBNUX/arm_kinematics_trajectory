#!/usr/bin/env python3
"""
Loader for the flat DH parameter files under arm_kinematics_trajectory/
robot_model/. Each file is a plain list of numbers (one per line), laid out
as:

    [alpha_0 .. alpha_{DoF-1},
     a_0     .. a_{DoF-1},
     theta_0 .. theta_{DoF-1},
     d_0     .. d_{DoF-1}]         (4*DoF values -- serialArm::SetGeometry's
                                     own [alpha|a|theta|d] layout, DoF each)

optionally followed by exactly one trailing scalar: a wrist-to-flange tool
length, added onto d[DoF-1] (matching sixaxis_1.cpp's use of d_[5] as the
tool offset -- see CartToJnt's `Vec pl(0, 0, d_[5])`).

This split was confirmed, not guessed: reading 4*DoF core values lines up
exactly with how scara.cpp/sixaxis_1.cpp index into a_[]/d_[] (e.g.
scara.cpp's `a_[1]`/`a_[2]` land precisely on the 325mm/275mm link lengths
in 6020_scara.txt when the core is split at exactly DoF-sized boundaries).
"""
import os

import numpy as np

ROBOT_MODEL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "robot_model"))


def load_dh(filename: str, dof: int):
    """Returns (alpha, a, theta, d) numpy arrays, each length `dof`, parsed
    from robot_model/<filename>. Any trailing scalar beyond the 4*dof core
    values is folded into d[-1] as a tool length."""
    path = os.path.join(ROBOT_MODEL_DIR, filename)
    vals = np.loadtxt(path)
    core = 4 * dof
    if vals.size < core:
        raise ValueError(f"{path}: only {vals.size} values, need >= {core} "
                          f"(4 * dof={dof})")
    alpha = vals[0:dof].copy()
    a = vals[dof:2 * dof].copy()
    theta = vals[2 * dof:3 * dof].copy()
    d = vals[3 * dof:4 * dof].copy()
    extra = vals[4 * dof:]
    if extra.size == 1:
        # Confirmed convention for RV_2FR_D.txt (6-DOF): a lone trailing
        # scalar is the wrist-to-flange tool length, landing on d_[DoF-1] --
        # exactly the slot sixaxis_1.cpp itself treats as the tool offset.
        d[-1] += extra[0]
    elif extra.size > 1 and np.any(extra != 0.0):
        raise ValueError(f"{path}: {extra.size} nonzero trailing values "
                          f"beyond the 4*dof={core} core -- unrecognized "
                          "layout, don't know how to interpret these "
                          f"(got {extra})")
    elif extra.size > 1:
        # 6020_scara.txt has 4 trailing zeros (dof=4): plausibly a spare
        # tool-frame DH row [alpha,a,theta,d] that happens to be identity,
        # but that's unconfirmed. Since they're all exactly 0 either way
        # it's a no-op to skip them -- don't guess a semantic assignment for
        # a shape we haven't seen a nonzero example of.
        pass
    return alpha, a, theta, d


def build_para(filename: str, dof: int) -> np.ndarray:
    """[alpha|a|theta|d] column vector, ready for m.Robot(...)."""
    alpha, a, theta, d = load_dh(filename, dof)
    return np.array([np.concatenate([alpha, a, theta, d])]).T
