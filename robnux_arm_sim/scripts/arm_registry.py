#!/usr/bin/env python3
"""
Shared registry of every robot arm topology under robnux_arm_sim, for use by
the FK/IK and trajectory test scripts.

Each ArmSpec bundles exactly what's needed to construct an `m.Robot(...)`
matching the corresponding simulate_*.py demo script (same PARA/DH layout,
same Profile numbers) plus test-specific metadata:
  - joint_bounds: safe sampling range per joint (drawn from the URDF <limit>
    tags in robnux_arm_sim/urdf/*.xacro, shrunk a bit off the hard stops)
  - singular_guards: (joint_index, eps) pairs -- redraw that joint if it
    lands within `eps` of 0, since 0 is a wrist/shoulder singularity for
    every spherical-wrist arm here (the concurrent wrist's middle joint)
  - pos_tol / rot_tol: FK(IK(pose)) round-trip tolerance

This file has no ROS-execution side effects; importing it just builds numpy
parameter arrays. `rob_motion_commands` is only imported (for m.Profile) by
scripts that call build_robot(), not by consumers that just want the specs.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

import robot_model_loader

PI = math.pi
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T


@dataclass
class ArmSpec:
    name: str
    robname: str
    dof: int
    para: np.ndarray
    profile_args: Tuple[float, float, float, float, float, float]
    joint_bounds: List[Tuple[float, float]]
    singular_guards: List[Tuple[int, float]] = field(default_factory=list)
    # InverseKin's output buffer must be sized by the robot's *actuator*
    # count (CreateRobot::DoF_, i.e. armMap_->GetActDoF()), not by `dof`
    # above (which is the Cartesian/meaningful-joint count used for FK
    # sampling). These differ only for over-actuated parallel mechanisms
    # like Quattro (4 actuators, 3 Cartesian DOF) -- passing `dof` there
    # used to abort the process (Eigen::Ref resize assertion) inside
    # CreateRobot::InverseKin. None -> same as `dof`.
    ik_dof: Optional[int] = None
    pos_tol: float = 1e-6
    rot_tol: float = 1e-6
    # Fraction of samples IK is allowed to legitimately refuse (real
    # singularities near the sampled joint-space region), overriding the
    # test script's global default. None -> use the script's default.
    ik_fail_tolerance: Optional[float] = None
    notes: str = ""

    def build_robot(self, m):
        pf = m.Profile(*self.profile_args)
        return m.Robot(self.robname, self.para, BASE_OFF, BASE_OFF, pf)


def _make_poe_params(axes, tl_pos, tool_offset):
    """Pack axes / T_left / T_right into the 35*N column vector (same layout
    as each simulate_poe_*.py's _make_poe_params)."""
    N = len(axes)
    p = np.zeros(35 * N)
    for i, ax in enumerate(axes):
        p[3 * i: 3 * (i + 1)] = ax
    base = 3 * N
    for i, t in enumerate(tl_pos):
        T = np.eye(4)
        T[:3, 3] = t
        p[base + 16 * i: base + 16 * (i + 1)] = T.flatten()
    base += 16 * N
    for i in range(N):
        T = np.eye(4)
        if i == N - 1:
            T[:3, 3] = tool_offset
        p[base + 16 * i: base + 16 * (i + 1)] = T.flatten()
    return p.reshape(-1, 1)


def _scara_spec() -> ArmSpec:
    ALPHA = [0.0, 0.0, 0.0, 0.0]
    A = [0.0, 0.30, 0.20, 0.0]
    THETA = [0.0, 0.0, 0.0, 0.0]
    D = [0.30, 0.0, 0.0, -0.05]
    para = np.array([ALPHA + A + THETA + D]).T
    return ArmSpec(
        name="scara", robname="scara", dof=4, para=para,
        profile_args=(0.3, 1.0, 10.0, 0.3, 1.0, 10.0),
        # joint2 is prismatic z-stroke (URDF limit -0.15..0.05); rest revolute
        joint_bounds=[(-PI * 0.9, PI * 0.9), (-PI * 0.9, PI * 0.9),
                      (-0.14, 0.04), (-PI * 0.9, PI * 0.9)],
    )


def _quattro_spec() -> ArmSpec:
    para = np.array([[0.3, -PI / 2.0, 0.4, 0.0, 1.02, 0.1, 0.0, 0.18 * math.sqrt(2)]]).T
    return ArmSpec(
        name="quattro", robname="quattro", dof=3, para=para,
        profile_args=(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0),
        # actuator angle URDF limit is -0.5..1.1 rad; stay off the hard stops
        joint_bounds=[(-0.4, 1.0)] * 3,
        # Quattro is over-actuated: 4 physical legs (A_DoF_) drive 3
        # Cartesian DOF; FK only reads 3 of them, but IK returns all 4.
        ik_dof=4,
    )


def _quattro4_spec() -> ArmSpec:
    para = np.array([[
        0.3, -PI / 2.0, 0.4, 0.0, 1.02, 0.1,
        0.0, 0.0,
        0.12, 0.12,
    ]]).T
    return ArmSpec(
        name="quattro_4", robname="quattro_4", dof=4, para=para,
        profile_args=(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0),
        joint_bounds=[(-0.4, 1.0)] * 4,
    )


def _xyz_gantry_spec() -> ArmSpec:
    ALPHA = [0.0, PI / 2, -PI / 2]
    A = [0.0, 0.0, 0.0]
    THETA = [0.0, 0.0, 0.0]
    D = [0.0, 0.0, 0.0]
    para = np.array([ALPHA + A + THETA + D]).T
    return ArmSpec(
        name="xyz_gantry", robname="xyz_gantry", dof=3, para=para,
        profile_args=(0.1, 0.5, 5.0, 0.1, 0.5, 5.0),
        # RAIL_Z=0.7 -> z in [-0.56,0]; RAIL_X=RAIL_Y=1.2 -> +-0.48, trimmed
        joint_bounds=[(-0.45, -0.02), (-0.4, 0.4), (-0.4, 0.4)],
    )


def _xyz_ur_spec() -> ArmSpec:
    # UJNT tail alpha=[0,-PI/2]: at q3=q4=0, joint3 axis=world Z (pan),
    # joint4 axis=world Y (tilt) -- must match UJNT::CartToJnt's derivation.
    ALPHA = [0.0, PI / 2, -PI / 2, 0.0, -PI / 2]
    A = [0.0, 0.0, 0.0, 0.0, 0.0]
    THETA = [0.0, 0.0, 0.0, 0.0, 0.0]
    D = [0.0, 0.0, 0.0, 0.05, 0.05]
    para = np.array([ALPHA + A + THETA + D]).T
    return ArmSpec(
        name="xyz_ur", robname="xyz_ur", dof=5, para=para,
        profile_args=(0.1, 0.5, 5.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-0.45, -0.02), (-0.4, 0.4), (-0.4, 0.4),
                      (-1.3, 1.3), (-1.3, 1.3)],
    )


def _sixaxis_spec() -> ArmSpec:
    ALPHA = [0, -PI / 2, 0, -PI / 2, PI / 2, -PI / 2]
    A = [0, 0.075, 0.25, 0.03, 0, 0]
    THETA = [0, PI / 2, 0, 0, 0, 0]
    D = [0.33, 0, 0, 0.25, 0, 0.09]
    para = np.array([ALPHA + A + THETA + D]).T
    return ArmSpec(
        name="sixaxis_1", robname="sixaxis_1", dof=6, para=para,
        profile_args=(0.05, 0.2, 2.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-PI * 0.9, PI * 0.9)] * 4 + [(-PI * 0.7, PI * 0.7)] + [(-PI * 0.9, PI * 0.9)],
        # q(4) is the wrist-bend joint; 0 is the spherical-wrist singularity
        singular_guards=[(4, 0.15)],
        # a_[1]=0.075 is a small shoulder offset, so a non-trivial fraction of
        # the sampled joint volume legitimately lands within SIXDOF_HEADDIST
        # (5cm) of the base Z-axis (~10-17% empirically) -- a real azimuth
        # singularity, not a bug.
        ik_fail_tolerance=0.25,
        notes="see project_sixaxis_ik_bugs memory: FK/IK is exact post-fix",
    )


def _scara6020_spec() -> ArmSpec:
    # Real robot_model/6020_scara.txt DH data (not a synthetic demo geometry)
    # -- see robot_model_loader.py for how the flat file maps onto
    # [alpha|a|theta|d]. a=[0,0.325,0.275,0] lands exactly on scara.cpp's own
    # a_[1]/a_[2] usage; d=[0,0,0,0] means no base-height/tool offset is
    # baked into this particular file (shoulder axis sits at world Z=0).
    para = robot_model_loader.build_para("6020_scara.txt", 4)
    return ArmSpec(
        name="scara_6020", robname="scara", dof=4, para=para,
        profile_args=(0.3, 1.0, 10.0, 0.3, 1.0, 10.0),
        joint_bounds=[(-PI * 0.9, PI * 0.9), (-PI * 0.9, PI * 0.9),
                      (-0.14, 0.04), (-PI * 0.9, PI * 0.9)],
        notes="real 6020 SCARA model (325mm+275mm reach) for further "
              "regression-testing scara.cpp beyond the synthetic demo geometry",
    )


def _rv2fr_spec() -> ArmSpec:
    # Real robot_model/RV_2FR_D.txt DH data (Mitsubishi RV-2FR-D 6-axis arm)
    # -- 25 values = 4*6 core [alpha|a|theta|d] + 1 trailing tool length,
    # folded into d[5] by robot_model_loader (matching sixaxis_1.cpp's own
    # use of d_[5] as the wrist-to-flange offset). Unlike the synthetic demo
    # sixaxis_1 geometry, a_[1]=0 here (shoulder axis intersects the waist
    # axis), so the near-base-axis (SIXDOF_HEADDIST) singularity is *more*
    # prevalent under random sampling -- see ik_fail_tolerance below.
    para = robot_model_loader.build_para("RV_2FR_D.txt", 6)
    return ArmSpec(
        name="rv2fr", robname="sixaxis_1", dof=6, para=para,
        profile_args=(0.05, 0.2, 2.0, 0.1, 0.5, 5.0),
        joint_bounds=[(-PI * 0.9, PI * 0.9)] * 4 + [(-PI * 0.7, PI * 0.7)] + [(-PI * 0.9, PI * 0.9)],
        singular_guards=[(4, 0.15)],
        ik_fail_tolerance=0.35,
        notes="real Mitsubishi RV-2FR-D model for further regression-testing "
              "sixaxis_1.cpp beyond the synthetic demo geometry",
    )


def _poe6_spec(name, axes, tl_pos, tool_offset, joint_bounds) -> ArmSpec:
    para = _make_poe_params(axes, tl_pos, tool_offset)
    return ArmSpec(
        name=name, robname="poe_arm_6r", dof=6, para=para,
        profile_args=(0.3, 1.0, 10.0, 0.3, 1.0, 10.0),
        joint_bounds=joint_bounds,
        # axes[3:6] are the concurrent spherical wrist; axes[4] is the middle
        # (bend) axis -- 0 there is the wrist singularity.
        singular_guards=[(4, 0.15)],
    )


def _poe_ur5_spec() -> ArmSpec:
    axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, -1], [0, 1, 0]]
    tl_pos = [
        [0, 0, 0.0892], [0, 0.1358, 0], [0.4250, 0, 0],
        [0.3922, 0, 0], [0, 0.1091, 0], [0, 0, 0.0946],
    ]
    tool_offset = [0, 0, 0.0823]
    bounds = [(-3.0, 3.0)] * 6
    return _poe6_spec("poe_ur5", axes, tl_pos, tool_offset, bounds)


def _poe_kuka_spec() -> ArmSpec:
    axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1]]
    tl_pos = [
        [0, 0, 0.400], [0.025, 0, 0], [0, 0, 0.490],
        [0, 0, 0.430], [0, 0, 0], [0, 0, 0],
    ]
    tool_offset = [0, 0, 0.080]
    bounds = [(-2.9, 2.9), (-2.0, 2.0), (-2.0, 2.0), (-3.1, 3.1), (-2.0, 2.0), (-3.1, 3.1)]
    return _poe6_spec("poe_kuka", axes, tl_pos, tool_offset, bounds)


def _poe_lite6_spec() -> ArmSpec:
    axes = [[0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0]]
    tl_pos = [
        [0, 0, 0.300], [0, 0, 0], [0, 0, 0.270],
        [0.280, 0, 0], [0, 0, 0], [0, 0, 0],
    ]
    tool_offset = [0, 0, 0.070]
    bounds = [(-3.0, 3.0), (-2.5, 2.5), (-3.0, 3.0), (-2.5, 2.5), (-3.0, 3.0), (-2.5, 2.5)]
    return _poe6_spec("poe_lite6", axes, tl_pos, tool_offset, bounds)


def _poe7_spec(name, axes, tl_pos, tool_offset, joint_bounds) -> ArmSpec:
    para = _make_poe_params(axes, tl_pos, tool_offset)
    return ArmSpec(
        name=name, robname="poe_arm_7r", dof=7, para=para,
        profile_args=(0.3, 1.0, 10.0, 0.3, 1.0, 10.0),
        joint_bounds=joint_bounds,
        # axes[4:7] are the concurrent wrist; axes[5] is the middle (bend) axis
        singular_guards=[(5, 0.15)],
    )


def _poe_iiwa_spec() -> ArmSpec:
    axes = [[0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1]]
    tl_pos = [
        [0, 0, 0.340], [0, 0, 0], [0, 0, 0], [0, 0, 0.400],
        [0, 0, 0.400], [0, 0, 0], [0, 0, 0],
    ]
    tool_offset = [0, 0, 0.126]
    bounds = [(-2.9, 2.9), (-2.0, 2.0), (-2.9, 2.9), (-2.0, 2.0), (-2.9, 2.9), (-2.0, 2.0), (-3.0, 3.0)]
    return _poe7_spec("poe_iiwa", axes, tl_pos, tool_offset, bounds)


def _poe_panda_spec() -> ArmSpec:
    axes = [[0, 0, 1], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, 1], [0, 1, 0], [0, 0, 1]]
    tl_pos = [
        [0, 0, 0.333], [0, 0, 0], [0, 0, 0.316], [0.0825, 0, 0],
        [-0.0825, 0, 0.384], [0, 0, 0], [0.088, 0, 0],
    ]
    tool_offset = [0, 0, 0.107]
    bounds = [(-2.8, 2.8), (-1.7, 1.7), (-2.8, 2.8), (-3.0, -0.1),
              (-2.8, 2.8), (0.05, 3.7), (-2.8, 2.8)]
    return _poe7_spec("poe_panda", axes, tl_pos, tool_offset, bounds)


def build_all_specs() -> List[ArmSpec]:
    return [
        _scara_spec(),
        _quattro_spec(),
        _quattro4_spec(),
        _xyz_gantry_spec(),
        _xyz_ur_spec(),
        _sixaxis_spec(),
        _scara6020_spec(),
        _rv2fr_spec(),
        _poe_ur5_spec(),
        _poe_kuka_spec(),
        _poe_lite6_spec(),
        _poe_iiwa_spec(),
        _poe_panda_spec(),
    ]


def spec_by_name(name: str) -> ArmSpec:
    for s in build_all_specs():
        if s.name == name:
            return s
    raise KeyError(f"no ArmSpec named {name!r}")
