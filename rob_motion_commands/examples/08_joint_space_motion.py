#!/usr/bin/env python3
"""Example 08 — joint-space motion: MovePTPJ and SetJntProfile.

Demonstrates:
  - Setting per-axis JntProfile limits (different limits for each joint).
  - MovePTPJ to move directly to a joint-space target (bypasses IK on the
    command path — the trajectory planner works in joint space).
  - GetCartFromJnt to inspect the resulting Cartesian position after each move.
  - GetSpeed / SetSpeed to query and change speed scaling on the fly.

Run from a sourced ROS 2 workspace:

    python3 examples/08_joint_space_motion.py
"""

import math
import sys
import time

import numpy as np

import rob_motion_commands as m

PARA = np.array([[0.3, -math.pi / 2.0, 0.4, 0.0, 1.02, 0.1, 0.0,
                  0.18 * math.sqrt(2)]]).T
BASE_OFF = np.array([[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
DOF = 4


def wait_done(rob: m.Robot, poll_s: float = 0.5) -> None:
    while not rob.MotionDone():
        time.sleep(poll_s)


def print_cart(rob: m.Robot, label: str, jnt: np.ndarray) -> None:
    ok, cart = rob.GetCartFromJnt(jnt, 7)
    if ok:
        print(f"  {label}: x={cart[0]:.4f} y={cart[1]:.4f} z={cart[2]:.4f}")
    else:
        print(f"  {label}: GetCartFromJnt failed")


def main() -> int:
    print("=" * 60)
    print("Joint-space motion — MovePTPJ / SetJntProfile")
    print("=" * 60)

    # Cartesian profile (required for Robot construction)
    pf = m.Profile(2.0, 50.0, 1500.0, 2.0, 50.0, 1500.0)
    rob = m.Robot("quattro", PARA, BASE_OFF, BASE_OFF, pf)

    home_jnt = np.zeros((DOF, 1))
    rob.SetFeedback(home_jnt)

    # -----------------------------------------------------------------
    # Assign per-axis joint profiles (different limits per joint)
    # -----------------------------------------------------------------
    joint_profiles = [
        m.JntProfile(20.0, 50.0, 200.0),   # joint 0 — nominal
        m.JntProfile(15.0, 40.0, 160.0),   # joint 1 — slightly slower
        m.JntProfile(20.0, 50.0, 200.0),   # joint 2 — nominal
        m.JntProfile(10.0, 30.0, 120.0),   # joint 3 — most restricted
    ]
    for i, jpf in enumerate(joint_profiles):
        rob.SetJntProfile(i, jpf)
        print(f"Joint {i}: vel={jpf.max_vel} rad/s  "
              f"acc={jpf.max_acc} rad/s²  jerk={jpf.max_jerk} rad/s³")

    rob.StartMotion()

    # -----------------------------------------------------------------
    # Speed scaling — show GetSpeed, then change it
    # -----------------------------------------------------------------
    default_spd = rob.GetSpeed()
    print(f"\nDefault speed: v={default_spd.v}%  a={default_spd.a}%  "
          f"j={default_spd.j}%")
    rob.SetSpeed(m.Percent(60, 60, 60))
    print("Speed set to 60 % for first half …")

    # -----------------------------------------------------------------
    # Joint-space targets
    # -----------------------------------------------------------------
    deg = math.radians
    targets = [
        np.array([[ deg(10), deg(-10), deg(10), deg(-10)]]).T,
        np.array([[ deg(20), deg(-20), deg(20), deg(-20)]]).T,
        np.array([[ deg(30),  deg(30), deg(30),  deg(30)]]).T,
        np.array([[-deg(10), deg(-15), deg(5),  deg(-5)]]).T,
        np.zeros((DOF, 1)),   # return to home
    ]

    try:
        print_cart(rob, "home", home_jnt)

        for idx, jnt_target in enumerate(targets):
            print(f"\n--- MovePTPJ target {idx}: "
                  f"{np.degrees(jnt_target.flatten()).round(1)} deg ---")

            # Ramp up speed at the midpoint
            if idx == len(targets) // 2:
                rob.SetSpeed(m.Percent(90, 90, 90))
                print("  Speed raised to 90 %")

            rob.MovePTPJ(jnt_target, 0)
            wait_done(rob)
            print_cart(rob, f"after target {idx}", jnt_target)

        print("\nAll joint-space motions complete.")

    except RuntimeError as e:
        rob.Shutdown()
        if "SIGNAL" in str(e):
            code = int(str(e).rsplit(" ", 1)[1])
            sys.exit(128 + code)
        raise

    rob.Shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
