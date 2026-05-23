#!/usr/bin/env python3
"""Example 01 — geometric primitives from rob_motion_commands.

Demonstrates Vec / Quaternion / Rotation / Frame round-trip operations.
Run from a sourced workspace:

    python3 examples/01_geometric_types.py
"""

import math

import rob_motion_commands as m


def main() -> int:
    print("=" * 60)
    print("Geometric primitives — rob_motion_commands")
    print("=" * 60)

    # ---------------------------------------------------------------- Vec
    v = m.Vec(1.0, 2.0, 3.0)
    print(f"\nVec({v.x():.3f}, {v.y():.3f}, {v.z():.3f}); norm = {v.Norm():.4f}")
    print(f"Vec.Zero() == ({m.Vec.Zero().x()}, {m.Vec.Zero().y()}, "
          f"{m.Vec.Zero().z()})")

    # --------------------------------------------------------- Quaternion
    q = m.Quaternion()
    q.SetEulerZYX(math.radians(30), math.radians(20), math.radians(10))
    ok, yaw, pitch, roll = q.GetEulerZYX()
    print(f"\nQuaternion ZYX round-trip (degrees): "
          f"({math.degrees(yaw):.3f}, {math.degrees(pitch):.3f}, "
          f"{math.degrees(roll):.3f})  ok={ok}")
    print(f"Quaternion(w,x,y,z) = ({q.w():.4f}, {q.x():.4f}, {q.y():.4f}, "
          f"{q.z():.4f})")

    # ----------------------------------------------------------- Rotation
    r = m.Rotation(math.radians(45), 0.0, 0.0)
    ok2, y2, p2, r2 = r.GetEulerZYX()
    print(f"\nRotation(yaw=45°): yaw read back = {math.degrees(y2):.3f}°")

    # ------------------------------------------------------------- Frame
    ee = m.Frame(q, v)
    t = ee.getTranslation()
    print(f"\nFrame translation:  ({t.x():.3f}, {t.y():.3f}, {t.z():.3f})")
    inv = ee.Inverse()
    inv_t = inv.getTranslation()
    print(f"Frame.Inverse() t:  ({inv_t.x():.3f}, {inv_t.y():.3f}, "
          f"{inv_t.z():.3f})")
    print(f"DH(a=0.5, alpha=π/2, d=0.1, theta=0) translation: "
          f"{m.Frame.DH(0.5, math.pi / 2, 0.1, 0.0).getTranslation().x():.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
