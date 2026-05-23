#!/usr/bin/env python3
"""Example 02 — Pose / refPose and motion-command data types.

Shows how refPose composes a Frame with non-default base / tool frames, and
how the motion-command value types (LocData, FrameData, ProfileData,
JntProfile, Percent) interoperate.
"""

import rob_motion_commands as m


def main() -> int:
    print("=" * 60)
    print("Pose / refPose / motion data types — rob_motion_commands")
    print("=" * 60)

    # ----------------------------------------------- refPose construction
    ee = m.Frame(m.Vec(0.5, 0.0, 0.0))
    base = m.Frame(m.Vec(1.0, 0.0, 0.0))
    tool = m.Frame(m.Vec(0.0, 1.0, 0.0))
    rp = m.refPose(ee, base, tool, [0], [0])

    ok_b, base_out = rp.getBase()
    ok_t, tool_out = rp.getTool()
    bt = base_out.getTranslation()
    tt = tool_out.getTranslation()
    print(f"\nrefPose base translation: ({bt.x():.3f}, {bt.y():.3f}, "
          f"{bt.z():.3f})  ok={ok_b}")
    print(f"refPose tool translation: ({tt.x():.3f}, {tt.y():.3f}, "
          f"{tt.z():.3f})  ok={ok_t}")

    # ---------------------------------------- motion-command data classes
    loc = m.LocData(0.5, 0.1, 0.2, 0.0, 0.0, 0.0, 1, 0)
    print(f"\nLocData: x={loc.x}, y={loc.y}, z={loc.z}, "
          f"branch={loc.G}, turn={loc.T}")

    fd = m.FrameData(1, 2, m.IpoMode.BASE)
    print(f"FrameData: base={fd.base}, tool={fd.tool}, ipo={fd.ipo}")

    pf = m.Profile(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    print(f"Profile.max_vel_t={pf.max_vel_t}, max_acc_r={pf.max_acc_r}")

    jp = m.JntProfile(2.0, 4.0, 8.0)
    print(f"JntProfile: vel={jp.max_vel}, acc={jp.max_acc}, "
          f"jerk={jp.max_jerk}")

    pct = m.Percent(50, 70, 90)
    print(f"Percent: v={pct.v}%, a={pct.a}%, j={pct.j}%")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
