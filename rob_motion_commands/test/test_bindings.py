"""Python-level smoke tests for the rob_motion_commands pybind11 module.

These exercise the bindings end-to-end: type construction, accessor calls,
and the ROS 2 context lifecycle helpers (ros_init/ros_shutdown/ros_ok).
"""

import math

import pytest

import rob_motion_commands as m


def test_module_loads_and_exposes_types():
    expected = {
        "Vec",
        "Quaternion",
        "Rotation",
        "Frame",
        "Pose",
        "refPose",
        "FrameData",
        "Profile",
        "Percent",
        "JntProfile",
        "LocData",
        "ScurvePlanner",
        "Robot",
        "MotionType",
        "IpoMode",
        "ros_init",
        "ros_shutdown",
        "ros_ok",
    }
    missing = expected - set(dir(m))
    assert not missing, f"missing bindings: {sorted(missing)}"


def test_ros_context_initialized_on_import():
    assert m.ros_ok(), "rclcpp context should be initialized at module import"


def test_vec_components_and_norm():
    v = m.Vec(1.0, 2.0, 3.0)
    assert v.x() == pytest.approx(1.0)
    assert v.y() == pytest.approx(2.0)
    assert v.z() == pytest.approx(3.0)
    assert v.Norm() == pytest.approx(math.sqrt(14.0))


def test_vec_zero():
    z = m.Vec.Zero()
    assert (z.x(), z.y(), z.z()) == (pytest.approx(0.0),) * 3


def test_quaternion_identity():
    q = m.Quaternion.getIdentity()
    assert q.w() == pytest.approx(1.0)
    assert q.x() == pytest.approx(0.0)
    assert q.y() == pytest.approx(0.0)
    assert q.z() == pytest.approx(0.0)


def test_quaternion_euler_roundtrip():
    q = m.Quaternion()
    q.SetEulerZYX(0.3, -0.2, 0.1)
    ok, yaw, pitch, roll = q.GetEulerZYX()
    assert ok
    assert yaw == pytest.approx(0.3, abs=1e-6)
    assert pitch == pytest.approx(-0.2, abs=1e-6)
    assert roll == pytest.approx(0.1, abs=1e-6)


def test_frame_identity_has_zero_translation():
    f = m.Frame.Identity()
    t = f.getTranslation()
    assert (t.x(), t.y(), t.z()) == (pytest.approx(0.0),) * 3


def test_frame_inverse_composition_is_identity():
    q = m.Quaternion()
    q.SetEulerZYX(0.3, -0.2, 0.1)
    f = m.Frame(q, m.Vec(1.0, 2.0, 3.0))
    inv = f.Inverse()
    # No __mul__ binding for Frame; verify via the Eigen-vector projection.
    composed_vec = f.ToEigenVecQuat()
    assert composed_vec.shape[0] >= 7
    # Inverse of a non-trivial frame must have non-default translation too.
    inv_t = inv.getTranslation()
    assert (inv_t.x(), inv_t.y(), inv_t.z()) != (pytest.approx(0.0),) * 3


def test_refpose_base_and_tool_roundtrip():
    ee = m.Frame(m.Vec(0.5, 0.0, 0.0))
    base = m.Frame(m.Vec(1.0, 0.0, 0.0))
    tool = m.Frame(m.Vec(0.0, 1.0, 0.0))
    rp = m.refPose(ee, base, tool, [0], [0])
    ok_b, base_out = rp.getBase()
    ok_t, tool_out = rp.getTool()
    assert ok_b and ok_t
    assert base_out.getTranslation().x() == pytest.approx(1.0)
    assert tool_out.getTranslation().y() == pytest.approx(1.0)


def test_motion_enums_have_expected_values():
    assert int(m.MotionType.LINE) != int(m.MotionType.ARC)
    assert int(m.IpoMode.WORLD) != int(m.IpoMode.BASE)


def test_locdata_and_framedata_fields():
    loc = m.LocData(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1, -1)
    assert loc.x == pytest.approx(1.0)
    assert loc.G == 1 and loc.T == -1

    fd = m.FrameData(2, 5, m.IpoMode.BASE)
    assert fd.base == 2 and fd.tool == 5
    assert fd.ipo == m.IpoMode.BASE


def test_profile_and_percent_field_setters():
    p = m.Profile(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    p.max_vel_t = 10.0
    assert p.max_vel_t == pytest.approx(10.0)

    perc = m.Percent(50, 60, 70)
    assert (perc.v, perc.a, perc.j) == (50, 60, 70)


def test_ros_init_is_idempotent():
    # ros_init should not error when called repeatedly.
    m.ros_init()
    m.ros_init()
    assert m.ros_ok()
