// C++ smoke tests for the geometric and motion data types that
// rob_motion_commands binds to Python. These exercise the underlying
// robnux_kdl_common / robnux_trajectory classes directly so that a regression
// in the C++ libraries is caught before it surfaces through the bindings.

#include <gtest/gtest.h>

#include <cmath>

#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_trajectory/move_command.hpp"

namespace {

constexpr double kTol = 1e-9;

TEST(VecTest, ComponentConstructorAndAccessors) {
  kinematics_lib::Vec v(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(v.x(), 1.0);
  EXPECT_DOUBLE_EQ(v.y(), 2.0);
  EXPECT_DOUBLE_EQ(v.z(), 3.0);
  EXPECT_NEAR(v.Norm(), std::sqrt(14.0), kTol);
}

TEST(VecTest, ZeroIsZero) {
  kinematics_lib::Vec z = kinematics_lib::Vec::Zero();
  EXPECT_DOUBLE_EQ(z.x(), 0.0);
  EXPECT_DOUBLE_EQ(z.y(), 0.0);
  EXPECT_DOUBLE_EQ(z.z(), 0.0);
}

TEST(VecTest, EigenRoundTrip) {
  kinematics_lib::Vec v(0.5, -1.5, 2.5);
  Eigen::Vector3d e = v.ToEigenVec();
  kinematics_lib::Vec back = kinematics_lib::Vec::FromEigenVec(e);
  EXPECT_NEAR(back.x(), 0.5, kTol);
  EXPECT_NEAR(back.y(), -1.5, kTol);
  EXPECT_NEAR(back.z(), 2.5, kTol);
}

TEST(QuaternionTest, IdentityIsUnitW) {
  kinematics_lib::Quaternion q = kinematics_lib::Quaternion::getIdentity();
  EXPECT_NEAR(q.w(), 1.0, kTol);
  EXPECT_NEAR(q.x(), 0.0, kTol);
  EXPECT_NEAR(q.y(), 0.0, kTol);
  EXPECT_NEAR(q.z(), 0.0, kTol);
}

TEST(QuaternionTest, EulerRoundTrip) {
  kinematics_lib::Quaternion q;
  const double yaw_in = 0.3;
  const double pitch_in = -0.2;
  const double roll_in = 0.1;
  q.SetEulerZYX(yaw_in, pitch_in, roll_in);

  double yaw = 0;
  double pitch = 0;
  double roll = 0;
  ASSERT_TRUE(q.GetEulerZYX(&yaw, &pitch, &roll));
  EXPECT_NEAR(yaw, yaw_in, 1e-6);
  EXPECT_NEAR(pitch, pitch_in, 1e-6);
  EXPECT_NEAR(roll, roll_in, 1e-6);
}

TEST(FrameTest, IdentityHasZeroTranslation) {
  kinematics_lib::Frame f = kinematics_lib::Frame::Identity();
  kinematics_lib::Vec p = f.getTranslation();
  EXPECT_NEAR(p.x(), 0.0, kTol);
  EXPECT_NEAR(p.y(), 0.0, kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(FrameTest, ComposeWithInverseGivesIdentity) {
  kinematics_lib::Vec v(1.0, 2.0, 3.0);
  kinematics_lib::Quaternion q;
  q.SetEulerZYX(0.3, -0.2, 0.1);
  kinematics_lib::Frame f(q, v);
  kinematics_lib::Frame inv = f.Inverse();
  kinematics_lib::Frame id = f * inv;
  kinematics_lib::Vec t = id.getTranslation();
  EXPECT_NEAR(t.x(), 0.0, 1e-9);
  EXPECT_NEAR(t.y(), 0.0, 1e-9);
  EXPECT_NEAR(t.z(), 0.0, 1e-9);
}

TEST(PoseTest, DefaultConstructible) {
  kinematics_lib::Pose p;
  std::vector<int> branch;
  std::vector<int> turns;
  EXPECT_TRUE(p.getBranchFlags(&branch));
  EXPECT_TRUE(p.getJointTurns(&turns));
}

TEST(PoseTest, FrameAccessor) {
  kinematics_lib::Vec v(0.1, 0.2, 0.3);
  kinematics_lib::Frame ee(v);
  kinematics_lib::Pose p(ee);
  kinematics_lib::Frame back;
  ASSERT_TRUE(p.getFrame(&back));
  EXPECT_NEAR(back.getTranslation().x(), 0.1, kTol);
}

TEST(RefPoseTest, BaseAndToolRoundTrip) {
  kinematics_lib::Frame ee(kinematics_lib::Vec(0.5, 0.0, 0.0));
  kinematics_lib::Frame base(kinematics_lib::Vec(1.0, 0.0, 0.0));
  kinematics_lib::Frame tool(kinematics_lib::Vec(0.0, 1.0, 0.0));
  std::vector<int> branch = {0};
  std::vector<int> turns = {0};
  kinematics_lib::refPose rp(ee, base, tool, branch, turns);

  kinematics_lib::Frame base_out;
  kinematics_lib::Frame tool_out;
  ASSERT_TRUE(rp.getBase(&base_out));
  ASSERT_TRUE(rp.getTool(&tool_out));
  EXPECT_NEAR(base_out.getTranslation().x(), 1.0, kTol);
  EXPECT_NEAR(tool_out.getTranslation().y(), 1.0, kTol);
}

TEST(MotionDataTest, FrameDataConstructorAssignsFields) {
  kinematics_lib::FrameData fd(2, 5, kinematics_lib::IPO_MODE::ID_BASE);
  EXPECT_EQ(fd.baseNo_, 2u);
  EXPECT_EQ(fd.toolNo_, 5u);
  EXPECT_EQ(fd.ipo_, kinematics_lib::IPO_MODE::ID_BASE);
}

TEST(MotionDataTest, ProfilePercentAssignsFields) {
  kinematics_lib::ProfilePercent pp(50, 60, 70);
  EXPECT_EQ(pp.vel_perc_, 50);
  EXPECT_EQ(pp.acc_perc_, 60);
  EXPECT_EQ(pp.jerk_perc_, 70);
}

TEST(MotionDataTest, LocDataAssignsFields) {
  kinematics_lib::LocData l(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1, -1);
  EXPECT_DOUBLE_EQ(l.x_, 1.0);
  EXPECT_DOUBLE_EQ(l.y_, 2.0);
  EXPECT_DOUBLE_EQ(l.z_, 3.0);
  EXPECT_EQ(l.branch_, 1);
  EXPECT_EQ(l.turns_, -1);
}

}  // namespace
