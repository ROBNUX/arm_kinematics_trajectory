// Smoke tests for robnux_kdl_common: Vec, Quaternion, Rotation, Frame, Pose,
// refPose, Twist, Wrench.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/twistWrench.hpp"
#include "robnux_kdl_common/vec.hpp"

namespace {

using kinematics_lib::Frame;
using kinematics_lib::Pose;
using kinematics_lib::Quaternion;
using kinematics_lib::refPose;
using kinematics_lib::Rotation;
using kinematics_lib::Twist;
using kinematics_lib::Vec;
using kinematics_lib::Wrench;

constexpr double kTol = 1e-9;

TEST(VecTest, ConstructionAndAccess) {
  Vec v(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(v.x(), 1.0);
  EXPECT_DOUBLE_EQ(v.y(), 2.0);
  EXPECT_DOUBLE_EQ(v.z(), 3.0);
  EXPECT_NEAR(v.Norm(), std::sqrt(14.0), kTol);
}

TEST(VecTest, ArithmeticOperators) {
  Vec a(1.0, 0.0, 0.0);
  Vec b(0.0, 1.0, 0.0);
  Vec sum = a + b;
  EXPECT_NEAR(sum.x(), 1.0, kTol);
  EXPECT_NEAR(sum.y(), 1.0, kTol);
  EXPECT_NEAR(a.dot(b), 0.0, kTol);
}

TEST(VecTest, NormalizeReturnsUnitVector) {
  Vec v(3.0, 4.0, 0.0);
  Vec n = v.NormalizeVec();
  EXPECT_NEAR(n.Norm(), 1.0, 1e-6);
}

TEST(QuaternionTest, IdentityAndEulerRoundTrip) {
  Quaternion q = Quaternion::getIdentity();
  EXPECT_NEAR(q.w(), 1.0, kTol);
  EXPECT_NEAR(q.x(), 0.0, kTol);
  q.SetEulerZYX(0.4, 0.2, -0.1);
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
  ASSERT_TRUE(q.GetEulerZYX(&yaw, &pitch, &roll));
  EXPECT_NEAR(yaw, 0.4, 1e-6);
  EXPECT_NEAR(pitch, 0.2, 1e-6);
  EXPECT_NEAR(roll, -0.1, 1e-6);
}

TEST(RotationTest, IdentityIsIdempotent) {
  Rotation r = Rotation::Identity();
  // The 4x4 access operator returns the (i,j) element of the identity Frame
  // backing; we instead verify that the Euler decomposition is all zero.
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
  ASSERT_TRUE(r.GetEulerZYX(&yaw, &pitch, &roll));
  EXPECT_NEAR(yaw, 0.0, kTol);
  EXPECT_NEAR(pitch, 0.0, kTol);
  EXPECT_NEAR(roll, 0.0, kTol);
}

TEST(FrameTest, IdentityHasZeroTranslation) {
  Frame f = Frame::Identity();
  Vec t = f.getTranslation();
  EXPECT_NEAR(t.x(), 0.0, kTol);
  EXPECT_NEAR(t.y(), 0.0, kTol);
  EXPECT_NEAR(t.z(), 0.0, kTol);
}

TEST(FrameTest, ComposeWithInverseGivesIdentity) {
  Quaternion q;
  q.SetEulerZYX(0.3, -0.2, 0.1);
  Frame f(q, Vec(1.0, 2.0, 3.0));
  Frame inv = f.Inverse();
  Frame id = f * inv;
  Vec t = id.getTranslation();
  EXPECT_NEAR(t.x(), 0.0, 1e-9);
  EXPECT_NEAR(t.y(), 0.0, 1e-9);
  EXPECT_NEAR(t.z(), 0.0, 1e-9);
}

TEST(FrameTest, DhCraig1989ProducesValidFrame) {
  // For all-zero DH parameters the result must be a valid frame (no NaN).
  Frame f = Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0);
  Vec t = f.getTranslation();
  EXPECT_FALSE(std::isnan(t.x()));
  EXPECT_FALSE(std::isnan(t.y()));
  EXPECT_FALSE(std::isnan(t.z()));
}

TEST(PoseTest, DefaultConstructible) {
  Pose p;
  std::vector<int> branch;
  std::vector<int> turns;
  EXPECT_TRUE(p.getBranchFlags(&branch));
  EXPECT_TRUE(p.getJointTurns(&turns));
}

TEST(RefPoseTest, BaseAndToolRoundTrip) {
  Frame ee(Vec(0.5, 0.0, 0.0));
  Frame base(Vec(1.0, 0.0, 0.0));
  Frame tool(Vec(0.0, 1.0, 0.0));
  refPose rp(ee, base, tool, {0}, {0});

  Frame base_out;
  Frame tool_out;
  ASSERT_TRUE(rp.getBase(&base_out));
  ASSERT_TRUE(rp.getTool(&tool_out));
  EXPECT_NEAR(base_out.getTranslation().x(), 1.0, kTol);
  EXPECT_NEAR(tool_out.getTranslation().y(), 1.0, kTol);
}

TEST(TwistTest, ZeroTwistHasZeroComponents) {
  Twist t = Twist::Zero();
  // Twist exposes its 6 components through operator[] (0..2 = vel, 3..5 = rot).
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(t[i], 0.0, kTol);
  }
}

TEST(WrenchTest, ConstructorAssignsForceAndTorque) {
  Wrench w(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0));
  EXPECT_NEAR(w.getForce().x(), 1.0, kTol);
  EXPECT_NEAR(w.getTorque().y(), 1.0, kTol);
}

}  // namespace
