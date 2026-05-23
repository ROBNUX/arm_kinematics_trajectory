// Smoke tests for robnux_trajectory: SCurveProfile, line/arc trajectories,
// rotational trajectory, and motion-command data types.

#include <gtest/gtest.h>

#include "robnux_trajectory/arc_trajectory.hpp"
#include "robnux_trajectory/line_trajectory.hpp"
#include "robnux_trajectory/move_command.hpp"
#include "robnux_trajectory/s_curve.hpp"
#include "robnux_trajectory/shortest_quat_trajectory.hpp"

namespace {

using kinematics_lib::ArcTrajectory;
using kinematics_lib::FrameData;
using kinematics_lib::IPO_MODE;
using kinematics_lib::JntProfile;
using kinematics_lib::LineTrajectory;
using kinematics_lib::LocData;
using kinematics_lib::ProfileData;
using kinematics_lib::ProfilePercent;
using kinematics_lib::SCurveProfile;
using kinematics_lib::ShortestQuatTrajectory;

constexpr double kTol = 1e-9;

TEST(SCurveProfileTest, ConstructibleAndAcceptsConstraints) {
  SCurveProfile p;
  p.setConstraints(2.0, 5.0, 20.0);
  EXPECT_TRUE(p.setBoundaryCondition(0.0, 1.0, 0.0, 0.0));
}

TEST(SCurveProfileTest, EvaluatesAtTimeZeroIsStartPos) {
  SCurveProfile p;
  p.setConstraints(2.0, 5.0, 20.0);
  ASSERT_TRUE(p.setBoundaryCondition(0.0, 1.0, 0.0, 0.0));
  double pos = 0;
  double vel = 0;
  double acc = 0;
  ASSERT_TRUE(p.Trajectory(0.0, &pos, &vel, &acc));
  EXPECT_NEAR(pos, 0.0, 1e-6);
  EXPECT_NEAR(vel, 0.0, 1e-6);
}

TEST(LineTrajectoryTest, DefaultConstructibleWithoutCrash) {
  LineTrajectory traj;
  // No public setter on the default-constructed line beyond the inherited
  // CartTrajectory API; we only assert it constructs cleanly.
  SUCCEED();
}

TEST(ArcTrajectoryTest, DefaultConstructibleWithoutCrash) {
  ArcTrajectory traj;
  SUCCEED();
}

TEST(ShortestQuatTrajectoryTest, DefaultConstructibleWithoutCrash) {
  ShortestQuatTrajectory traj;
  SUCCEED();
}

TEST(MoveCommandDataTest, FrameDataFieldsRoundTrip) {
  FrameData fd(1, 2, IPO_MODE::ID_TOOL);
  EXPECT_EQ(fd.baseNo_, 1u);
  EXPECT_EQ(fd.toolNo_, 2u);
  EXPECT_EQ(fd.ipo_, IPO_MODE::ID_TOOL);
}

TEST(MoveCommandDataTest, ProfileDataFieldsRoundTrip) {
  ProfileData p(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  EXPECT_DOUBLE_EQ(p.max_vel_t_, 1.0);
  EXPECT_DOUBLE_EQ(p.max_acc_t_, 2.0);
  EXPECT_DOUBLE_EQ(p.max_jerk_t_, 3.0);
  EXPECT_DOUBLE_EQ(p.max_vel_r_, 4.0);
  EXPECT_DOUBLE_EQ(p.max_acc_r_, 5.0);
  EXPECT_DOUBLE_EQ(p.max_jerk_r_, 6.0);
}

TEST(MoveCommandDataTest, ProfilePercentFieldsRoundTrip) {
  ProfilePercent pp(50, 60, 70);
  EXPECT_EQ(pp.vel_perc_, 50);
  EXPECT_EQ(pp.acc_perc_, 60);
  EXPECT_EQ(pp.jerk_perc_, 70);
}

TEST(MoveCommandDataTest, JntProfileFieldsRoundTrip) {
  JntProfile jp(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(jp.max_vel_, 1.0);
  EXPECT_DOUBLE_EQ(jp.max_acc_, 2.0);
  EXPECT_DOUBLE_EQ(jp.max_jerk_, 3.0);
}

TEST(MoveCommandDataTest, LocDataFieldsRoundTrip) {
  LocData l(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1, -1);
  EXPECT_DOUBLE_EQ(l.x_, 1.0);
  EXPECT_DOUBLE_EQ(l.y_, 2.0);
  EXPECT_DOUBLE_EQ(l.z_, 3.0);
  EXPECT_DOUBLE_EQ(l.A_, 0.1);
  EXPECT_EQ(l.branch_, 1);
  EXPECT_EQ(l.turns_, -1);
}

}  // namespace
