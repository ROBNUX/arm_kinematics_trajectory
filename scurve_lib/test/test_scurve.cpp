// Smoke tests for the S-curve trajectory utilities in scurve_lib.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "scurve_lib/piecewise_function.hpp"
#include "scurve_lib/segment_planning.hpp"

namespace {

using kinematics_lib::PiecewiseFunction;
using kinematics_lib::PolyNomials;
using kinematics_lib::SegmentPlanning;

constexpr double kTol = 1e-9;

TEST(PolyNomialsTest, EvaluatesConstantPolynomial) {
  PolyNomials p(std::vector<double>{2.0});
  EXPECT_NEAR(p.evaluate(0.0), 2.0, kTol);
  EXPECT_NEAR(p.evaluate(100.0), 2.0, kTol);
}

TEST(PolyNomialsTest, EvaluatesLinearPolynomial) {
  // f(t) = 1.0 + 2.0 * t
  PolyNomials p(std::vector<double>{1.0, 2.0});
  EXPECT_NEAR(p.evaluate(0.0), 1.0, kTol);
  EXPECT_NEAR(p.evaluate(3.0), 7.0, kTol);
}

TEST(PolyNomialsTest, IntegrateAddsDegreeAndConstant) {
  // f(t) = 2.0 -> integral = c + 2t
  PolyNomials p(std::vector<double>{2.0});
  PolyNomials anti = p.Integrate(5.0);
  EXPECT_NEAR(anti.evaluate(0.0), 5.0, kTol);
  EXPECT_NEAR(anti.evaluate(1.0), 7.0, kTol);
  EXPECT_EQ(anti.GetMaxDegree(), 1);
}

TEST(SegmentPlanningTest, EqualVelCaseReturnsNonEmptyPlan) {
  // Symmetric segment: move 1.0 unit with matched start/end vel.
  std::vector<double> plan =
      SegmentPlanning::equal_vel_case_planning(1.0, 0.0, 5.0, 10.0, 100.0);
  EXPECT_FALSE(plan.empty());
}

TEST(SegmentPlanningTest, MinPosToReachMaxAccIsNonNegative) {
  // Travelling from rest with v_max=1, a_max=2, j_max=5 — distance needed to
  // reach a_max must be a non-negative real value.
  const double v = 0.0;
  const double v_max = 1.0;
  const double a_max = 2.0;
  const double j_max = 5.0;
  double dp = SegmentPlanning::calculate_min_pos_to_reach_max_acc(v, v_max,
                                                                   a_max, j_max);
  EXPECT_GE(dp, 0.0);
}

TEST(SegmentPlanningTest, FitTrajSegmentProducesPiecewiseSegments) {
  // p_start=0, p_end=1, v_start=v_end=0, p_max=10, v_max=2, a_max=4, j_max=10
  std::vector<PiecewiseFunction> segs = SegmentPlanning::fit_traj_segment(
      0.0, 1.0, 0.0, 0.0, 10.0, 2.0, 4.0, 10.0);
  // The fit is expected to produce position/velocity/acceleration/jerk pieces;
  // we don't assume a specific count but it should be non-empty for a real
  // motion.
  EXPECT_FALSE(segs.empty());
}

}  // namespace
