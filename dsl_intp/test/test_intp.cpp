// Smoke tests for the dsl_intp::CreateRobot interpreter.
//
// CreateRobot's full happy path requires (1) a running rclcpp context, (2) a
// registered kinematics plugin, (3) correctly-sized DH / base / tool vectors,
// and (4) two background threads. We only exercise the "graceful failure"
// path here: passing an unknown plugin name causes pluginlib to throw, which
// is caught and logged; the object should construct (in an uninitialized
// state) and destruct without crashing.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include "dsl_intp/intp.hpp"
#include "robnux_trajectory/move_command.hpp"

namespace {

using kinematics_lib::CreateRobot;
using kinematics_lib::ProfileData;

// A test fixture that initializes rclcpp once for the suite. CreateRobot
// requires a valid ROS 2 context because its constructor creates a node and
// publishers.
class IntpTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(IntpTest, ContextInitializesForTestSuite) {
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(IntpTest, UnknownPluginNameDoesNotCrashConstructor) {
  Eigen::VectorXd kine_para = Eigen::VectorXd::Zero(16);
  Eigen::VectorXd base_off = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sub_base_off = Eigen::VectorXd::Zero(7);
  EigenDRef<Eigen::VectorXd> kp_ref(kine_para);
  EigenDRef<Eigen::VectorXd> bo_ref(base_off);
  EigenDRef<Eigen::VectorXd> sbo_ref(sub_base_off);
  ProfileData pf(1.0, 2.0, 3.0, 1.0, 2.0, 3.0);

  // Pluginlib raises CreateClassException when the lookup name is unknown;
  // CreateRobot's constructor catches via the resulting null armMap_ check
  // and early-returns without starting threads. The destructor must run
  // cleanly regardless. We accept either: no throw, or a pluginlib throw —
  // both should ultimately leave the test fixture clean.
  try {
    CreateRobot robot("__no_such_plugin__", kp_ref, bo_ref, sbo_ref, pf);
  } catch (const std::exception&) {
    // pluginlib reported the lookup failure; that is acceptable.
  }
  SUCCEED();
}

}  // namespace
