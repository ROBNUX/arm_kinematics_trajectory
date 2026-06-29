#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rclcpp/rclcpp.hpp>

#include "dsl_intp/intp.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_trajectory/move_command.hpp"
#include "scurve_lib/segment_planning.hpp"

namespace py = pybind11;

namespace {

// Wrap CreateRobot kinematics methods that fill output Eigen / LocData buffers
// through reference parameters. The wrapped versions allocate the output buffer
// (caller specifies the DoF when needed) and return it to Python by value.

py::tuple py_GetCartFromJnt(kinematics_lib::CreateRobot& self,
                            const EigenDRef<Eigen::VectorXd>& jnt,
                            Eigen::Index /*cart_size*/) {
  // GetCartFromJnt always outputs [x,y,z,roll,pitch,yaw] — 6 elements.
  // Eigen::Ref cannot be resized, so we must pre-allocate the exact output size.
  Eigen::VectorXd cart = Eigen::VectorXd::Zero(6);
  EigenDRef<Eigen::VectorXd> cart_ref(cart);
  bool ok = self.GetCartFromJnt(jnt, cart_ref);
  return py::make_tuple(ok, cart);
}

py::tuple py_GetPoseFromJnt(kinematics_lib::CreateRobot& self,
                            const EigenDRef<Eigen::VectorXd>& jnt,
                            Eigen::Index /*pose_size*/) {
  // GetPoseFromJnt always outputs 8 elements: [x,y,z,roll,pitch,yaw,config,turns].
  Eigen::VectorXd pose = Eigen::VectorXd::Zero(8);
  EigenDRef<Eigen::VectorXd> pose_ref(pose);
  bool ok = self.GetPoseFromJnt(jnt, pose_ref);
  return py::make_tuple(ok, pose);
}

py::tuple py_GetJntFromPose(kinematics_lib::CreateRobot& self,
                            const EigenDRef<Eigen::VectorXd>& pose,
                            Eigen::Index dof) {
  Eigen::VectorXd jnt = Eigen::VectorXd::Zero(dof);
  EigenDRef<Eigen::VectorXd> jnt_ref(jnt);
  bool ok = self.GetJntFromPose(pose, jnt_ref);
  return py::make_tuple(ok, jnt);
}

py::tuple py_ForwardKin(kinematics_lib::CreateRobot& self,
                        const EigenDRef<Eigen::VectorXd>& jnt) {
  kinematics_lib::LocData pose;
  bool ok = self.ForwardKin(jnt, pose);
  return py::make_tuple(ok, pose);
}

py::tuple py_InverseKin(kinematics_lib::CreateRobot& self,
                        const kinematics_lib::LocData& pose, Eigen::Index dof) {
  Eigen::VectorXd jnt = Eigen::VectorXd::Zero(dof);
  EigenDRef<Eigen::VectorXd> jnt_ref(jnt);
  bool ok = self.InverseKin(pose, jnt_ref);
  return py::make_tuple(ok, jnt);
}

}  // namespace

PYBIND11_MODULE(rob_motion_commands, m) {
  m.doc() =
      "Robot motion commands: enums, profile/loc/frame data types, "
      "Vec/Quaternion/Rotation/Frame/Pose/refPose, scurve planner, and the "
      "CreateRobot interpreter.";

  // Tie the ROS 2 context lifetime to this Python module. CreateRobot
  // constructs a rclcpp::Node and publishers in its constructor; without a
  // running context that throws at the first create_publisher call. We
  // initialize on module import and shut down via Python's atexit so that
  // CreateRobot destructors (which join threads and reset the node) get a
  // chance to run before rclcpp::shutdown() tears the context down.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  py::module_::import("atexit").attr("register")(py::cpp_function([]() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }));

  // Expose explicit init/shutdown for callers that want to manage the
  // context themselves (e.g. embedding inside an existing rclpy session).
  m.def(
      "ros_init",
      []() {
        if (!rclcpp::ok()) {
          rclcpp::init(0, nullptr);
        }
      },
      "Initialize the ROS 2 context if it is not already running.");
  m.def(
      "ros_shutdown",
      []() {
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
      },
      "Shut down the ROS 2 context. Called automatically at interpreter "
      "exit; expose for tests/embedding cases that need an early teardown.");
  m.def(
      "ros_ok", []() { return rclcpp::ok(); },
      "True if the ROS 2 context is currently active.");

  // --- Geometric primitives (robnux_kdl_common) ----------------------------
  py::class_<kinematics_lib::Vec>(m, "Vec")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
           py::arg("z"))
      .def(py::init<const Eigen::Vector3d&>(), py::arg("v"))
      .def("x", &kinematics_lib::Vec::x)
      .def("y", &kinematics_lib::Vec::y)
      .def("z", &kinematics_lib::Vec::z)
      .def("set_x", &kinematics_lib::Vec::set_x)
      .def("set_y", &kinematics_lib::Vec::set_y)
      .def("set_z", &kinematics_lib::Vec::set_z)
      .def("Norm", &kinematics_lib::Vec::Norm)
      .def("ToEigenVec", &kinematics_lib::Vec::ToEigenVec)
      .def_static("Zero", &kinematics_lib::Vec::Zero)
      .def_static("FromEigenVec", &kinematics_lib::Vec::FromEigenVec)
      .def("__repr__", [](const kinematics_lib::Vec& v) {
        return std::string("Vec(") + v.ToString() + ")";
      });

  py::class_<kinematics_lib::Quaternion>(m, "Quaternion")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("w"),
           py::arg("x"), py::arg("y"), py::arg("z"))
      .def(py::init<const kinematics_lib::Vec&, double>(), py::arg("axis"),
           py::arg("angle"))
      .def("w", &kinematics_lib::Quaternion::w)
      .def("x", &kinematics_lib::Quaternion::x)
      .def("y", &kinematics_lib::Quaternion::y)
      .def("z", &kinematics_lib::Quaternion::z)
      .def("SetEulerZYX", &kinematics_lib::Quaternion::SetEulerZYX,
           py::arg("yaw"), py::arg("pitch"), py::arg("roll"))
      .def("GetEulerZYX",
           [](const kinematics_lib::Quaternion& q) {
             double yaw = 0;
             double pitch = 0;
             double roll = 0;
             bool ok = q.GetEulerZYX(&yaw, &pitch, &roll);
             return py::make_tuple(ok, yaw, pitch, roll);
           })
      .def_static("getIdentity", &kinematics_lib::Quaternion::getIdentity);

  py::class_<kinematics_lib::Rotation>(m, "Rotation")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("yaw"), py::arg("pitch"),
           py::arg("roll"))
      .def(py::init<const kinematics_lib::Quaternion&>(), py::arg("q"))
      .def(py::init<const kinematics_lib::Vec&, double>(), py::arg("axis"),
           py::arg("angle"))
      .def("SetEulerZYX", &kinematics_lib::Rotation::SetEulerZYX,
           py::arg("yaw"), py::arg("pitch"), py::arg("roll"))
      .def("GetEulerZYX",
           [](const kinematics_lib::Rotation& r) {
             double yaw = 0;
             double pitch = 0;
             double roll = 0;
             bool ok = r.GetEulerZYX(&yaw, &pitch, &roll);
             return py::make_tuple(ok, yaw, pitch, roll);
           })
      .def_static("Identity", &kinematics_lib::Rotation::Identity);

  py::class_<kinematics_lib::Frame>(m, "Frame")
      .def(py::init<>())
      .def(py::init<const kinematics_lib::Quaternion&,
                    const kinematics_lib::Vec&>(),
           py::arg("q"), py::arg("p"))
      .def(py::init<const kinematics_lib::Rotation&,
                    const kinematics_lib::Vec&>(),
           py::arg("R"), py::arg("V"))
      .def(py::init<const kinematics_lib::Vec&>(), py::arg("V"))
      .def(py::init<const kinematics_lib::Rotation&>(), py::arg("R"))
      .def("getTranslation", &kinematics_lib::Frame::getTranslation)
      .def("getRotation", &kinematics_lib::Frame::getRotation)
      .def("getQuaternion", &kinematics_lib::Frame::getQuaternion)
      .def("setTranslation", &kinematics_lib::Frame::setTranslation,
           py::arg("trans"))
      .def("setRotation", &kinematics_lib::Frame::setRotation, py::arg("rot"))
      .def("setQuaternion", &kinematics_lib::Frame::setQuaternion, py::arg("q"))
      .def("Inverse", &kinematics_lib::Frame::Inverse)
      .def("ToEigenVecQuat", &kinematics_lib::Frame::ToEigenVecQuat)
      .def("ToEigenVecEulerZYX", &kinematics_lib::Frame::ToEigenVecEulerZYX)
      .def_static("Identity", &kinematics_lib::Frame::Identity)
      .def_static("DH_Craig1989", &kinematics_lib::Frame::DH_Craig1989,
                  py::arg("a"), py::arg("alpha"), py::arg("d"),
                  py::arg("theta"))
      .def_static("DH", &kinematics_lib::Frame::DH, py::arg("a"),
                  py::arg("alpha"), py::arg("d"), py::arg("theta"));

  py::class_<kinematics_lib::Pose, kinematics_lib::Frame>(m, "Pose")
      .def(py::init<>())
      .def(py::init<const std::vector<int>&, const std::vector<int>>(),
           py::arg("ikBranchFlags"), py::arg("ikJointTurns"))
      .def(py::init<const kinematics_lib::Frame&>(), py::arg("ee_frame"))
      .def(py::init<const kinematics_lib::Frame&, const std::vector<int>&,
                    const std::vector<int>>(),
           py::arg("ee_frame"), py::arg("ikBranchFlags"),
           py::arg("ikJointTurns"))
      .def("setFrame", &kinematics_lib::Pose::setFrame, py::arg("ee_frame"))
      .def("setBranchFlags", &kinematics_lib::Pose::setBranchFlags,
           py::arg("ikBranchFlags"))
      .def("setJointTurns", &kinematics_lib::Pose::setJointTurns,
           py::arg("ikJointTurns"))
      .def("getBranchFlags",
           [](const kinematics_lib::Pose& self) {
             std::vector<int> v;
             bool ok = self.getBranchFlags(&v);
             return py::make_tuple(ok, v);
           })
      .def("getJointTurns",
           [](const kinematics_lib::Pose& self) {
             std::vector<int> v;
             bool ok = self.getJointTurns(&v);
             return py::make_tuple(ok, v);
           })
      .def("getFrame", [](const kinematics_lib::Pose& self) {
        kinematics_lib::Frame fr;
        bool ok = self.getFrame(&fr);
        return py::make_tuple(ok, fr);
      });

  py::class_<kinematics_lib::refPose, kinematics_lib::Pose>(m, "refPose")
      .def(py::init<>())
      .def(py::init<const kinematics_lib::Frame&, const kinematics_lib::Frame&,
                    const kinematics_lib::Frame&, const std::vector<int>&,
                    const std::vector<int>>(),
           py::arg("ee_frame"), py::arg("base"), py::arg("tool"),
           py::arg("ikBranchFlags"), py::arg("ikJointTurns"))
      .def("setDefaultPose", &kinematics_lib::refPose::setDefaultPose,
           py::arg("ps"))
      .def("setBase", &kinematics_lib::refPose::setBase, py::arg("base"))
      .def("setTool", &kinematics_lib::refPose::setTool, py::arg("tool"))
      .def("getBase",
           [](const kinematics_lib::refPose& self) {
             kinematics_lib::Frame f;
             bool ok = self.getBase(&f);
             return py::make_tuple(ok, f);
           })
      .def("getTool",
           [](const kinematics_lib::refPose& self) {
             kinematics_lib::Frame f;
             bool ok = self.getTool(&f);
             return py::make_tuple(ok, f);
           })
      .def("getDefaultFrame",
           [](const kinematics_lib::refPose& self) {
             kinematics_lib::Frame f;
             bool ok = self.getDefaultFrame(&f);
             return py::make_tuple(ok, f);
           })
      .def("getDefaultPose", [](const kinematics_lib::refPose& self) {
        kinematics_lib::Pose p;
        bool ok = self.getDefaultPose(&p);
        return py::make_tuple(ok, p);
      });

  // --- Motion command enums / types ---------------------------------------
  py::enum_<kinematics_lib::Motion_Command_Type>(m, "MotionType")
      .value("NULL", kinematics_lib::Motion_Command_Type::ID_NULL)
      .value("LINE", kinematics_lib::Motion_Command_Type::ID_LINE)
      .value("ARC", kinematics_lib::Motion_Command_Type::ID_ARC)
      .value("PTP", kinematics_lib::Motion_Command_Type::ID_PTP)
      .export_values();

  py::enum_<kinematics_lib::IPO_MODE>(m, "IpoMode")
      .value("WORLD", kinematics_lib::IPO_MODE::ID_WORLD)
      .value("BASE", kinematics_lib::IPO_MODE::ID_BASE)
      .value("TOOL", kinematics_lib::IPO_MODE::ID_TOOL)
      .export_values();

  py::class_<kinematics_lib::FrameData>(m, "FrameData")
      .def(py::init<>())
      .def(py::init<const unsigned int, const unsigned int,
                    const kinematics_lib::IPO_MODE&>(),
           py::arg("base"), py::arg("tool"), py::arg("ipo"))
      .def_readwrite("base", &kinematics_lib::FrameData::baseNo_)
      .def_readwrite("tool", &kinematics_lib::FrameData::toolNo_)
      .def_readwrite("ipo", &kinematics_lib::FrameData::ipo_);

  py::class_<kinematics_lib::ProfilePercent>(m, "Percent")
      .def(py::init<>())
      .def(py::init<const int, const int, const int>(), py::arg("v"),
           py::arg("a"), py::arg("j"))
      .def_readwrite("v", &kinematics_lib::ProfilePercent::vel_perc_)
      .def_readwrite("a", &kinematics_lib::ProfilePercent::acc_perc_)
      .def_readwrite("j", &kinematics_lib::ProfilePercent::jerk_perc_);

  py::class_<kinematics_lib::ProfileData>(m, "Profile")
      .def(py::init<>())
      .def(py::init<const double, const double, const double, const double,
                    const double, const double>(),
           py::arg("max_vel_t"), py::arg("max_acc_t"), py::arg("max_jerk_t"),
           py::arg("max_vel_r"), py::arg("max_acc_r"), py::arg("max_jerk_r"))
      .def_readwrite("max_vel_t", &kinematics_lib::ProfileData::max_vel_t_)
      .def_readwrite("max_acc_t", &kinematics_lib::ProfileData::max_acc_t_)
      .def_readwrite("max_jerk_t", &kinematics_lib::ProfileData::max_jerk_t_)
      .def_readwrite("max_vel_r", &kinematics_lib::ProfileData::max_vel_r_)
      .def_readwrite("max_acc_r", &kinematics_lib::ProfileData::max_acc_r_)
      .def_readwrite("max_jerk_r", &kinematics_lib::ProfileData::max_jerk_r_);

  py::class_<kinematics_lib::JntProfile>(m, "JntProfile")
      .def(py::init<>())
      .def(py::init<const double, const double, const double>(),
           py::arg("max_vel"), py::arg("max_acc"), py::arg("max_jerk"))
      .def_readwrite("max_vel", &kinematics_lib::JntProfile::max_vel_)
      .def_readwrite("max_acc", &kinematics_lib::JntProfile::max_acc_)
      .def_readwrite("max_jerk", &kinematics_lib::JntProfile::max_jerk_);

  py::class_<kinematics_lib::LocData>(m, "LocData")
      .def(py::init<>())
      .def(py::init<const double, const double, const double, const double,
                    const double, const double, const int, const int>(),
           py::arg("x"), py::arg("y"), py::arg("z"), py::arg("A"), py::arg("B"),
           py::arg("C"), py::arg("G"), py::arg("T"))
      .def_readwrite("x", &kinematics_lib::LocData::x_)
      .def_readwrite("y", &kinematics_lib::LocData::y_)
      .def_readwrite("z", &kinematics_lib::LocData::z_)
      .def_readwrite("A", &kinematics_lib::LocData::A_)
      .def_readwrite("B", &kinematics_lib::LocData::B_)
      .def_readwrite("C", &kinematics_lib::LocData::C_)
      .def_readwrite("G", &kinematics_lib::LocData::branch_)
      .def_readwrite("T", &kinematics_lib::LocData::turns_);

  py::class_<kinematics_lib::SegmentPlanning>(m, "ScurvePlanner")
      .def_static("FitScurveSegment",
                  &kinematics_lib::SegmentPlanning::fit_traj_segment_samples);

  py::class_<kinematics_lib::CreateRobot>(m, "Robot")
      .def(py::init<const std::string&, const EigenDRef<Eigen::VectorXd>&,
                    const EigenDRef<Eigen::VectorXd>&,
                    const EigenDRef<Eigen::VectorXd>&,
                    const kinematics_lib::ProfileData&>(),
           py::arg("robName"), py::arg("kine_para"), py::arg("defaultBaseOff"),
           py::arg("defaultSubBaseOff"), py::arg("pf"))
      .def("SetFeedback", &kinematics_lib::CreateRobot::SetJntFeedback)
      .def("StartMotion", &kinematics_lib::CreateRobot::StartMotion)
      .def("StopMotion", &kinematics_lib::CreateRobot::StopMotion)
      .def("PauseMotion", &kinematics_lib::CreateRobot::PauseMotion)
      .def("ResumeMotion", &kinematics_lib::CreateRobot::ResumeMotion)
      .def("MotionDone", &kinematics_lib::CreateRobot::MotionDone)
      .def("SetSpeed", &kinematics_lib::CreateRobot::SetSpeedScale)
      .def("GetSpeed", &kinematics_lib::CreateRobot::GetSpeedScale)
      .def("SetJntProfile", &kinematics_lib::CreateRobot::SetJntProfile)
      .def("MoveLine", &kinematics_lib::CreateRobot::LIN)
      .def("MoveArc", &kinematics_lib::CreateRobot::ARC)
      .def("MovePTP", &kinematics_lib::CreateRobot::PTP)
      .def("MovePTPJ", &kinematics_lib::CreateRobot::PTPJ)
      .def("MoveLineRel", &kinematics_lib::CreateRobot::LIN_REL)
      .def("MovePTPRel", &kinematics_lib::CreateRobot::PTP_REL)
      .def("GetCartFromJnt", &py_GetCartFromJnt, py::arg("jnt"),
           py::arg("cart_size"),
           "Forward kin → flat Cartesian vector. Returns (ok, cart).")
      .def("GetPoseFromJnt", &py_GetPoseFromJnt, py::arg("jnt"),
           py::arg("pose_size"),
           "Forward kin → flat pose vector. Returns (ok, pose).")
      .def("GetJntFromPose", &py_GetJntFromPose, py::arg("pose"),
           py::arg("dof"),
           "Inverse kin from a flat pose vector. Returns (ok, jnt).")
      .def("ForwardKin", &py_ForwardKin, py::arg("jnt"),
           "Forward kin → LocData. Returns (ok, pose).")
      .def("InverseKin", &py_InverseKin, py::arg("pose"), py::arg("dof"),
           "Inverse kin from LocData. Returns (ok, jnt).")
      .def("Shutdown", &kinematics_lib::CreateRobot::ShutDown);
}
