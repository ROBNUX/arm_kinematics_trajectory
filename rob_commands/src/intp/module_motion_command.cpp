#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <trajectory/move_command.hpp>

#include "intp/intp.hpp"
#include "scurve_lib/segment_planning.hpp"
namespace py = pybind11;

PYBIND11_MODULE(rob_commands, m) {
  // optional module docstring
  m.doc() = "commonly used robotics command and data types";

  // binding to Motion_Command_Type
  py::enum_<kinematics_lib::Motion_Command_Type>(m, "MotionType")
      .value("NULL", kinematics_lib::Motion_Command_Type::ID_NULL)
      .value("LINE", kinematics_lib::Motion_Command_Type::ID_LINE)
      .value("ARC", kinematics_lib::Motion_Command_Type::ID_ARC)
      .value("PTP", kinematics_lib::Motion_Command_Type::ID_PTP)
      .export_values();

  // binding to IPO_FrameType
  py::enum_<kinematics_lib::IPO_MODE>(m, "IpoMode")
      .value("WORLD", kinematics_lib::IPO_MODE::ID_WORLD)
      .value("BASE", kinematics_lib::IPO_MODE::ID_BASE)
      .value("TOOL", kinematics_lib::IPO_MODE::ID_TOOL)
      .export_values();

  // binding to FrameData class
  py::class_<kinematics_lib::FrameData>(m, "FrameData")
      .def(py::init<>())
      .def(py::init<const unsigned int, const unsigned int,
                    const kinematics_lib::IPO_MODE &>())
      .def_readwrite("base", &kinematics_lib::FrameData::baseNo_)
      .def_readwrite("tool", &kinematics_lib::FrameData::toolNo_)
      .def_readwrite("ipo", &kinematics_lib::FrameData::ipo_);

  // bindings to ProfilePercent class
  py::class_<kinematics_lib::ProfilePercent>(m, "Percent")
      .def(py::init<>())
      .def(py::init<const int, const int, const int>())
      .def_readwrite("v", &kinematics_lib::ProfilePercent::vel_perc_)
      .def_readwrite("a", &kinematics_lib::ProfilePercent::acc_perc_)
      .def_readwrite("j", &kinematics_lib::ProfilePercent::jerk_perc_);

  // bindings to Profile class
  py::class_<kinematics_lib::ProfileData>(m, "Profile")
      .def(py::init<>())
      .def(py::init<const double, const double, const double, const double,
                    const double, const double>())
      .def_readwrite(
          "max_vel_t",
          &kinematics_lib::ProfileData::max_vel_t_)  // translational vel.
      .def_readwrite("max_acc_t", &kinematics_lib::ProfileData::max_acc_t_)
      .def_readwrite("max_jerk_t", &kinematics_lib::ProfileData::max_jerk_t_)
      .def_readwrite(
          "max_vel_r",
          &kinematics_lib::ProfileData::max_vel_r_)  // translational vel.
      .def_readwrite("max_acc_r", &kinematics_lib::ProfileData::max_acc_r_)
      .def_readwrite("max_jerk_r", &kinematics_lib::ProfileData::max_jerk_r_);

  // binding to JntProfile class
  py::class_<kinematics_lib::JntProfile>(m, "JntProfile")
      .def(py::init<>())
      .def(py::init<const double, const double, const double>())
      .def_readwrite(
          "max_vel",
          &kinematics_lib::JntProfile::max_vel_)  // translational vel.
      .def_readwrite("max_acc", &kinematics_lib::JntProfile::max_acc_)
      .def_readwrite("max_jerk", &kinematics_lib::JntProfile::max_jerk_);

  // bindings to LocData class
  py::class_<kinematics_lib::LocData>(m, "LocData")
      .def(py::init<>())
      .def(py::init<const double, const double, const double, const double,
                    const double, const double, const int, const int>())
      .def_readwrite("x", &kinematics_lib::LocData::x_)  // translational vel.
      .def_readwrite("y", &kinematics_lib::LocData::y_)
      .def_readwrite("z", &kinematics_lib::LocData::z_)
      .def_readwrite("A", &kinematics_lib::LocData::A_)  // translational vel.
      .def_readwrite("B", &kinematics_lib::LocData::B_)
      .def_readwrite("C", &kinematics_lib::LocData::C_)
      .def_readwrite("G", &kinematics_lib::LocData::branch_)
      .def_readwrite("T", &kinematics_lib::LocData::turns_);

  // binding to TestLocData class
  py::class_<kinematics_lib::SegmentPlanning>(m, "ScurvePlanner")
      .def_static("FitScurveSegment",
                  &kinematics_lib::SegmentPlanning::fit_traj_segment_samples);

  // binding to CreateRobot class
  py::class_<kinematics_lib::CreateRobot>(m, "Robot")
      .def(py::init<const std::string &, const EigenDRef<Eigen::VectorXd> &,
                    const EigenDRef<Eigen::VectorXd> &,
                    const EigenDRef<Eigen::VectorXd> &,
                    const kinematics_lib::ProfileData &>())
      .def("SetFeedback", &kinematics_lib::CreateRobot::SetJntFeedback)
      .def("StartMotion", &kinematics_lib::CreateRobot::StartMotion)
      .def("StopMotion", &kinematics_lib::CreateRobot::StopMotion)
      .def("PauseMotion", &kinematics_lib::CreateRobot::PauseMotion)
      .def("ResumeMotion", &kinematics_lib::CreateRobot::ResumeMotion)
      .def("MotionDone", &kinematics_lib::CreateRobot::MotionDone)
      .def("SetSpeed", &kinematics_lib::CreateRobot::SetSpeedScale)
      .def("GetSpeed", &kinematics_lib::CreateRobot::GetSpeedScale)
      .def("SetJntProfile", &kinematics_lib::CreateRobot::SetJntProfile)
      .def("MoveLine", &kinematics_lib::CreateRobot::LIN)  // lin command
      .def("MoveArc", &kinematics_lib::CreateRobot::ARC)   // arc command
      .def("MovePTP", &kinematics_lib::CreateRobot::PTP)
      .def("MovePTPJ", &kinematics_lib::CreateRobot::PTPJ)
      .def("MoveLineRel",
           &kinematics_lib::CreateRobot::LIN_REL)  // relative line
      .def("MovePTPRel", &kinematics_lib::CreateRobot::PTP_REL)
      .def("ForwardKin", &kinematics_lib::CreateRobot::ForwardKin)
      .def("InverseKin",
           &kinematics_lib::CreateRobot::InverseKin)  // re;atove {T{}}
      .def("Shutdown", &kinematics_lib::CreateRobot::ShutDown);
}
