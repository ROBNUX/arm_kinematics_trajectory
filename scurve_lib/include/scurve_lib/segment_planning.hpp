#ifndef KINEMATICS_LIB_SEGMENT_PLANNING_HPP_
#define KINEMATICS_LIB_SEGMENT_PLANNING_HPP_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <limits>
#include <sstream>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "scurve_lib/computeRootsUtil.hpp"
#include "scurve_lib/piecewise_function.hpp"

using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
template <typename MatrixType>
using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;
template <typename MatrixType>
using EigenDMap = Eigen::Map<MatrixType, 0, EigenDStride>;

using namespace ROBNUXLogging;

namespace kinematics_lib {

struct JerkTimePair {
 public:
  JerkTimePair(double jerk, double timeInt)
      : Jerk_(jerk), TimeInterval_(timeInt) {}
  double Jerk_;          // the jerk value in this following interval
  double TimeInterval_;  // the time interval in which we have above jerk value
};

class SCURVE_API SegmentPlanning {
 public:
  /*
     This function calculates the following variables to reach the absolute
    Maximum Velocity "vm"  starting with initial_velocity "v":
    1. the minimum position required to reach the maximum absolute velocity "vm"
    2. the acceleration "acc" that has been reached to reach the maximum
    absolute velocity starting with "v"
   */
  static std::vector<double> calculate_min_pos_reached_acc_to_reach_max_vel(
      const double v, const double vm, const double am, const double jm);

  /*
   This function calculates the acceleration that has been reached to move the
   joint a position equals to "DP" starting with velocity "v" this considers the
   case where there is no a constant acceleration phase.
  */
  static double calculate_reached_acc_in_case_no_const_acc_phase(
      const double Dp, const double v, const double vm, const double am,
      const double jm);

  /*
   This function calculates the following variables to reach the absolute
  Maximum Velocity "vm" starting with initial_velocity "v" considering that the
  absolute maximum acceleration "am" will be reached:
     1. the minimum position required to reach the maximum absolute velocity
  "vm"
     2. the acceleration_phase time "ta" to reach the maximum absolute velocity
  "vm"
  */
  static std::vector<double>
  calculate_min_pos_const_acc_time_to_reach_max_acc_and_max_vel(
      const double v, const double vm, const double am, const double jm);

  /*
   This function calculates the minimum position required to reach the absolute
   maximum acceleration "am", starting with initial velocity "v"
  */
  static double calculate_min_pos_to_reach_max_acc(const double v,
                                                   const double vm,
                                                   const double am,
                                                   const double jm);

  /*
   This function calculates the acceleration_phase time "ta" which is required
   to move the joint to new position equal to the current position plus a
   position difference "DP" starting with initial velocity "v"
  */
  static double calculate_const_acc_time(const double Dp, const double v,
                                         const double vm, const double am,
                                         const double jm);

  /*
  This function calculates the following variables to reach the final Velocity
  "vf"  starting with initial_vel "v0":
   1. the minimum position required to reach the final Velocity "vf"
   2. the acceleration "acc" that has been reached to reach the final Velocity
  "vf" starting with "v0"
   3. phases times to reach the final Velocity "vf": jerk_phase time "tj" and
  acceleration_phase time "ta"
  */
  static std::vector<double>
  calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(
      const double v0, const double vf, const double vm, const double am,
      const double jm);

  /*
  this function selects a motion profile for a trajectory segment in case of the
  final veoclity equals the starting velocity, both equal "v"
  */
  static std::vector<double> equal_vel_case_planning(const double pos_diff,
                                                     const double v,
                                                     const double abs_max_vel,
                                                     const double abs_max_acc,
                                                     const double abs_max_jrk);

  /*
  this function selects a motion profile for a trajectory segment with a given
  start and end velocities/positions, considering the start and end
  accelerations/jerks are zeros!
  */
  static std::vector<double> traj_segment_planning(
      const double p_start, const double p_end, const double abs_v_start,
      const double abs_v_end, const double abs_max_vel,
      const double abs_max_acc, const double abs_max_jrk);

  /*
   This function assigns jerk sign for each phase of the segment based on the
   motion type (+ve/-ve)
   */
  static std::vector<double> assign_jerk_sign_According_to_motion_type(
      const double p_start, const double p_end, const double v_start,
      const double v_end, const double p_max, const double v_max,
      const double a_max, const double j_max);

  /* '''
   this function calculates the jerk_value && the duration associated with each
   phase of the segment
   '''
   */
  static std::vector<JerkTimePair> calculate_jerk_sign_and_duration(
      const double p_start, const double p_end, const double v_start,
      const double v_end, const double p_max, const double v_max,
      const double a_max, const double j_max);

  /*
  '''
   This function selects a motion profile for a general trajectory segment with
  a given start/end velocities/positions considering the start and end
  accelerations/jerks are zeros
   '''
  */
  static std::vector<PiecewiseFunction> fit_traj_segment(
      const double p_start, const double p_end, const double v_start,
      const double v_end, const double p_max, const double v_max,
      const double a_max, const double j_max);

  /*
  '''
   This function selects a motion profile for a general trajectory segment with
  a given start/end velocities/positions considering the start and end
  accelerations/jerks are zeros
   '''
  */
  static std::vector<double> fit_traj_segment_samples(
      const double p_start, const double p_end, const double v_start,
      const double v_end, const double p_max, const double v_max,
      const double a_max, const double j_max, const int nplots,
      EigenDRef<Eigen::VectorXd> &time_list,
      EigenDRef<Eigen::VectorXd> &pos_list,
      EigenDRef<Eigen::VectorXd> &vel_list,
      EigenDRef<Eigen::VectorXd> &acc_list,
      EigenDRef<Eigen::VectorXd> &jerk_lis);
};

}  // namespace kinematics_lib

#endif