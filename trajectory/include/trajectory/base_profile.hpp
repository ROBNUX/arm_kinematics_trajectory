/*
 * File:   base_profile.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 3, 2021, 12:19 PM
 */
#ifndef BASE_PROFILE_HPP
#define BASE_PROFILE_HPP
#include <limits>
#include <ostream>

#include "simple_motion_logger/Logger.h"
#include "kdl_common/common_constants.hpp"
#include "trajectory/trajectory_exportdecl.h"
using namespace ROBNUXLogging;

namespace kinematics_lib {
/**
 * BaseProfile define interfaces: inlcuding x(t), xdot(t), xddot(t), xdddot(t)
 * as the function of time t. It defines the how fast a point  moves on a  1-D
 * path. Here x can be arc length paramter, or individual cartesian or joint
 * coordinates
 *
 */
class BaseProfile {
 public:
  BaseProfile()
      : max_pos_(std::numeric_limits<double>::max()),
        max_vel_(0),
        max_acc_(0),
        max_jerk_(0),
        scale_(1.0),
        duration_(0),
        dist_(0),
        bsetConstraints_(false),
        initialized_(false) {}

  /*
   * @brief: set boundary conditions needed to find a time optimal profile
   */
  virtual bool setBoundaryCondition(const double pos_start,
                                    const double pos_end,
                                    const double vel_start,
                                    const double vel_end) = 0;

  /*
   * @brief  set constraints
   */
  virtual void setConstraints(
      const double max_vel, const double max_acc, const double max_jerk,
      const double max_pos = std::numeric_limits<double>::max()) {
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jerk_ = max_jerk;
    max_pos_ = max_pos;
    bsetConstraints_ = true;
  }

  // main (forward Profile) APIs  of profile
  /*
   * @brief returns the position/speed/acceleration at <time> in the units of
   * seconds
   */
  virtual bool Trajectory(const double time, double *X, double *Xdot,
                          double *Xddot) const = 0;

  /*
   * @brief get time at given distance after the starting pos, i.e.,
   * inverse function of X(time), Note this applies canonical inverse
   * which means assume scale_ = 1.0
   */
  virtual double TimeAtDist(const double dist) const = 0;

  /*
   * @brief  return the pos/speed/acceleration at <dist> in the units of meters
   */
  virtual bool TrajectoryAtdist(const double dist, double *X, double *Xdot,
                                double *Xddot) const {
    double time = scale_ * TimeAtDist(dist);
    return Trajectory(time, X, Xdot, Xddot);
  }
  //!  properties
  /*
   * @brief, obtain the constraints
   */
  virtual double getMaxVel() const { return max_vel_; }
  virtual double getMaxAcc() const { return max_acc_; }
  virtual double getMaxJerk() const { return max_jerk_; }

  /*
   * @brief duration of the profile
   */
  virtual double Duration() const { return duration_; }

  /*
   *  @brief  synDuration with a larger duration than the optimal (minimal one)
   */
  virtual bool syncDuration(const double duration);

  /*
   * @brief overall traveled distance
   */
  virtual double Dist() const { return dist_; }

  //! default destructor
  virtual ~BaseProfile() {}

 protected:
  // maximal vel/acc/jerk
  double max_pos_;  // means pos should be in [-max_pos_, max_pos_]
  double max_vel_;
  double max_acc_;
  double max_jerk_;
  double scale_;  // for sync time with other profile from other component
  // duration_ of this profile, and optimal (minimal duration)
  // scale_ = new_duration / this->duration
  double duration_;
  double dist_;
  bool bsetConstraints_;  // whether constraints has been set
  bool initialized_;      // whether this profile has been initialized
};

}  // namespace kinematics_lib
#endif  //  BASE_PROFILE_HPP