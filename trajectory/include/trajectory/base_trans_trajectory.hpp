
/*
 * File:   base_trans_trajectory.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 * Created on April 3, 2021, 5:06 PM
 */

#ifndef KINEMATICS_LIB_BASE_TRANS_TRAJECTORY_HPP
#define KINEMATICS_LIB_BASE_TRANS_TRAJECTORY_HPP
#include <memory>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "kdl_common/vec.hpp"
#include "trajectory/base_profile.hpp"
#include "trajectory/trajectory_exportdecl.h"
using namespace ROBNUXLogging;

namespace kinematics_lib {
/**
 * An abstract class that implements translational motions
 */
class BaseTransTrajectory {
 public:
  //! constructor for initializing variables
  BaseTransTrajectory()
      : duration_(0), dist_(0), isProfSet_(false), planDone_(false) {}

  //! destructor
  virtual ~BaseTransTrajectory() {}

  /*
   * @brief  time durtion of this trajectory segment
   */
  virtual double Duration() const { return duration_; }

  /*
   * @brief  translational dist (meters) if the translation > 0, otherwise it is
   * rotational distance (angles) at time
   */
  virtual double Dist() const { return dist_; }

  /*
   *  set Profile; recall that translation (x,y,z), and rotation (Euler angles)
   * might have different profile
   */
  virtual void setProfile(const std::shared_ptr<BaseProfile> &trans_prof) {
    trans_prof_ = trans_prof;
    if (trans_prof_) {
      this->isProfSet_ = true;
    }
  }

  /*
   * @brief set up boundary conditions, given start_pv (pos and vel),
   * and end_pv (end pos and vel)
   */
  virtual bool setBoundaryCond(const Vec &start_pos, const Vec &start_vel,
                               const Vec &end_pos, const Vec &end_vel) = 0;

  /*
   *  @brief SyncDuration() to scale the entire trajectory (e.g., if robot has
   * to make large rotation, while translation is very small, then translation
   * time has to scaled larger to be sync with rotation)
   */
  virtual bool syncDuration(double duration) {
    std::ostringstream ss;
    // recall duration_ is already optimal, so can only be set larger
    if (!this->planDone_ || duration < duration_ || !this->trans_prof_) {
      ss << "The initial plan hasn't been done yet, "
         << "or the duration to be synced is smaller than optimal,"
         << " or trans_prof_ is null, "
         << "can not sync duration, in function " << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    // let profile object to finish the sync operation
    bool ret = this->trans_prof_->syncDuration(duration);
    if (ret) {  // if duration_ is successfully scaled to duration
      duration_ = duration;
    }
    return ret;
  }

  /*
   *  Distance at time t
   */
  virtual bool Dist(const double time, double *s) {
    std::ostringstream ss;
    if (!trans_prof_ || !s) {
      ss << "trans_prof_ is null or time pointer is null "
         << "in function " << __FUNCTION__ << " at line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    double sdot, sddot;
    return this->trans_prof_->Trajectory(time, s, &sdot, &sddot);
  }

  /*
   *  compute trajectory {Pos, Vel,  Acc} at <time>
   */
  virtual bool Trajectory(const double time, Vec *p, Vec *pdot,
                          Vec *ppdot) const = 0;

  /*
   * @brief get time at given distance after the starting pos, i.e.,
   * inverse function of X(time)
   */
  virtual double TimeAtDist(const double dist) const {
    if (!trans_prof_) {
      std::cout << "trans_prof_ is null or time pointer is null "
                << "in function " << __FUNCTION__ << " at line " << __LINE__
                << std::endl;
      return 0;
    }
    return this->trans_prof_->TimeAtDist(dist);
  }

  /*
   * @brief  return the pos/speed/acceleration at <dist> in the units of meters
   */
  virtual bool TrajectoryAtdist(const double dist, Vec *p, Vec *pdot,
                                Vec *pddot) const {
    double time = TimeAtDist(dist);
    return Trajectory(time, p, pdot, pddot);
  }

  /*
   * @brief  is the planning already done
   */
  virtual bool isPlanDone() { return planDone_; }

 protected:
  // whether this traj will be rounding with the
  // next traj segment (i.e. continuous blending)
  // bool isRounding_;
  //  duration of this translational trajectory
  double duration_;
  // traveled distance of this translational trajectory
  double dist_;
  // velocity profile for each translational DoF
  std::shared_ptr<BaseProfile> trans_prof_;

  // whether the required profile has been set
  bool isProfSet_;
  // whether planning has been completed
  bool planDone_;
  // whether clean profile pointer
  // bool cleanProfile_;
  // rounding_dist
  // double rounding_dist_;
  // which type of translational trajectory
  // TransType type_;
};

}  // namespace kinematics_lib
#endif  //  KINEMATICS_LIB_BASE_TRANS_TRAJECTORY_HPP
