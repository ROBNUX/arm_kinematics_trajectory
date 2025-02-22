
/*
 * File:   base_rot_trajectory.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 * Created on April 3, 2021, 5:06 PM
 */

#ifndef BASE_ROT_TRAJECTORY_HPP
#define BASE_ROT_TRAJECTORY_HPP
#include <memory>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "kdl_common/rotation.hpp"
#include "kdl_common/vec.hpp"
#include "trajectory/base_profile.hpp"
#include "trajectory/trajectory_exportdecl.h"
using namespace ROBNUXLogging;

namespace kinematics_lib {
/**
 * An abstract class that implements a trajectory about robot orientations
 */
class BaseRotTrajectory {
 public:
  //! constructor for initializing variables
  BaseRotTrajectory()
      : duration_(0), dist_(0), isProfSet_(false), planDone_(false) {}

  //! destructor
  virtual ~BaseRotTrajectory() {}
  /*
   * @brief  determine whether this trajectory will be rounding
   * with the next trajectory segment
   */
  /*
   virtual bool isRounding() {
       return this->isRounding_;
   }
   */

  /*
   * @brief  time durtion of this trajectory segment
   */
  virtual double Duration() const { return this->duration_; }

  /*
   * @brief  translational dist (meters) if the translation > 0,
   * otherwise it is rotational distance (angles)
   * at time
   */
  virtual double Dist() const { return this->dist_; }

  /*
   *  @brief SyncDist() to scale the entire trajectory
   * (e.g., if robot has to make large rotation, while translation
   * is very small,
   * then translation time has to scaled larger to be sync with rotation)
   */
  virtual bool syncDuration(double duration) {
    std::ostringstream ss;
    if (!this->planDone_ || !this->rot_prof_ || duration_ > duration) {
      ss << "The initial plan hasn't been done yet,"
         << "can not sync duration, in function " << __FUNCTION__ << ", line "
         << __LINE__ << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    // let profile object to finish the sync operation
    bool ret = this->rot_prof_->syncDuration(duration);
    ss.str("");
    ss << "rot_prof sync duration " << ret << std::endl;
    LOG_INFO(ss);
    if (duration_ >
        MIN_TRAJ_DURATION) {  // if duration_ is successfully scaled to duration
      duration_ = duration;
    }
    return ret;
  }

  /*
   * @brief set up boundary conditions, given start_rv
   * (start orientation and angular velocity)
   * and end_rv (end orientation and angular velocity)
   */

  virtual bool setBoundaryCond(const Quaternion &start_q,
                               const Quaternion &start_qdot,
                               const Quaternion &end_q,
                               const Quaternion &end_qdot) = 0;

  /*
   *  set Profile; recall that translation (x,y,z), and
   * rotation (Euler angles) might have different profile
   */
  virtual void setProfile(const std::shared_ptr<BaseProfile> &rot_prof) {
    rot_prof_ = rot_prof;
    if (rot_prof_) {
      this->isProfSet_ = true;
    }
  }

  /*
   *  Distance at time t
   */
  virtual bool Dist(const double time, double *s) {
    std::ostringstream ss;
    if (!rot_prof_ || !s) {
      ss << "rot_prof_ is null or time pointer is null "
         << "in function " << __FUNCTION__ << " at line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    double sdot, sddot;
    return this->rot_prof_->Trajectory(time, s, &sdot, &sddot);
  }

  /*
   *  compute orientation trajectory {rot, w, wdot}  at <time>
   */
  virtual bool Trajectory(const double time, Rotation *rot, Vec *w,
                          Vec *wdot) const = 0;
  /*
   *  compute orientation trajectory in euler angles {euler,
   *eulerdot, eulerddot}  at <time>
   */
  virtual bool Trajectory(const double time, EulerAngle *euler, Vec *eulerdot,
                          Vec *eulerddot) const = 0;
  /*
   *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
   */
  virtual bool Trajectory(const double time, Quaternion *q, Quaternion *qdot,
                          Quaternion *qddot) const = 0;

  /*
   * @brief get time at given distance after the starting pos, i.e.,
   * inverse function of X(time)
   */
  virtual double TimeAtDist(const double dist) const {
    std::ostringstream ss;
    if (!rot_prof_) {
      ss << "rot_prof_ is null or time pointer is null "
         << "in function " << __FUNCTION__ << " at line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return 0;
    }
    return this->rot_prof_->TimeAtDist(dist);
  }

  /*
   * @brief  return the pos/speed/acceleration at <dist> in the units of meters
   */
  virtual bool TrajectoryAtdist(const double dist, Quaternion *q,
                                Quaternion *qdot, Quaternion *qddot) const {
    double time = TimeAtDist(dist);
    return Trajectory(time, q, qdot, qddot);
  }

  /*
   *  whether this trajectory has been successfully planed
   */
  virtual bool isPlanDone() { return planDone_; }

 protected:
  // whether this traj will be rounding with the next
  // traj segment (i.e. continuous blending)
  // bool isRounding_;
  //  duration of this  trajectory
  double duration_;
  // traveled rotational distance  (angle or shortest arc
  // distance on quaternion) of this translational trajectory
  double dist_;
  //! motion profiles: recall each BaseProfile gives pos(t),
  // vel(t), acc(t) of 1-D motion
  std::shared_ptr<BaseProfile> rot_prof_;

  // whether the required profile has been set
  bool isProfSet_;
  // whether planning has been completed
  bool planDone_;
  // whether clean profile
  // bool cleanProfile_;
};

}  // namespace kinematics_lib

#endif /* BASE_ROT_TRAJECTORY_HPP */
