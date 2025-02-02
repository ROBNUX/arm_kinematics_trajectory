#include "trajectory/line_trajectory.hpp"

namespace kinematics_lib {
LineTrajectory::LineTrajectory() : BaseTransTrajectory() {}

// here start_acc and end_acc are not used, and are assuming
// as 0. Because for linear trajectory,  start_acc must be in the
// direction of end_pos - start_pos, similarly, end_acc also
// should be in the same direction. For simplicity of s-curve profile,
// we always have ddot(s) =0 at start and end point.
// therefore, we assume start_acc =0, and end_accc =0
bool LineTrajectory::setBoundaryCond(const Vec &start_pos, const Vec &start_vel,
                                     const Vec &end_pos, const Vec &end_vel) {
  if (!isProfSet_ || !trans_prof_) {
    //        std::ostringstream ss;
    //        ss << "profile is not set yet, in function "
    //        << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    //        LOG_ERROR(ss);
    return false;
  }

  this->start_ps_ = start_pos;
  this->start_vel_ = start_vel;
  this->end_ps_ = end_pos;
  this->end_vel_ = end_vel;
  Vec line = end_ps_ - start_ps_;
  dist_ = line.Norm();
  if (dist_ < MIN_TRANS_DIST) {
    direction_ = Vec(0, 0, 1);  // default to z axis
    duration_ = 0; /*************** TODO: needs to carefully handle this case,
                      and make sure it returns the current pos **/
  } else {
    direction_ = line / dist_;

    double sv = direction_.dot(start_vel);
    double ev = direction_.dot(end_vel);
    if (!this->trans_prof_->setBoundaryCondition(0, dist_, sv, ev)) {
      return false;
    }
    duration_ = this->trans_prof_->Duration();
    // once profile is computed, then the trajectory
    // can be completely constructed
  }
  this->planDone_ = true;
  return true;
}

bool LineTrajectory::Trajectory(const double time, Vec *p, Vec *pdot,
                                Vec *pddot) const {
  if (!this->planDone_ || !p || !pdot || !pddot) {
    return false;
  } else {
    // default values
    *p = start_ps_;
    *pdot = start_vel_;
    *pddot = Vec::Zero();
    if (duration_ > MIN_TRAJ_DURATION) {
      double X, Xdot, Xddot;
      this->trans_prof_->Trajectory(time, &X, &Xdot, &Xddot);
      *p = direction_ * X + start_ps_;
      *pdot = direction_ * Xdot;
      *pddot = direction_ * Xddot;
    }
    return true;
  }
}

}  // namespace kinematics_lib
