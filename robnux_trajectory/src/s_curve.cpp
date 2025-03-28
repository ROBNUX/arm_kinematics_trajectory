#include "robnux_trajectory/s_curve.hpp"
using namespace ROBNUXLogging;
namespace kinematics_lib {
SCurveProfile::SCurveProfile() : BaseProfile() {}

/*
 * @brief: set boundary conditions needed to find a time optimal profile
 */
bool SCurveProfile::setBoundaryCondition(const double pos_start,
                                         const double pos_end,
                                         const double vel_start,
                                         const double vel_end) {
  std::ostringstream ss;
  if (!bsetConstraints_) {
    ss << "max vel/acc/jerk limits not set yet"
       << "can not compute profile based upon boundary condition" << std::endl;
    LOG_ERROR(ss);
    return false;  // todo: convert to error code in the future
  }
  start_pos_ = pos_start;
  end_pos_ = pos_end;
  start_vel_ = vel_start;
  end_vel_ = vel_end;
  dist_ = fabs(end_pos_ - start_pos_);
  if (dist_ < MIN_SCURVE_DIST) {
    duration_ = 0;
    this->initialized_ = true;
    return true;
  }
  trajs_ = SegmentPlanning::fit_traj_segment(pos_start, pos_end, vel_start,
                                             vel_end, max_pos_, max_vel_,
                                             max_acc_, max_jerk_);
  if (trajs_.size() ==
      4) {  // means all functions (pos, vel, acc, jerk) has been planned out
    double traj_duration;
    if (trajs_[0].duration(traj_duration)) {
      duration_ = traj_duration;
      initialized_ = true;
      trajs_[0].lowest_t_bound(lower_t_bound_);
      trajs_[0].largest_t_bound(up_t_bound_);
    }
  }
  return initialized_;
}

bool SCurveProfile::Trajectory(const double t, double *X, double *Xdot,
                               double *Xddot) const {
  if (!initialized_) {
    return false;
  }
  if (duration_ < MIN_TRAJ_DURATION) {
    *X = start_pos_;
    *Xdot = 0;
    *Xddot = 0;
    return true;
  }
  double new_t = std::min(std::max(t / scale_, lower_t_bound_), up_t_bound_);
  *X = trajs_[0].evaluate(new_t);
  *Xdot = trajs_[1].evaluate(new_t);
  *Xddot = trajs_[2].evaluate(new_t);
  return true;
}

double SCurveProfile::TimeAtDist(const double dist) const {
  std::ostringstream ss;
  if (!this->initialized_) {
    ss.str("");
    ss << __FUNCTION__ << ":" << __LINE__
       << "s curve is not initialized,  function " << __FUNCTION__
       << " can not be executed" << std::endl;
    LOG_ERROR(ss);
    return 0;
  }
  if (dist < 0) {
    return 0;
  } else if (dist >= dist_ - MIN_SCURVE_DIST) {
    return duration_;
  } else {
    double time;
    if (trajs_[0].TimeAtDist(dist, time)) {
      return time;
    } else {
      ss.str("");
      ss << __FUNCTION__ << ":" << __LINE__
         << " error calling position(p.w. function)  "
         << " -> TimAtDist() fails" << std::endl;
      LOG_ERROR(ss);
      return 0;
    }
  }
}

}  // namespace kinematics_lib
