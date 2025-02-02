#include "trajectory/shortest_quat_trajectory.hpp"

namespace kinematics_lib {

ShortestQuatTrajectory::ShortestQuatTrajectory()
    : BaseRotTrajectory(), tp_(eZYX) {}

ShortestQuatTrajectory::ShortestQuatTrajectory(const EulerType &tp)
    : BaseRotTrajectory(), tp_(tp) {}

bool ShortestQuatTrajectory::setBoundaryCond(const Quaternion &start_q,
                                             const Quaternion &start_qdot,
                                             const Quaternion &end_q,
                                             const Quaternion &end_qdot) {
  if (!isProfSet_ || !rot_prof_) {
    std::ostringstream ss;
    ss << "profile is not set yet, in function " << __FUNCTION__ << ", line "
       << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }

  this->sq_ = start_q;
  this->eq_ = end_q;
  this->sqdot_ = start_qdot;
  this->eqdot_ = end_qdot;

  // using spatial angular velocity, instead of sq_.inverse() * eq_;
  Quaternion diff = eq_ * sq_.inverse();

  // here dist_ is actually the minimal rotation angle, in radian
  dist_ = diff.getAngleShortestPath(&direction_);

  // first get ZYX euler angle of sq and eq
  // we use them to determine the bigger arc or small arc to take
  // for quaternion interpolation
  // Rotation rs(sq_);
  // Rotation re(eq_);
  /*
  double yaw_s, pitch_s, roll_s;
  double yaw_e, pitch_e, roll_e;
  sq_.GetEulerZYX(&yaw_s, &pitch_s, &roll_s);
  eq_.GetEulerZYX(&yaw_e, &pitch_e, &roll_e);
  Vec eu_s(roll_s, pitch_s, yaw_s);
  Vec eu_e(roll_e, pitch_e, yaw_e);
  Vec diff_eu = eu_e - eu_s;
  //Vec norm_diff_eu = diff_eu.NormalizeVec();
   // now compute the normalized rotational velocity
   // if we go from eu_s to eu_e
   Vec desired_angular;
   Rotation::GetTwistVelZYX(eu_s, diff_eu, &desired_angular);
   if (desired_angular.dot(direction_) < 0) {
         direction_ *= -1;
         dist_ = 2 * M_PI - dist_;
   }
   */
  if (dist_ > M_PI) {
    direction_ *= -1;
    dist_ = 2 * M_PI - dist_;
  }
  if (dist_ < MIN_ROT_RADIAN) {
    duration_ = 0;
  } else {
    // direction_ = diff.getAxis();

    /*
     * we replace the following body-angular velocity comments by
     * spatial-angular velocity comments
     * \dot q = 0.5 \omega * q, \omega = \omega_u {\dot \theta},
     * here \omega is spatial angular velocity, \omega_u is unit
     * spatial angular velocity, which is constant in this shortest Euler
     * trajectory
     */
    Quaternion omega_s = 2.0 * sqdot_ * sq_.inverse();
    // \dot \theta at starting quaternion
    double sv = direction_.dot(omega_s.getVirtualPart());
    Quaternion omega_e = 2.0 * eqdot_ * eq_.inverse();
    double ev = direction_.dot(omega_e.getVirtualPart());

    if (!this->rot_prof_->setBoundaryCondition(0, dist_, sv, ev)) {
      return false;
    }
    duration_ = this->rot_prof_->Duration();
  }
  // once profile is computed, then the trajectory can be completely
  // constructed
  this->planDone_ = true;
  return true;
}

/*
 *  compute orientation trajectory {rot, w, wdot}  at <time>
 */
bool ShortestQuatTrajectory::Trajectory(const double time, Rotation *rot,
                                        Vec *w, Vec *wdot) const {
  if (!rot || !w || !wdot) {
    std::cout << "input pointers to " << __FUNCTION__ << " is null"
              << std::endl;
    return false;
  }
  double s, sdot, sddot;
  Quaternion p = sq_;
  *rot = Rotation(p);
  *w = Vec::Zero();
  *wdot = Vec::Zero();
  if (duration_ > MIN_TRAJ_DURATION) {
    bool ret = this->rot_prof_->Trajectory(time, &s, &sdot, &sddot);
    if (ret) {
      p.setRealPart(cos(s / 2.0));
      p.setVirtualPart(sin(s / 2.0) * direction_);
      p = p * sq_;
      *rot = Rotation(p);
      *w = sdot * direction_;      // spatial angular velocity
      *wdot = sddot * direction_;  // spatial angular acceleration
    }
    return ret;
  }
  return true;
}
/*
 *  compute orientation trajectory in euler angles
 * {euler, eulerdot, eulerddot}  at <time>
 */
bool ShortestQuatTrajectory::Trajectory(const double time, EulerAngle *euler,
                                        Vec *eulerdot, Vec *eulerddot) const {
  if (!euler || !eulerdot || !eulerddot) {
    std::cout << "input pointers to " << __FUNCTION__ << " is null"
              << std::endl;
    return false;
  }
  double s, sdot, sddot;
  euler->setType(tp_);
  Rotation r1(sq_);
  double A, B, C;  // ZYX or ZYZ Euler angles
  if (this->tp_ == eZYX) {
    if (r1.GetEulerZYX(&C, &B, &A)) {
      euler->set_x(A);  // rotation about x axis
      euler->set_y(B);  // rotation about y axis
      euler->set_z(C);  // C is rotation about z axis
    }
  } else {
    if (r1.GetEulerZYZ(&A, &B, &C)) {
      euler->set_x(A);
      euler->set_y(B);
      euler->set_z(C);
    }
  }
  *eulerdot = Vec::Zero();
  *eulerddot = Vec::Zero();

  if (duration_ > MIN_TRAJ_DURATION) {
    bool ret = this->rot_prof_->Trajectory(time, &s, &sdot, &sddot);
    if (ret) {
      Quaternion p;
      p.setRealPart(cos(s / 2.0));
      p.setVirtualPart(sin(s / 2.0) * direction_);
      p = p * sq_;
      Rotation r(p);
      Vec w = sdot * direction_;
      Vec wdot = sddot * direction_;

      if (this->tp_ == eZYX) {
        if (r.GetEulerZYX(&C, &B, &A)) {
          euler->set_x(A);  // rotation about x axis
          euler->set_y(B);  // rotation about y axis
          euler->set_z(C);  // rotation about z axis
          if (Rotation::GetEulerVelZYX(*euler, w, eulerdot)) {
            if (Rotation::GetEulerAccZYX(*euler, *eulerdot, wdot, eulerddot)) {
              return true;
            }
          }
        }
      } else {
        if (r.GetEulerZYZ(&A, &B, &C)) {
          euler->set_x(A);
          euler->set_y(B);
          euler->set_z(C);
          if (Rotation::GetEulerVelZYZ(*euler, w, eulerdot)) {
            if (Rotation::GetEulerAccZYZ(*euler, *eulerdot, wdot, eulerddot)) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }
  return true;
}
/*
 *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
 */
bool ShortestQuatTrajectory::Trajectory(const double time, Quaternion *q,
                                        Quaternion *qdot,
                                        Quaternion *qddot) const {
  if (!q || !qdot || !qddot) {
    std::cout << "input pointers to " << __FUNCTION__ << " is null"
              << std::endl;
    return false;
  }
  double s, sdot, sddot;
  *q = sq_;
  *qdot = Quaternion::getZero();
  *qddot = Quaternion::getZero();
  if (duration_ > MIN_TRAJ_DURATION) {
    bool ret = this->rot_prof_->Trajectory(time, &s, &sdot, &sddot);
    if (ret) {
      Quaternion p;
      p.setRealPart(cos(s / 2.0));
      p.setVirtualPart(sin(s / 2.0) * direction_);
      *q = p * sq_;  // sq_ * p;
      Quaternion pdot = Quaternion::getZero();
      pdot.setVirtualPart(sdot * direction_ / 2.0);
      *qdot = pdot * (*q);
      Quaternion pddot = Quaternion::getZero();
      pddot.setRealPart(-sdot * sdot / 4.0);
      pddot.setVirtualPart(sddot * direction_ / 2.0);
      *qddot = pddot * (*q);
    }
    return ret;
  }
  return true;
}

}  // namespace kinematics_lib
