#include "trajectory/trajectory.hpp"
namespace kinematics_lib {
JntTrajectory::JntTrajectory(const size_t numJnts)
    : numJnts_(numJnts),
      duration_(0),
      dist_(0),
      isProfSet_(false),
      planDone_(false) {
  isTailBlended_ = false;
  tailBlendDist_ = 0;
  tailBlendTime_ = 0;
  armMap_ = nullptr;
}

bool JntTrajectory::Trajectory(const double time, Pose *p, Twist *v,
                               Twist *a) const {
  std::ostringstream ss;
  if (!p || !v || !a) {
    ss << "input p, v, a are null, can not generate trajectory in function "
       << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!planDone_) {
    ss.str("");
    ss << "planning is not done, can not generate trajectory in function "
       << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  Eigen::VectorXd q, qdot, qddot;
  Pose ps;
  if (this->Trajectory(time, &q, &qdot, &qddot)) {
    int ret = armMap_->JntToCart(q, qdot, &ps, v);
    if (ret >= 0) {
      if (ps.getFrame(p)) {
        *a = Twist::Zero();
        return true;
      }
    } else {
      ss.str("");
      ss << "Jnt Trajectory function, FK fails with error code  " << ret
         << " in function " << __FUNCTION__ << " , line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
    }
  } else {
    ss.str("");
    ss << "Trajectory function fails, in function " << __FUNCTION__
       << " , line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
  }
  return false;
}

bool JntTrajectory::setBoundaryCond(const Eigen::VectorXd &start_jp,
                                    const Eigen::VectorXd &start_jv,
                                    const Eigen::VectorXd &end_jp,
                                    const Eigen::VectorXd &end_jv) {
  std::ostringstream ss;
  if (!isProfSet_) {
    ss << "JntTrajectory has no profile set in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_INFO(ss);
    return false;
  }
  if (start_jp.size() != numJnts_ || start_jv.size() != numJnts_ ||
      end_jp.size() != numJnts_ || end_jv.size() != numJnts_ ||
      jnt_prof_.empty() || !armMap_) {
    ss.str("");
    ss << "Input parameters have wrong dimension or nullptr" << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_INFO(ss);
    return false;
  }
  // Eigen::VectorXd start_eig_pos, end_eig_pos, start_eig_vel, end_eig_vel;
  sq_ = start_jp;
  eq_ = end_jp;
  sqdot_ = start_jv;
  eqdot_ = end_jv;

  if (sqdot_.norm() > K_EPSILON || eqdot_.norm() > K_EPSILON) {
    ss.str("");
    ss << "JntTrajectory requires start and end vel being 0 " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_INFO(ss);
    return false;
  }
  jntVec_ = eq_ - sq_;
  dist_ = jntVec_.norm();
  if (dist_ < MIN_TRANS_DIST) {
    duration_ = 0; /*************** TODO: needs to carefully handle this case,
                      and make sure it returns the current pos **/
  } else {
    jntVec_ /= dist_;  // normalize
    double maxDuration = 0;
    for (size_t i = 0; i < numJnts_; i++) {
      // tmp_dist +=  j_direction_[i] * j_direction_[i];
      // we need to compute the expected minimal duration of each joint
      // and then pick the index and joint diff of the slowest one
      jnt_prof_[i]->setBoundaryCondition(0, dist_, 0, 0);
      if (jnt_prof_[i]->Duration() > maxDuration) {
        maxDuration = jnt_prof_[i]->Duration();
        slowest_jnt_index_ = i;
      }
    }
    duration_ = maxDuration;
  }

  planDone_ = true;
  return true;
}

bool JntTrajectory::setBoundaryCond(const Pose &start_pos,
                                    const Twist &start_vel, const Pose &end_pos,
                                    const Twist &end_vel) {
  std::ostringstream ss;
  if (!isProfSet_) {
    ss.str("");
    ss << "JntTrajectory has no profile set in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!armMap_) {
    ss.str("");
    ss << "The FK/IK map of robot is not set yet, in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  Eigen::VectorXd sj, sv, ej, ev;
  if (armMap_->CartToJnt(start_pos, start_vel, &sj, &sv) < 0) {
    ss.str("");
    ss << "The IK map fails in function " << __FUNCTION__ << ", at line "
       << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (armMap_->CartToJnt(end_pos, end_vel, &ej, &ev) < 0) {
    ss.str("");
    ss << "The IK map fails in function " << __FUNCTION__ << ", at line "
       << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }

  return setBoundaryCond(sj, sv, ej, ev);
}

bool JntTrajectory::setBoundaryCond(const Pose &start_pos, const Vec &sp_dot,
                                    const Quaternion &sq_dot,
                                    const Pose &end_pos, const Vec &ep_dot,
                                    const Quaternion &eq_dot) {
  Quaternion sq = start_pos.getQuaternion();
  Quaternion tmp = 2 * sq_dot * sq.inverse();
  Vec w = tmp.getVirtualPart();
  Twist start_vel(sp_dot, w);

  Quaternion eq = end_pos.getQuaternion();
  tmp = 2 * eq_dot * eq.inverse();
  w = tmp.getVirtualPart();
  Twist end_vel(ep_dot, w);
  return setBoundaryCond(start_pos, start_vel, end_pos, end_vel);
}
bool JntTrajectory::Trajectory(const double time, Eigen::VectorXd *jp,
                               Eigen::VectorXd *jv, Eigen::VectorXd *ja) const {
  std::ostringstream ss;
  if (!jp || !jv || !ja) {
    ss.str("");
    ss << "input jp, jv, and ja pointers are null, in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!planDone_) {
    ss.str("");
    ss << "plan is not done yet, can not call this function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (duration_ < MIN_TRAJ_DURATION) {
    *jp = sq_;
    *jv = sqdot_;
    *ja = Eigen::VectorXd::Zero(numJnts_);
    return true;
  } else {
    double X, Xdot, Xddot;
    jnt_prof_[slowest_jnt_index_]->Trajectory(time, &X, &Xdot, &Xddot);
    *jp = jntVec_ * X + sq_;
    *jv = jntVec_ * Xdot;
    *ja = jntVec_ * Xddot;
  }
  return false;
}

void JntTrajectory::SetTailBlendDist(const double b_dist) {
  std::ostringstream ss;
  if (!planDone_) {
    ss.str("");
    ss << "plan is not done yet, can not call this function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return;
  }
  tailBlendDist_ = std::min(std::max(0.0, b_dist), dist_);
  isTailBlended_ = tailBlendDist_ > 0.0;
  tailBlendTime_ =
      jnt_prof_[slowest_jnt_index_]->TimeAtDist(dist_ - tailBlendDist_);
}

void JntTrajectory::SetTailBlendPercent(const double b_perc) {
  std::ostringstream ss;
  if (!planDone_) {
    ss.str("");
    ss << "plan is not done yet, can not call this function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return;
  }
  tailBlendDist_ = dist_ * std::min(std::max(0.0, b_perc), 100.0);
  isTailBlended_ = tailBlendDist_ > 0.0;
  tailBlendTime_ =
      jnt_prof_[slowest_jnt_index_]->TimeAtDist(dist_ - tailBlendDist_);
}

bool JntTrajectory::RelTrajectory(const double time, Eigen::VectorXd *jp,
                                  Eigen::VectorXd *jv,
                                  Eigen::VectorXd *ja) const {
  std::ostringstream ss;
  if (!jp || !jv || !ja) {
    ss.str("");
    ss << "input jp, jv, and ja pointers are null, in function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!planDone_) {
    ss.str("");
    ss << "plan is not done yet, can not call this function " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (duration_ < MIN_TRAJ_DURATION) {
    *jp = Eigen::VectorXd::Zero(numJnts_);
    *jv = sqdot_;
    *ja = Eigen::VectorXd::Zero(numJnts_);
    return true;
  } else {
    double X, Xdot, Xddot;
    jnt_prof_[slowest_jnt_index_]->Trajectory(time, &X, &Xdot, &Xddot);
    *jp = jntVec_ * X;
    *jv = jntVec_ * Xdot;
    *ja = jntVec_ * Xddot;
    return true;
  }
  return false;
}

bool JntTrajectory::TranslationalTrajectory(const double time, Vec *p,
                                            Vec *pdot, Vec *pddot) const {
  if (!p || !pdot || !pddot) {
    std::cout << "input p, pdot, pddot are null, can not generate trajectory "
                 "in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  Pose ps;
  Twist v, a;
  if (Trajectory(time, &ps, &v, &a)) {
    *p = ps.getTranslation();
    *pdot = v.getLinearVel();
    *pddot = a.getLinearVel();
    return true;
  }
  return false;
}

bool JntTrajectory::RotationalTrajectory(const double time, Rotation *rot,
                                         Vec *w, Vec *wdot) const {
  std::ostringstream ss;
  if (!rot || !w || !wdot) {
    ss.str("");
    ss << "input rot, w, wdot are null, can not generate trajectory "
          "in function "
       << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  Pose ps;
  Twist v, a;
  if (Trajectory(time, &ps, &v, &a)) {
    *rot = ps.getRotation();
    *w = v.getAngularVel();
    *wdot = a.getAngularVel();
    return true;
  }
  return false;
}

bool JntTrajectory::RotationalTrajectory(const double time, EulerAngle *euler,
                                         Vec *eulerdot, Vec *eulerddot) const {
  if (!euler || !eulerdot || !eulerddot) {
    std::cout << "input rot, w, wdot are null, can not generate trajectory "
                 "in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  Rotation r;
  Vec w, wdot;
  if (RotationalTrajectory(time, &r, &w, &wdot)) {
    euler->setType(eZYX);
    double C, B, A;  // yaw, pitch, roll
    if (r.GetEulerZYX(&C, &B, &A)) {
      euler->set_x(A);  // euler=[A, B, C]
      euler->set_y(B);
      euler->set_z(C);
      if (Rotation::GetEulerVelZYX(*euler, w, eulerdot)) {
        if (Rotation::GetEulerAccZYX(*euler, *eulerdot, wdot, eulerddot)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool JntTrajectory::RotationalTrajectory(const double time, Quaternion *q,
                                         Quaternion *qdot,
                                         Quaternion *qddot) const {
  if (!q || !qdot || !qddot) {
    std::cout << "input rot, w, wdot are null, can not generate trajectory "
                 "in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  Rotation r;
  Vec w, wdot;
  if (RotationalTrajectory(time, &r, &w, &wdot)) {
    if (r.GetQuaternion(q)) {
      Quaternion tmp, tmp1;
      tmp.setRealPart(0);
      tmp.setVirtualPart(w);
      *qdot = 0.5 * tmp * (*q);
      tmp1.setRealPart(0);
      tmp1.setVirtualPart(wdot);
      *qddot = 0.5 * (tmp1 * (*q) + tmp * (*qdot));
    }
  }
  return false;
}
}  // namespace kinematics_lib
