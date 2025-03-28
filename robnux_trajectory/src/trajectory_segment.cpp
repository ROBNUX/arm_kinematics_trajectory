#include "robnux_trajectory/trajectory_segment.hpp"
namespace kinematics_lib {

TrajectorySegment::TrajectorySegment()
    : trans_T_(NULL),
      rot_T_(NULL),
      duration_(0),
      dist_(0),
      planDone_(false),
      transDominated_(true) {
  isTailBlended_ = false;
}

TrajectorySegment::~TrajectorySegment() {
  if (trans_T_) {
    trans_T_.reset();
    trans_T_ = nullptr;
  }
  if (rot_T_) {
    rot_T_.reset();
    rot_T_ = nullptr;
  }
}

bool TrajectorySegment::InitializeTrajectoryPlanners(
    const std::shared_ptr<BaseTransTrajectory> &transPlanner,
    const std::shared_ptr<BaseRotTrajectory> &rotPlanner) {
  std::ostringstream ss;
  if (!transPlanner || !rotPlanner) {
    ss.str("");
    ss << __FUNCTION__ << ":" << __LINE__
       << ": translational or rotational trajectory"
       << "planner is not initialized in TrajectorySegment" << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  trans_T_ = transPlanner;
  rot_T_ = rotPlanner;
  return true;
}

bool TrajectorySegment::setBoundCond(const Pose &start_pos,
                                     const Twist &start_vel,
                                     const Pose &end_pos,
                                     const Twist &end_vel) {
  std::ostringstream ss;
  if (!trans_T_ || !rot_T_) {
    ss.str("");
    ss << "translational or rotational trajectory planner"
       << "is not initilized in setBoundCond" << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  int ret = armMap_->CartToJnt(start_pos, &sq_);
  if (ret < 0) {
    ss.str("");
    ss << "IK for computing starting jnt fails in " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  // get start pose branch and turns
  start_pos.getBranchFlags(&branchFlags_);
  start_pos.getJointTurns(&jointTurns_);
  // get end pose branch and turns
  std::vector<int> goalBranchFlags;  //, goalJointTurns;
  end_pos.getBranchFlags(&goalBranchFlags);
  // end_pos.getJointTurns(&goalJointTurns);
  if (!CompareRobotConfigTurn(branchFlags_, goalBranchFlags)
      //|| !CompareRobotConfigTurn(jointTurns_, goalJointTurns)
  ) {
    ss.str("");
    ss << "config not matching, at function " << __FUNCTION__ << ", line "
       << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }

  trans_T_->setBoundaryCond(start_pos.getTranslation(),
                            start_vel.getLinearVel(), end_pos.getTranslation(),
                            end_vel.getLinearVel());
  Quaternion sq = start_pos.getQuaternion();
  Vec omega_s = start_vel.getAngularVel();
  // next recall a formula about derivative of a unit quaternion for spatial
  // rotational velocity \dot q = 0.5 \omega * q,  and \omega =  \omega_u {\dot
  // \theta},  here \omega is angular velocity, and \omega_u is unit angular
  // velocity, which is constant because we take shortest Euler trajectory
  Quaternion tmp;
  tmp.setVirtualPart(0.5 * omega_s);
  Quaternion sqdot = tmp * sq;

  Quaternion eq = end_pos.getQuaternion();
  Vec omega_e = end_vel.getAngularVel();
  tmp = Quaternion();  // default
  tmp.setVirtualPart(0.5 * omega_e);
  Quaternion eqdot = tmp * eq;
  rot_T_->setBoundaryCond(sq, sqdot, eq, eqdot);
  double transDuration = trans_T_->Duration();
  double rotDuration = rot_T_->Duration();
  if (transDuration > rotDuration) {
    duration_ = transDuration;
    if (!rot_T_->syncDuration(duration_)) {
      ss.str("");
      ss << "rot_T_ sync failed " << __FUNCTION__ << " line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    dist_ = trans_T_->Dist();
  } else {
    transDominated_ = false;
    duration_ = rotDuration;
    if (!trans_T_->syncDuration(duration_)) {
      ss.str("");
      ss << "trans_T_ sync failed " << __FUNCTION__ << " line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
    dist_ = rot_T_->Dist();
  }
  planDone_ = true;
  return true;
}

void TrajectorySegment::SetTailBlendDist(const double b_dist) {
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
  if (transDominated_) {
    tailBlendTime_ = trans_T_->TimeAtDist(dist_ - tailBlendDist_);
  } else {
    tailBlendTime_ = rot_T_->TimeAtDist(dist_ - tailBlendDist_);
  }
}
void TrajectorySegment::SetTailBlendPercent(const double b_perc) {
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
  if (transDominated_) {
    tailBlendTime_ = trans_T_->TimeAtDist(dist_ - tailBlendDist_);
  } else {
    tailBlendTime_ = rot_T_->TimeAtDist(dist_ - tailBlendDist_);
  }
}

double TrajectorySegment::Duration() const { return duration_; }

double TrajectorySegment::Dist() const { return dist_; }

bool TrajectorySegment::Trajectory(const double time, Pose *ps, Twist *tw,
                                   Twist *acctw) const {
  if (!planDone_ || !ps || !tw || !acctw) {
    std::ostringstream ss;
    ss << "Trajectory planning is not done or input pointers to "
       << __FUNCTION__ << " is null" << std::endl;
    LOG_ERROR(ss);
    return false;
  }

  Vec p, pdot, pddot;
  bool ret = this->trans_T_->Trajectory(time, &p, &pdot, &pddot);
  Rotation r;
  Vec w, wdot;
  ret = ret && this->rot_T_->Trajectory(time, &r, &w, &wdot);
  if (ret) {
    ps->setTranslation(p);
    ps->setRotation(r);
    ps->setBranchFlags(branchFlags_);
    ps->setJointTurns(jointTurns_);
    tw->setLinearVel(pdot);
    tw->setAngularVel(w);
    acctw->setLinearVel(pddot);
    acctw->setAngularVel(wdot);
  }
  return ret;
}

bool TrajectorySegment::TranslationalTrajectory(const double time, Vec *p,
                                                Vec *pdot, Vec *pddot) const {
  if (!planDone_) {
    std::ostringstream ss;
    ss << "Trajectory planning is not done" << __FUNCTION__ << " is null"
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  return this->trans_T_->Trajectory(time, p, pdot, pddot);
}

/*
 * rotational component in Rotation matrix
 */
bool TrajectorySegment::RotationalTrajectory(const double time, Rotation *rot,
                                             Vec *w, Vec *wdot) const {
  if (!planDone_) {
    std::ostringstream ss;
    ss << "Trajectory planning is not done" << __FUNCTION__ << " is null"
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  return this->rot_T_->Trajectory(time, rot, w, wdot);
}

/*
 *  compute orientation trajectory in euler angles {euler,
 *eulerdot, eulerddot}  at <time>
 */
bool TrajectorySegment::RotationalTrajectory(const double time,
                                             EulerAngle *euler, Vec *eulerdot,
                                             Vec *eulerddot) const {
  if (!planDone_) {
    std::ostringstream ss;
    ss << "Trajectory planning is not done" << __FUNCTION__ << " is null"
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  return this->rot_T_->Trajectory(time, euler, eulerdot, eulerddot);
}
/*
 *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
 */
bool TrajectorySegment::RotationalTrajectory(const double time, Quaternion *q,
                                             Quaternion *qdot,
                                             Quaternion *qddot) const {
  if (!planDone_) {
    std::ostringstream ss;
    ss << "Trajectory planning is not done" << __FUNCTION__ << " is null"
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  return this->rot_T_->Trajectory(time, q, qdot, qddot);
}

bool TrajectorySegment::Trajectory(const double time, Eigen::VectorXd *jp,
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
  Pose ps;
  Twist tw, acctw;
  if (!Trajectory(time, &ps, &tw, &acctw)) {
    ss << "Cartesian Trajectory Segment planning fails in " << __FUNCTION__
       << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  int ret = armMap_->CartToJnt(ps, tw, jp, jv);
  if (ret >= 0) {
    *ja = Eigen::VectorXd::Zero(jp->size());
    return true;
  } else {
    ss.str("");
    ss << "IK fails with error code  " << ret << " in function " << __FUNCTION__
       << " , line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
  }
  return false;
}

bool TrajectorySegment::RelTrajectory(const double time, Eigen::VectorXd *jp,
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
  if (!Trajectory(time, jp, jv, ja)) {
    return false;
  }
  (*jp) -= sq_;
  return true;
}

}  // namespace kinematics_lib
