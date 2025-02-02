#include "trajectory/PTPTrajectory.hpp"
namespace kinematics_lib {
bool PTPTrajectory::setBoundaryCond(const Eigen::VectorXd &start_jp,
                                    const Eigen::VectorXd &start_jv,
                                    const Eigen::VectorXd &end_jp,
                                    const Eigen::VectorXd &end_jv) {
  std::ostringstream ss;
  if (start_jp.size() != numJnts_ || start_jv.size() != numJnts_ ||
      end_jp.size() != numJnts_ || end_jv.size() != numJnts_) {
    ss.str("");
    ss << "Boundary conditions have wrong dimension, in function "
       << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!isProfSet_) {
    ss.str("");
    ss << "joint profile hasn't been set, can not do setBoundaryCond"
       << ", in function " << __FUNCTION__ << " at line " << __LINE__
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!armMap_) {
    ss.str("");
    std::cout << "The FK/IK map of robot is not set yet, in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  // default implementation, just assigning boundary value only
  sq_ = start_jp;
  eq_ = end_jp;
  sqdot_ = start_jv;
  eqdot_ = end_jv;

  j_direction_ = end_jp - start_jp;
  double maxDuration = 0;
  // double tmp_dist = 0;
  for (size_t i = 0; i < numJnts_; i++) {
    // tmp_dist +=  j_direction_[i] * j_direction_[i];
    // we need to compute the expected minimal duration of each joint
    // and then pick the index and joint diff of the slowest one
    jnt_prof_[i]->setBoundaryCondition(start_jp(i), end_jp(i), start_jv(i),
                                       end_jv(i));
    if (jnt_prof_[i]->Duration() > maxDuration) {
      maxDuration = jnt_prof_[i]->Duration();
      slowest_jnt_index_ = i;
    }
  }
  dist_ = fabs(j_direction_[slowest_jnt_index_]);
  duration_ = maxDuration;
  // next sink all fast joint axes so that totally they
  // move in the same time period
  // use a local flag to check whether sync has been successful
  for (size_t i = 0; i < numJnts_; i++) {
    if (i != slowest_jnt_index_) {
      if (!jnt_prof_[i]->syncDuration(duration_)) {
        std::cout << "joint profile " << i << " sync duration " << duration_
                  << " fails, original duration is " << jnt_prof_[i]->Duration()
                  << ", in function " << __FUNCTION__ << " at line " << __LINE__
                  << std::endl;
        LOG_ERROR(ss);
        return false;
      }
    }
  }
  planDone_ = true;
  return true;
}

bool PTPTrajectory::setBoundaryCond(const Pose &start_pos,
                                    const Twist &start_vel, const Pose &end_pos,
                                    const Twist &end_vel) {
  std::ostringstream ss;
  if (!isProfSet_) {
    ss.str("");
    ss << "joint profile hasn't been set, can not do setBoundaryCond"
       << ", in function " << __FUNCTION__ << " at line " << __LINE__
       << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (!armMap_) {
    ss.str("");
    std::cout << "The FK/IK map of robot is not set yet, in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  Eigen::VectorXd start_jp(numJnts_), start_jv(numJnts_), end_jp(numJnts_),
      end_jv(numJnts_);
  if (armMap_->CartToJnt(start_pos, start_vel, &start_jp, &start_jv) < 0) {
    ss.str("");
    std::cout << "The IK map of start pose/vel fails, in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  if (armMap_->CartToJnt(end_pos, end_vel, &end_jp, &end_jv) < 0) {
    ss.str("");
    std::cout << "The IK map of end pose/vel fails, in function "
              << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return false;
  }
  return setBoundaryCond(start_jp, start_jv, end_jp, end_jv);
}

bool PTPTrajectory::setBoundaryCond(const Pose &start_pos, const Vec &sp_dot,
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

bool PTPTrajectory::Trajectory(const double time, Eigen::VectorXd *jp,
                               Eigen::VectorXd *jv, Eigen::VectorXd *ja) const {
  std::ostringstream ss;
  if (!planDone_) {
    std::cout << "planning not done yet, can not call this function"
              << ", in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  if (!jp || !jv || !ja) {
    std::cout << "input jp, jv, and ja is null "
              << ", in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  // jp->resize(numJnts_, 0);
  // jv->resize(numJnts_, 0);
  // ja->resize(numJnts_, 0);
  // if duration is too small, just return the start pos, vel, acc
  if (duration_ < MIN_TRAJ_DURATION) {
    *jp = sq_;
    *jv = sqdot_;
    *ja = Eigen::VectorXd::Zero(numJnts_);
  }
  for (size_t i = 0; i < numJnts_; i++) {
    double X, Xdot, Xddot;
    if (jnt_prof_[i]->Trajectory(time, &X, &Xdot, &Xddot)) {
      (*jp)(i) = X;
      (*jv)(i) = Xdot;
      (*ja)(i) = Xddot;
    } else {
      ss.str("");
      ss << "call joint profile " << i << ", Trajectory function fails"
         << " at time " << time << ", duration=" << duration_
         << ", in function " << __FUNCTION__ << ", at line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
  }
  return true;
}

bool PTPTrajectory::RelTrajectory(const double time, Eigen::VectorXd *jp,
                                  Eigen::VectorXd *jv,
                                  Eigen::VectorXd *ja) const {
  std::ostringstream ss;
  if (!planDone_) {
    std::cout << "planning not done yet, can not call this function"
              << ", in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  if (!jp || !jv || !ja) {
    std::cout << "input jp, jv, and ja is null "
              << ", in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    // LOG_ERROR(ss);
    return false;
  }
  // if duration is too small, just return the start pos, vel, acc
  if (duration_ < MIN_TRAJ_DURATION) {
    *jp = sq_;
    *jv = sqdot_;
    *ja = Eigen::VectorXd::Zero(numJnts_);
  }
  for (size_t i = 0; i < numJnts_; i++) {
    double X, Xdot, Xddot;
    if (jnt_prof_[i]->Trajectory(time, &X, &Xdot, &Xddot)) {
      (*jp)(i) = X - sq_(i);
      (*jv)(i) = Xdot;
      (*ja)(i) = Xddot;
    } else {
      ss.str("");
      ss << "call joint profile " << i << ", Trajectory function fails"
         << " at time " << time << ", duration=" << duration_
         << ", in function " << __FUNCTION__ << ", at line " << __LINE__
         << std::endl;
      LOG_ERROR(ss);
      return false;
    }
  }
  return true;
}

}  // namespace kinematics_lib
