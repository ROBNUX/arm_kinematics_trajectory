#include "trajectory/move_command.hpp"
namespace kinematics_lib {

PTPMotionCommand::PTPMotionCommand(
    const Pose& start_pose, const Pose& goal_pose,
    const std::vector<JntProfile>& jnt_pf,
    const boost::shared_ptr<BaseKinematicMap>& armKM, const int percent)
    : jnt_pf_(jnt_pf),
      armKM_(armKM),
      MotionCommand(ID_PTP, start_pose, goal_pose, ProfileData(), percent) {
  std::ostringstream strs;
  if (!armKM) {
    return;  // if kinematics pointer is null, return directly
  }
  int errCode1 = armKM->CartToJnt(start_, &start_jnt_);
  int errCode2 = armKM->CartToJnt(goal_, &goal_jnt_);
  if (errCode1 < 0 || errCode2 < 0) {
    strs.str("");
    strs << "Inverse kinematics error with error code 1: " << errCode1
         << ", error code 2: " << errCode2 << ", in function " << __FUNCTION__
         << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return;
  }

  DoF_ = armKM_->GetDoF();
  A_DoF_ = armKM_->GetActDoF();
  diff_jnt_.resize(A_DoF_);

  // now compute the joint-space distance of this PTP motion
  diff_jnt_ = goal_jnt_ - start_jnt_;
  totalJntLength_ = diff_jnt_.norm();
  this->totalArcLength_ =
      totalJntLength_;  // totalJntLength_ * armKM-> GetCharLength();
  if (totalArcLength_ > MIN_JNT_DIST) {
    initialized_ = true;
  }
}

}  // namespace kinematics_lib
