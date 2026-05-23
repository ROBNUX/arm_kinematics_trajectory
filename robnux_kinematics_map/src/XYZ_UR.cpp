#include "robnux_kinematics_map/XYZ_UR.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XYZ_UR, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {


XYZ_UR::XYZ_UR(): BaseKinematicMap(5, 5) {
    UR = std::make_shared<UJNT>();
    XYZ = std::make_shared<XYZGantry>();
    jnt_names_.resize(DoF_);
    jnt_names_[0] = "JOINT_1_ACT";
    jnt_names_[1] = "JOINT_2_ACT";
    jnt_names_[2] = "JOINT_3_ACT";
    jnt_names_[3] = "JOINT_4_ACT";
    jnt_names_[4] = "JOINT_5_ACT";
 }



void XYZ_UR::SetGeometry(const Eigen::VectorXd& kine_para) {
  std::ostringstream strs; 
  if (kine_para.size() < 4 * DoF_) {
      strs.str("");
      strs << GetName() << ":" << "XYZ_UR set geometry got input parameters with wrong dimension"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
  }
  if (!UR || !XYZ) {
      strs.str("");
      strs << GetName() << ":" << "XYZ_UR set geometry fails because UR and XYZ are null"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
  }
  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_);
  alpha = kine_para.segment(0, DoF_);
  a = kine_para.segment(DoF_, DoF_);
  theta = kine_para.segment(2 * DoF_, DoF_);
  d = kine_para.segment(3 * DoF_, DoF_);

  Eigen::VectorXd dh_XYZ(4 * 3), dh_UR(4 * 2);
  dh_XYZ.segment(0, 3) = alpha.head(3);
  dh_XYZ.segment(3, 3) = a.head(3);
  dh_XYZ.segment(6, 3) = theta.head(3);
  dh_XYZ.segment(9, 3) = d.head(3);
  XYZ->SetGeometry(dh_XYZ);
  dh_UR.segment(0, 2) = alpha.tail(2);
  dh_UR.segment(2, 2) = a.tail(2);
  dh_UR.segment(4, 2) = theta.tail(2);
  dh_UR.segment(6, 2) = d.tail(2);
  UR->SetGeometry(dh_UR);
  initialized_ = true;
}

int XYZ_UR::CartToJnt(const Pose& p, Eigen::VectorXd& q) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }
  std::vector<int> ikJointTurns;
  p.getJointTurns(&ikJointTurns);

  std::vector<int> ikJointTurn_UR;
  ikJointTurn_UR.insert(ikJointTurn_UR.end(), ikJointTurns.begin()+ XYZ->GetDoF(), ikJointTurns.end());
  Pose p1 = p;
  p1.setJointTurns(ikJointTurn_UR);
  Eigen::VectorXd UR_jnt(2);
  int ret = UR->CartToJnt(p1, UR_jnt); // using p->getRotation() to compute UR_jnt
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "UR->CartToJnt fails in "
        << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }

  Pose p2;
  Rotation r; // default rotation
  p2.setTranslation(p.getTranslation());
  p2.setRotation(r);
  
  Eigen::VectorXd XYZ_jnt(3);
  ret = XYZ->CartToJnt(p2, XYZ_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "XYZ->CartToJnt fails in "
        << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  if (q.size() < DoF_) {
    q.resize(DoF_);
  }
  
  q.segment(0, 3) = XYZ_jnt;
  q.segment(3, 2) = UR_jnt;
  return 0;
}

 
int XYZ_UR::JntToCart(const Eigen::VectorXd& q, Pose& p) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }
  if (q.size() != DoF_) {
    strs.str("");
    strs << GetName() << ":" << "Input q dimension does not match with the robot"
          << ", q size=" << q.size() << ", DoF_ =" << DoF_
              << " in function " 
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM; 
  }
  Pose pXYZ;
  Eigen::VectorXd q1(3);
  q1 << q(0), q(1), q(2);
  int ret = XYZ->JntToCart(q1, pXYZ);
  if (ret < 0) {
    return ret;
  }

  Pose pUR;
  Eigen::VectorXd q2(2);
  q2 << q(3), q(4);
  ret = UR->JntToCart(q2, pUR);
  if (ret < 0) {
    return ret;
  }


  // turns and flags
  std::vector<int>  jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);

  // combine  
  Frame tmp(pUR.getRotation(), pXYZ.getTranslation());
  p.setFrame(tmp);
  p.setBranchFlags(ikBranchFlags);
  p.setJointTurns(jntTurns);
  return 0;
}
        

int XYZ_UR::JntToCart(const Eigen::VectorXd& q,
                      const Eigen::VectorXd& qdot,
                      Pose& p, Twist& v) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }
  if (q.size() != DoF_ || qdot.size() != DoF_) {
    strs.str("");
    strs << GetName() << ":" << "Input q and qdot dimension does not match with the robot"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_PARA_WRONG_DIM; 
  }
  Eigen::VectorXd q1(3), q1dot(3), q2(2), q2dot(2);
  q1 << q(0), q(1), q(2);
  q1dot << qdot(0), qdot(1), qdot(2);
  q2 << q(3), q(4);
  q2dot << qdot(3), qdot(4);
  Pose pXYZ, pUR;
  Twist vXYZ, vUR;
  int ret = XYZ->JntToCart(q1, q1dot, pXYZ, vXYZ);
  if (ret < 0) {
    return ret;
  }
  ret = UR->JntToCart(q2, q2dot, pUR, vUR);
  if (ret < 0) {
    return ret;
  }
  Frame fr(pUR.getRotation(), pXYZ.getTranslation());
  p.setFrame(fr);

  // turns and flags
  std::vector<int>  jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);

  p.setBranchFlags(ikBranchFlags);
  p.setJointTurns(jntTurns);

  
  Vec linear_vel = vXYZ.getLinearVel();
  Vec angular_vel = vUR.getAngularVel();
  v.setLinearVel(linear_vel); 
  v.setAngularVel(angular_vel);
  return 0;
}

  
int XYZ_UR::CartToJnt(const Pose& p, const Twist& v,
                      Eigen::VectorXd& q,
                      Eigen::VectorXd& qdot) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() <<  " geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }
  Eigen::VectorXd qXYZ(XYZ->GetDoF()), qUR(UR->GetDoF()), qXYZdot(XYZ->GetDoF()), qURdot(UR->GetDoF());
  int  ret = XYZ->CartToJnt(p, v, qXYZ, qXYZdot);
  if (ret < 0) {
    return ret;
  }
  ret = UR->CartToJnt(p, v, qUR, qURdot);
  if (ret < 0) {
    return ret;
  }
  if (q.size() < DoF_) {
    q.resize(DoF_);
  }
  // over_jnt=[XYZ_jnt, UR_jnt]
  q.segment(0, XYZ->GetDoF()) = qXYZ;
  q.segment(XYZ->GetDoF(), UR->GetDoF()) = qUR;

  if (qdot.size() < DoF_) {
    qdot.resize(DoF_);
  }
  qdot.segment(0, XYZ->GetDoF()) = qXYZdot;
  qdot.segment(XYZ->GetDoF(), UR->GetDoF()) = qURdot;
  return 0;
}
 
int XYZ_UR::JntToCart(const Eigen::VectorXd& /*q*/,
                      const Eigen::VectorXd& /*qdot*/,
                      const Eigen::VectorXd& /*qddot*/, Pose& /*p*/,
                      Twist& /*v*/, Twist& /*a*/) {
  // Acceleration-level FK not implemented for XYZ_UR.
  return -1;
}

int XYZ_UR::CartToJnt(const Pose& /*p*/, const Twist& /*v*/,
                      const Twist& /*a*/, Eigen::VectorXd& /*q*/,
                      Eigen::VectorXd& /*qdot*/, Eigen::VectorXd& /*qddot*/) {
  // Acceleration-level IK not implemented for XYZ_UR.
  return -1;
}

int XYZ_UR::CalcJacobian(const Eigen::VectorXd& kine_para,
                          Pose& p,
                          Eigen::MatrixXd& Jp_t,  // for XYZ
                          Eigen::MatrixXd& Jp_r,  // for UR
                          bool world_jac) {  
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << GetName() <<  " geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }
  if (kine_para.size() != 4 * DoF_) {
      strs.str("");
      strs << GetName() << ":" << "input kine_parameters has dimension not equal to 5 * DoF in "
                  << __FUNCTION__ << " line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;
  }

  Eigen::VectorXd alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_);
  alpha = kine_para.segment(0, DoF_);
  a = kine_para.segment(DoF_, DoF_);
  theta = kine_para.segment(2 * DoF_, DoF_);
  d = kine_para.segment(3 * DoF_, DoF_);
  
  Eigen::VectorXd dh_XYZ(4 * 3), dh_UR(4 * 2); 
  dh_XYZ.segment(0, 3) = alpha.head(3);
  dh_XYZ.segment(3, 3) = a.head(3);
  dh_XYZ.segment(6, 3) = theta.head(3);
  dh_XYZ.segment(9, 3) = d.head(3);
  dh_UR.segment(0, 2) = alpha.tail(2);
  dh_UR.segment(2, 2) = a.tail(2);
  dh_UR.segment(4, 2) = theta.tail(2);
  dh_UR.segment(6, 2) = d.tail(2);

  Pose pXYZ, pUR;
  Eigen::MatrixXd Jp_t_XYZ, Jp_r_XYZ;
  Eigen::MatrixXd Jp_t_UR, Jp_r_UR;
  int ret = XYZ->CalcJacobian(dh_XYZ, pXYZ, Jp_t_XYZ, Jp_r_XYZ, world_jac);
  if (ret < 0) {
    return ret;
  }

  ret = UR->CalcJacobian(dh_UR, pUR, Jp_t_UR, Jp_r_UR, world_jac);
  if (ret < 0) {
    return ret;
  }
  Frame fr(pUR.getRotation(), pXYZ.getTranslation());
  p.setFrame(fr);
  
  // turns and flags
  std::vector<int>  jntTurns1, jntTurns2, jntTurns;
  std::vector<int> ikBranchFlags;
  // jnt turns vector will be combo of XYZ jnt turns + UR jnt turns
  pXYZ.getJointTurns(&jntTurns1);
  pUR.getJointTurns(&jntTurns2);
  jntTurns.insert(jntTurns.end(), jntTurns1.begin(), jntTurns1.end());
  jntTurns.insert(jntTurns.end(), jntTurns2.begin(), jntTurns2.end());
  // BranchFlags will be only for UR robot
  pUR.getBranchFlags(&ikBranchFlags);
  p.setBranchFlags(ikBranchFlags);
  p.setJointTurns(jntTurns);
  Jp_t = Jp_t_XYZ;
  Jp_r = Jp_r_UR;
  return 0;
}


void XYZ_UR::SetUseCalibrated(const bool useCalibrated) {
  if (!initialized_) {
      std::ostringstream strs;
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    UR->SetUseCalibrated(useCalibrated);
    XYZ->SetUseCalibrated(useCalibrated);
    useCalibrated_ = useCalibrated;
}

  void XYZ_UR::SetDefaultBaseOff(const Eigen::VectorXd& baseoff) {
    if (!initialized_) {
      std::ostringstream strs;
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    if (baseoff.size() != 14) {
      std::ostringstream strs;
      strs.str("");
      strs << GetName() << ":" << "input baseoff has dimension not equal to DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    UR->SetDefaultBaseOff(baseoff.segment(7, 7));
    XYZ->SetDefaultBaseOff(baseoff.segment(0, 7));
  }

  void XYZ_UR::GetDefaultBaseOff(Eigen::VectorXd& baseoff) const {
    if (!initialized_) {
      std::ostringstream strs;
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    if (baseoff.size() < 14) {
      baseoff.resize(14);
    }
    Eigen::VectorXd baseoff_XYZ, baseoff_UR;
    baseoff_XYZ.resize(7);
    baseoff_UR.resize(7);
    XYZ->GetDefaultBaseOff(baseoff_XYZ);
    UR->GetDefaultBaseOff(baseoff_UR);
    baseoff.segment(0, 7) = baseoff_XYZ;
    baseoff.segment(7, 7) = baseoff_UR;
  }

  void XYZ_UR::GetPitchAndBacklash(Eigen::VectorXd& pitch,
                           Eigen::VectorXd& backlash) const {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    pitch.resize(DoF_);
    backlash.resize(DoF_);
    Eigen::VectorXd pitch_XYZ, backlash_XYZ;
    Eigen::VectorXd pitch_UR, backlash_UR;
    XYZ->GetPitchAndBacklash(pitch_XYZ, backlash_XYZ);
    UR->GetPitchAndBacklash(pitch_UR, backlash_UR);
    pitch.segment(0, XYZ->GetDoF()) = pitch_XYZ;
    backlash.segment(0, XYZ->GetDoF()) = backlash_XYZ;
    pitch.segment(XYZ->GetDoF(), UR->GetDoF()) = pitch_UR;
    backlash.segment(XYZ->GetDoF(), UR->GetDoF()) = backlash_UR;
  }

  void XYZ_UR::SetPitchAndBacklash(const Eigen::VectorXd& pitch,
                           const Eigen::VectorXd& backlash) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    if (pitch.size() != DoF_ || backlash.size() != DoF_) {
      strs.str("");
      strs << GetName() << ":" << "input pitch and backlash have dimension not equal to DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    Eigen::VectorXd pitch_XYZ, backlash_XYZ;
    Eigen::VectorXd pitch_UR, backlash_UR;
    pitch_XYZ = pitch.segment(0, XYZ->GetDoF());
    backlash_XYZ = backlash.segment(0, XYZ->GetDoF());
    pitch_UR = pitch.segment(XYZ->GetDoF(), UR->GetDoF());
    backlash_UR = backlash.segment(XYZ->GetDoF(), UR->GetDoF());
    XYZ->SetPitchAndBacklash(pitch_XYZ, backlash_XYZ);
    UR->SetPitchAndBacklash(pitch_UR, backlash_UR);
  }

  int XYZ_UR::CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                  Eigen::VectorXd& qpassive) {
    return 0;  // means no passive joints
  }

  
  void XYZ_UR::ResetCalibration() {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    isDHCalibrated_ = false;
    XYZ->ResetCalibration();
    UR->ResetCalibration();
  }


}