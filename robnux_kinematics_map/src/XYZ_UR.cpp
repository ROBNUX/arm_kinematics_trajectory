#include "robnux_kinematics_map/XYZ_UR.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XYZ_UR, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {


XYZ_UR::XYZ_UR(): BaseKinematicMap(5, 5) {
    UR = std::make_shared<UJNT>();
    XYZ = std::make_shared<XYZGantry>();
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
  
  for (size_t i = 0; i < DoF_; i++) {
    if (i < XYZ->GetDoF()) {
        q(i) = XYZ_jnt(i);
    } else {
        q(i) = UR_jnt(i - XYZ->GetDoF());
    }
  }
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
  int ret = XYZ->JntToCart(q1, q1dot, &pXYZ, &vXYZ);
  if (ret < 0) {
    return ret;
  }
  ret = UR->JntToCart(q2, q2dot, &pUR, &vUR);
  if (ret < 0) {
    return ret;
  }
  Frame fr(pUR.getRotation(), pXYZ.getTranslation());
  p->setFrame(fr);

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
  q.segment(XYZ->GetDoF(), UR->GetDoF()) = qUR

  if (qdot.size() < DoF_) {
    qdot.resize(DoF_);
  }
  qdot.segment(0, XYZ->GetDoF()) = qXYZdot;
  qdot.segment(XYZ->GetDoF(), UR->GetDoF()) = qURdot;
  return 0;
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
    int ret = XYZ->CalcJacobian(dh_XYZ, pXYZ, Jp_t_XYZ, Jp_r_XYZ, reduction);
    if (ret < 0) {
      return ret;
    }
  
    ret = UR->CalcJacobian(dh_UR, pUR, Jp_t_UR, Jp_r_UR, reduction);
    if (ret < 0) {
      return ret;
    }
    Frame fr(pUR.getRotation(), pXYZ.getTranslation());
    p->setFrame(fr);
    
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


  


 

  


  


 
  
  
 

  
  void XYZ_UR::SetPitchCoef(const Eigen::VectorXd &pitch) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    if (pitch.size() != DoF_) {
      strs.str("");
      strs << GetName() << ":" << "input kine_parameters has dimension not equal to 5 * DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    pitch_ = pitch;
    strs.str("");
    strs << GetName() << ":" << "SetPitch = " << pitch_ << std::endl;
    LOG_INFO(strs);
    Eigen::VectorXd xyzSeg(XYZ->GetDoF());
    Eigen::VectorXd urSeg(UR->GetDoF());
    for (size_t i=0; i < DoF_; i++) {
      if (i < XYZ->GetDoF()) {
        xyzSeg(i) = pitch(i);
      } else {
        urSeg(i - XYZ->GetDoF()) = pitch(i);
      }
    }

    XYZ->SetPitchCoef(xyzSeg);
    UR->SetPitchCoef(urSeg);    
  }

  


  bool XYZ_UR::GetCalibParamSet(EigenDRef<Eigen::VectorXd> *cal_DH) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false; 
    }
    size_t dof_XYZ = XYZ->GetDoF();
    size_t dof_UR = UR->GetDoF();
    Eigen::VectorXd cal_DH_XYZ(5 * dof_XYZ + 14);
    Eigen::VectorXd cal_DH_UR(5 * dof_UR + 14);
    if (!XYZ->GetCalibParamSet(&cal_DH_XYZ)) {
      return false;
    }
    if (!UR->GetCalibParamSet(&cal_DH_UR)) {
      return false;
    }
    std::vector<double> alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);
    
    for (size_t i=0; i < dof_XYZ; i++) {
      alpha[i] = cal_DH_XYZ[i];
      a[i] = cal_DH_XYZ[dof_XYZ + i];
      theta[i] = cal_DH_XYZ[2 * dof_XYZ + i];
      d[i] = cal_DH_XYZ[3 * dof_XYZ + i];
      beta[i] = cal_DH_XYZ[4 * dof_XYZ + i];
    } 

    for (size_t i=0; i < dof_UR; i++) {
      alpha[dof_XYZ + i] = cal_DH_UR[i];
      a[dof_XYZ + i] = cal_DH_UR[dof_UR + i];
      theta[dof_XYZ + i] = cal_DH_UR[2 * dof_UR + i];
      d[dof_XYZ + i] = cal_DH_UR[3 * dof_UR + i];
      beta[dof_XYZ + i] = cal_DH_UR[4 * dof_UR + i];
    } 
   
    if (cal_DH->size() < 5 * DoF_ + 14) {
      strs.str("");
      strs << GetName() << ":" << "The input vector pointer has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
    }
    std::vector<double> tmp;
    tmp.insert(tmp.end(), alpha.begin(), alpha.end());
    tmp.insert(tmp.end(), a.begin(), a.end());
    tmp.insert(tmp.end(), theta.begin(), theta.end());
    tmp.insert(tmp.end(), d.begin(), d.end());
    tmp.insert(tmp.end(), beta.begin(), beta.end());
    for (size_t i=0; i < 5 * DoF_; i++) {
       (*cal_DH)(i) = tmp[i];
    }
    // get defaultBaseoff and subdefaultBaseoff
    cal_DH->segment(5 * DoF_, 7) = cal_DH_XYZ.segment(5 * dof_XYZ, 7);
    cal_DH->segment(5 * DoF_ + 7, 7) = cal_DH_UR.segment(5 * dof_UR, 7);
    strs.str("");
    strs << GetName() << ": GetCalibPram success" << std::endl;
    LOG_INFO(strs);
    return true;
  }

  bool XYZ_UR::GetCalibParamSet(Eigen::VectorXd *cal_DH) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false; 
    }
    size_t dof_XYZ = XYZ->GetDoF();
    size_t dof_UR = UR->GetDoF();
    Eigen::VectorXd cal_DH_XYZ(5 * dof_XYZ + 14);
    Eigen::VectorXd cal_DH_UR(5 * dof_UR + 14);
    if (!XYZ->GetCalibParamSet(&cal_DH_XYZ)) {
      return false;
    }
    if (!UR->GetCalibParamSet(&cal_DH_UR)) {
      return false;
    }
    std::vector<double> alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);
    
    for (size_t i=0; i < dof_XYZ; i++) {
      alpha[i] = cal_DH_XYZ[i];
      a[i] = cal_DH_XYZ[dof_XYZ + i];
      theta[i] = cal_DH_XYZ[2 * dof_XYZ + i];
      d[i] = cal_DH_XYZ[3 * dof_XYZ + i];
      beta[i] = cal_DH_XYZ[4 * dof_XYZ + i];
    } 

    for (size_t i=0; i < dof_UR; i++) {
      alpha[dof_XYZ + i] = cal_DH_UR[i];
      a[dof_XYZ + i] = cal_DH_UR[dof_UR + i];
      theta[dof_XYZ + i] = cal_DH_UR[2 * dof_UR + i];
      d[dof_XYZ + i] = cal_DH_UR[3 * dof_UR + i];
      beta[dof_XYZ + i] = cal_DH_UR[4 * dof_UR + i];
    } 
   
    if (cal_DH->size() < 5 * DoF_ + 14) {
      strs.str("");
      strs << GetName() << ":" << "The input vector pointer has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
    }
    std::vector<double> tmp;
    tmp.insert(tmp.end(), alpha.begin(), alpha.end());
    tmp.insert(tmp.end(), a.begin(), a.end());
    tmp.insert(tmp.end(), theta.begin(), theta.end());
    tmp.insert(tmp.end(), d.begin(), d.end());
    tmp.insert(tmp.end(), beta.begin(), beta.end());
    for (size_t i=0; i < 5 * DoF_; i++) {
       (*cal_DH)(i) = tmp[i];
    }
    // get defaultBaseoff and subdefaultBaseoff
    cal_DH->segment(5 * DoF_, 7) = cal_DH_XYZ.segment(5 * dof_XYZ, 7);
    cal_DH->segment(5 * DoF_ + 7, 7) = cal_DH_UR.segment(5 * dof_UR, 7);
    strs.str("");
    strs << GetName() << ": GetCalibPram success" << std::endl;
    LOG_INFO(strs);
    return true;
  }

  bool XYZ_UR::LoadCalibParamSet(const std::vector<double> &kine_para) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false; 
    }
    std::vector<double> alpha(DoF_), a(DoF_), theta(DoF_), d(DoF_), beta(DoF_);
    for (size_t i=0; i < DoF_; i++) {
          alpha[i] = kine_para[i];
          a[i] = kine_para[DoF_ + i];
          theta[i] = kine_para[2 * DoF_ + i];
          d[i] = kine_para[3 * DoF_ + i];
          beta[i] = kine_para[4 * DoF_ + i];
    }
    std::vector<double> dh_XYZ, dh_UR;  
    dh_XYZ.insert(dh_XYZ.end(), alpha.begin(), alpha.begin() + 3);
    dh_XYZ.insert(dh_XYZ.end(), a.begin(), a.begin() + 3);
    dh_XYZ.insert(dh_XYZ.end(), theta.begin(), theta.begin() + 3);
    dh_XYZ.insert(dh_XYZ.end(), d.begin(), d.begin() + 3);
    dh_XYZ.insert(dh_XYZ.end(), beta.begin(), beta.begin() + 3);
    dh_XYZ.insert(dh_XYZ.end(), kine_para.begin()+ 5 * DoF_, kine_para.begin() + 5 * DoF_ + 7);
    dh_XYZ.insert(dh_XYZ.end(), kine_para.begin()+ 5 * DoF_, kine_para.begin() + 5 * DoF_ + 7);
    dh_UR.insert(dh_UR.end(), alpha.begin() + 3, alpha.end());
    dh_UR.insert(dh_UR.end(), a.begin() + 3, a.end());
    dh_UR.insert(dh_UR.end(), theta.begin() + 3, theta.end());
    dh_UR.insert(dh_UR.end(), d.begin() + 3, d.end());
    dh_UR.insert(dh_UR.end(), beta.begin() + 3, beta.end());
    dh_UR.insert(dh_UR.end(), kine_para.begin()+ 5 * DoF_+7, kine_para.end());
    dh_UR.insert(dh_UR.end(), kine_para.begin()+ 5 * DoF_+7, kine_para.end());
    if (XYZ->LoadCalibParamSet(dh_XYZ) && UR->LoadCalibParamSet(dh_UR)) {
      this->isDHCalibrated_ = true;
    }
    return true;
  }

  

  void XYZ_UR::SetUsingCalibratedModel(bool useCalibratedModel) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    XYZ->SetUsingCalibratedModel(useCalibratedModel);
    UR->SetUsingCalibratedModel(useCalibratedModel);
    useCalibrated_ = useCalibratedModel;
  }

  //! has robot been calibrated
  bool XYZ_UR::isCalibrated() {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false; 
    }
    // std::cout << "XYZ clibrated: "  << XYZ->isCalibrated() << ", UR calibrated: " << UR->isCalibrated()  << std::endl;
    // std::cout << GetName() << ": isCalibrated() =" << isDHCalibrated_ << std::endl;
    return isDHCalibrated_;
  }

  //! is parameter initialized
  bool XYZ_UR::isInitialized() const {
    if (!initialized_) {
      return false;
    }
    return XYZ->isInitialized() && UR->isInitialized();
  }
  //! reset calibration model
  bool XYZ_UR::resetCalibration() {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false; 
    }
    isDHCalibrated_ = false;
    return XYZ->resetCalibration() && UR->resetCalibration();
  }


}