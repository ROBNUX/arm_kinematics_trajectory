#include "XYZ_UR.hpp"
#include <common/pose.hpp>
#include <common/utility.hpp>
namespace kinematics_lib {

 //! default constructor
  XYZ_UR::XYZ_UR(): BaseKinematicMap(5, 5) {
     UR = std::make_shared<UJNT>();
     XYZ = std::make_shared<XYZGantry>();
  }



  void XYZ_UR::SetGeometry(const std::vector<double> &kine_para) {
    std::ostringstream strs; 
    if (kine_para.size() < 5 * DoF_) {
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
    XYZ->SetGeometry(dh_XYZ);
    dh_UR.insert(dh_UR.end(), alpha.begin() + 3, alpha.end());
    dh_UR.insert(dh_UR.end(), a.begin() + 3, a.end());
    dh_UR.insert(dh_UR.end(), theta.begin() + 3, theta.end());
    dh_UR.insert(dh_UR.end(), d.begin() + 3, d.end());
    dh_UR.insert(dh_UR.end(), beta.begin() + 3, beta.end());
    UR->SetGeometry(dh_UR);
    initialized_ = true;
  }

  // here p=(r, t), with r=UJNT orientation w.r.t. world frame, t= t_XYZ  is translation of XYZ w.r.t. world frame
  int XYZ_UR::CartToJnt(const Pose &p, std::vector<double> *q) {
    std::ostringstream strs;
    if (!q) {
      strs.str("");
      strs << GetName() << " input joint angle pointer is null in " << __FUNCTION__
                 << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
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
    std::vector<double> UR_jnt;
    int ret = UR->CartToJnt(p1, &UR_jnt); // using p->getRotation() to compute UR_jnt
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
    
    std::vector<double> XYZ_jnt;
    ret = XYZ->CartToJnt(p2, &XYZ_jnt);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":" << "XYZ->CartToJnt fails in "
          << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;
    }
    if (q->size() < DoF_) {
      q->resize(DoF_);
    }
    // over_jnt=[XYZ_jnt, UR_jnt]
    for (size_t i = 0; i < DoF_; i++) {
      if (i < XYZ->GetDoF()) {
         q->at(i) = XYZ_jnt[i];
      } else {
         q->at(i) = UR_jnt[i - XYZ->GetDoF()];
      }
    }
    return 0;
  }

 
  int XYZ_UR::JntToCart(const std::vector<double> & q,
                       Pose *p) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!p) {
      strs.str("");
      strs << GetName() << " input pose pointer is null in " << __FUNCTION__
                 << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
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
    std::vector<double> q1;
    q1.insert(q1.end(), q.begin(), q.begin() + 3);
    int ret = XYZ->JntToCart(q1, &pXYZ);
    if (ret < 0) {
      return ret;
    }

    Pose pUR;
    std::vector<double> q2;
    q2.insert(q2.end(), q.begin()+3, q.end());
    ret = UR->JntToCart(q2, &pUR);
    if (ret < 0) {
      return ret;
    }

    // Eigen::VectorXd q2eg;
    // StdVec2EigenVec(q2, &q2eg);
    // strs.str("");
    // strs << GetName() << ":" << "jnt angle vector=" << q2eg << std::endl;
    // strs << ", cart=" << pUR.ToString(true) << "rot mat " << pUR.getRotation().ToEigenMat() << std::endl;
    // LOG_INFO(strs);
  

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
    p->setFrame(tmp);
    p->setBranchFlags(ikBranchFlags);
    p->setJointTurns(jntTurns);
    return 0;
  }
        

  int XYZ_UR::JntToCart(const std::vector<double> &q,
                        const std::vector<double> &qdot,
                        Pose *p, Twist *v) {
    std::ostringstream strs;
    if (!p || !v) {
      strs.str("");
      strs << GetName() << ":" << "input pose and twist parameter is null in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
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
    std::vector<double> q1, q1dot, q2, q2dot;
    q1.insert(q1.end(), q.begin(), q.begin() + 3);
    q1dot.insert(q1dot.end(), qdot.begin(), qdot.begin() + 3);
    q2.insert(q2.end(), q.begin() + 3, q.end());
    q2dot.insert(q2dot.end(), qdot.begin() + 3, qdot.end());

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

    p->setBranchFlags(ikBranchFlags);
    p->setJointTurns(jntTurns);

   
    Vec linear_vel = vXYZ.getLinearVel();
    Vec angular_vel = vUR.getAngularVel();
    v->setLinearVel(linear_vel); 
    v->setAngularVel(angular_vel);
    return 0;
  }

  void XYZ_UR::SetOrientNormal(const EigenDRef<Eigen::Vector3d>  &normal) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    UR->SetOrientNormal(normal);
    XYZ->SetOrientNormal(normal);
  }
  int XYZ_UR::CartToJnt(const Pose &p, const Twist &v,
                         std::vector<double> *q,
                         std::vector<double> *qdot) {
    std::ostringstream strs;
    if (!q || !qdot) {
       strs.str("");
       strs << GetName() << ":" << "Input q and qdot vectors are null"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return -ERR_INPUT_POINTER_NULL;  
    }
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    std::vector<double> qXYZ, qUR, qXYZdot, qURdot;
    int  ret = XYZ->CartToJnt(p, v, &qXYZ, &qXYZdot);
    if (ret < 0) {
      return ret;
    }
    ret = UR->CartToJnt(p, v, &qUR, &qURdot);
    if (ret < 0) {
      return ret;
    }
    if (q->size() < DoF_) {
      q->resize(DoF_);
    }
    // over_jnt=[XYZ_jnt, UR_jnt]
    for (size_t i = 0; i < DoF_; i++) {
      if (i < XYZ->GetDoF()) {
         q->at(i) = qXYZ[i];
      } else {
         q->at(i) = qUR[i - XYZ->GetDoF()];
      }
    }
    if (qdot->size() < DoF_) {
      qdot->resize(DoF_);
    }
    // over_jnt=[XYZ_jnt, UR_jnt]
    for (size_t i = 0; i < DoF_; i++) {
      if (i < XYZ->GetDoF()) {
         qdot->at(i) = qXYZdot[i];
      } else {
         qdot->at(i) = qURdot[i - XYZ->GetDoF()];
      }
    }
    return 0;
  }
 
  int XYZ_UR::CalcJacobian(const std::vector<double> &kine_para,
                            Pose *p,
                            Eigen::MatrixXd * Jp_t,  // for XYZ
                            Eigen::MatrixXd * Jp_r,
                            const bool reduction) {  // for UR
    std::ostringstream strs;
    // Jp is 6 * kine_para_.size() matrix
    if (!p || !Jp_t || !Jp_r) {
        strs.str("");
        strs << GetName() << ":" << "input pose, Jp, Jj pointers are null in " << __FUNCTION__
                << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
    }
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (kine_para.size() != 5 * DoF_) {
        strs.str("");
        strs << GetName() << ":" << "input kine_parameters has dimension not equal to 5 * DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_PARA_WRONG_DIM;
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

    Pose pXYZ, pUR;
    Eigen::MatrixXd Jp_t_XYZ, Jp_r_XYZ;
    Eigen::MatrixXd Jp_t_UR, Jp_r_UR;
    int ret = XYZ->CalcJacobian(dh_XYZ, &pXYZ, &Jp_t_XYZ, &Jp_r_XYZ, reduction);
    if (ret < 0) {
      return ret;
    }
  
    dh_UR.insert(dh_UR.end(), alpha.begin() + 3, alpha.end());
    dh_UR.insert(dh_UR.end(), a.begin() + 3, a.end());
    dh_UR.insert(dh_UR.end(), theta.begin() + 3, theta.end());
    dh_UR.insert(dh_UR.end(), d.begin() + 3, d.end());
    dh_UR.insert(dh_UR.end(), beta.begin() + 3, beta.end());
    ret = UR->CalcJacobian(dh_UR, &pUR, &Jp_t_UR, &Jp_r_UR, reduction);
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
    p->setBranchFlags(ikBranchFlags);
    p->setJointTurns(jntTurns);
    *Jp_t = Jp_t_XYZ;
    *Jp_r = Jp_r_UR;
    return 0;
  }


  void XYZ_UR::ConvertBranchFlag(const size_t inFlag,
                               std::vector<int> *branchFlags) const {
    std::ostringstream strs;                            
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    XYZ->ConvertBranchFlag(inFlag, branchFlags);
  }

// the following duplicated serialArm  multiTurnFlags
void XYZ_UR::ConvertMultiTurnFlag(const size_t inTurn,
                                   std::vector<int> *ikJointTurns)  const {
    std::ostringstream strs; 
    if (!ikJointTurns) {
        strs.str("");
        strs << GetName() << ":" << "ConvertMultiTurnFlag fails due to null input flag pointer"
                << " in " << __FUNCTION__
                << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return;
    }

    size_t turn = inTurn;
    ikJointTurns->resize(DoF_, 0);
    int i=0;
    while (turn >= 1 && i < DoF_) {
        ikJointTurns->at(i) = turn % 16;
        if (ikJointTurns->at(i) >= 8) {  // turn in [-8, 7]
          ikJointTurns->at(i) -= 16;
        }
        turn =std::floor(turn / 16);
        i++;
    }
    
    strs.str("");
    strs << GetName() << ":" << "turn value is " << inTurn << ", convert to turn vector " << std::endl;
    for (size_t j=0; j< DoF_; j++) {
      strs << "turn " << j << " = " << ikJointTurns->at(j) << std::endl;
    }
    LOG_INFO(strs);
}


  double XYZ_UR::CalibrateLaserCoplanar(
       const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
       const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
       const EigenDRef<Eigen::MatrixXd> &laserMat_x,
       const EigenDRef<Eigen::MatrixXd> &laserMat_y,
       const EigenDRef<Eigen::MatrixXd> &laserMat_z,
       const EigenDRef<Eigen::Vector3d> &laser_scale) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    
    Eigen::MatrixXd cart_m_x = cart_measure_x;
    Eigen::MatrixXd cart_m_y = cart_measure_y;
    Eigen::MatrixXd cart_m_z = cart_measure_z;
    size_t numRow = cart_m_x.rows();
    size_t numCol = cart_m_x.cols();
    if (numRow > 3) {
      cart_m_x.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
    }
    numRow = cart_m_y.rows();
    numCol = cart_m_y.cols();
    if (numRow > 3) {
      cart_m_y.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
    }
    numRow = cart_m_z.rows();
    numCol = cart_m_z.cols();
    if (numRow > 3) {
      cart_m_z.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
    }

    double ret = XYZ->CalibrateLaserCoplanar(cart_m_x, cart_m_y, cart_m_z, laserMat_x, laserMat_y, laserMat_z, laser_scale);
    if (ret >=0 ) {   // if XYZ calibration sucess
      if (UR->isCalibrated()) {
        isDHCalibrated_ = true;  // set XYZ-UR calibration success
      }
    }
    return ret;
  }

  double XYZ_UR::LaserCalibrateCoplanar(
       const Eigen::MatrixXd &cart_measure_x,  // cartesian coordinates reported from robot
       const Eigen::MatrixXd &cart_measure_y,
       const Eigen::MatrixXd &cart_measure_z,
       const Eigen::MatrixXd &laserMat_x,
       const Eigen::MatrixXd &laserMat_y,
       const Eigen::MatrixXd &laserMat_z,
       const Eigen::Vector3d &laser_scale) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      Eigen::MatrixXd cart_m_x = cart_measure_x;
      Eigen::MatrixXd cart_m_y = cart_measure_y;
      Eigen::MatrixXd cart_m_z = cart_measure_z;
      size_t numRow = cart_m_x.rows();
      size_t numCol = cart_m_x.cols();
      if (numRow > 3) {
        cart_m_x.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
      }
      numRow = cart_m_y.rows();
      numCol = cart_m_y.cols();
      if (numRow > 3) {
        cart_m_y.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
      }
      numRow = cart_m_z.rows();
      numCol = cart_m_z.cols();
      if (numRow > 3) {
        cart_m_z.block(3, 0, numRow - 3, numCol) = Eigen::MatrixXd::Zero(numRow - 3, numCol);  // for XYZ robot, needs to set Euler angles and turns, branch as 0
      }

      double ret = XYZ->LaserCalibrateCoplanar(cart_m_x, cart_m_y, cart_m_z, laserMat_x, laserMat_y, laserMat_z, laser_scale);
      if (ret >=0 ) {   // if XYZ calibration sucess
        if (UR->isCalibrated()) {
          isDHCalibrated_ = true;  // set XYZ-UR calibration success
        }
      }
      return ret;
    } catch (...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }

  int XYZ_UR::ErrManualPathUserBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,  //(x,y, z, u, r)
                                const Eigen::VectorXd &orig_tool,  // used tcp (default 0)
                                const int numXPts, // No. points in X plane
                                const int numYPts, // No. points in Y plane
                                const int numZPts, // No. points in Z plane
                                EigenDRef<Eigen::MatrixXd> *workobj_uncal,    // each col is a workobj frame (datum ref.)  appended with a UR jnt, so 8 * numLocs
                                EigenDRef<Eigen::MatrixXd> *workobj_cal    // each col is a workobj frame (data ref.)   appended with a UR jnt, so 8 * numLocs
                               ) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t numJnts = jnt_base_measures.cols();
      size_t numPtsOneLoc = numXPts + numYPts + numZPts;
      // here orig_base is the workpiece frame, should be reachable by IK
      if (jnt_base_measures.rows() < DoF_ ||  numJnts < numPtsOneLoc || numJnts % numPtsOneLoc != 0) {
        strs.str("");
        strs << GetName() << ":" << "The input vector has wrong dimension in function, input jnt dim= "
            << jnt_base_measures.rows() << ", numJnts=" << numJnts  << ", numPtsOneLoc=" << numPtsOneLoc
              << ", in " << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }
      if (!workobj_uncal || !workobj_cal) {
        strs.str("");
        strs << GetName() << ":"  << "input subTCP pointer is null in function " << __FUNCTION__ 
            << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }

      size_t numLocs = numJnts / numPtsOneLoc;
      strs.str("");
      strs << GetName() << ": numJnts=" << numJnts << ", numPtsOneLoc=" << numPtsOneLoc << ", numLocs=" << numLocs << std::endl;
      LOG_INFO(strs);

      // initialize workobj at all UR angles (each col is a workobj at one set of UR jnt, and all columns represent all workobjs)
      workobj_uncal->resize(9, numLocs);
      workobj_cal->resize(9, numLocs);


      Frame userBase; // default base
      Vec t(orig_tool.segment(0, 3)); //here tool only contains the trans. 
      Quaternion q(orig_tool(3), orig_tool(4), orig_tool(5), orig_tool(6)); // rotation of tool
      Frame userTool;
      userTool.setTranslation(t);  // user Tool
      userTool.setQuaternion(q);
    

      // temporary variables
      Pose ps;
      refPose rps;
      
      // We have to compute the uncalibrated coord, because version only provides x,y coordinates
      size_t xyz_dof = XYZ->GetDoF();
      size_t ur_dof = UR->GetDoF();
      XYZ->SetUsingCalibratedModel(true);
      for (size_t i=0; i<numLocs; i++) {
        Eigen::MatrixXd jnt_data = jnt_base_measures.block(0, i * numPtsOneLoc, DoF_, numPtsOneLoc);
        // get xyz_jnt data at one loc
        Eigen::MatrixXd  xyz_jnt_data = jnt_data.block(0,0, xyz_dof, numPtsOneLoc);
        Eigen::MatrixXd  xyz_data(xyz_dof, numPtsOneLoc);
        // get ur_jnt data at one loc
        Eigen::MatrixXd ur_data = jnt_data.block(xyz_dof, 0, ur_dof, numPtsOneLoc);
        // get mean ur_vec at this loc
        Eigen::VectorXd ur_vec = ur_data.rowwise().mean();

        // now compute cartesian data using calibrated model of XYZ arm
        for (size_t j=0; j< numPtsOneLoc; j++) {
          std::vector<double> jnt_xyz(xyz_dof, 0);
          Eigen::VectorXd jnt2 = xyz_jnt_data.col(j);
          EigenVec2StdVec(jnt2, &jnt_xyz);


          int ret = XYZ->JntToCart(jnt_xyz, &ps);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << ":" << "XYZ FK error, code  " << ret
                    << "can not do error compensation in"
                    << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret;  
          }
          refPose default_rps;
          default_rps.setDefaultPose(ps);
          // get refPose under default base and userTool, and calibrated model
          default_rps.getPoseUnderNewRef(userBase, userTool, &rps);
          strs.str("");
          strs << GetName() << ":" << " loc " << i <<  ", point " << j << ", rot = " << rps.getRotation().ToString() <<
              ", translation=" << rps.getTranslation().ToString() << std::endl;
          
          xyz_data.col(j) = rps.getTranslation().ToEigenVec();
          LOG_INFO(strs);
        }
        // now get X/Y/Z plane equation
        // now using xdata 
        Eigen::MatrixXd  xdata = xyz_data.block(0, 0, xyz_dof, numXPts);
        Eigen::Vector3d meanPtx = xdata.rowwise().mean();
        Eigen::MatrixXd tmp = xdata.colwise() - meanPtx;

        strs.str("");
        strs << GetName() << ": loc " << i << std::endl;
        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXd tmpV = svd.matrixV();
        Eigen::Vector3d pnx = tmpV.col(2); 
         // X plane equation will be pnx^T X = dx, now compute dx
        double dx = (pnx.transpose() * xdata).mean();

        strs <<"meanPtx=" << meanPtx.transpose() << ", pnx=" << pnx.transpose() << ",  dx=" << dx << std::endl;

        // now using ydata 
        Eigen::MatrixXd  ydata = xyz_data.block(0, numXPts, xyz_dof, numYPts);
        Eigen::VectorXd meanPty = ydata.rowwise().mean();
        tmp = ydata.colwise() - meanPty;
    
        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd1(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd1.matrixV();
        Eigen::Vector3d pny = tmpV.col(2); 
         // X plane equation will be pnx^T X = dx, now compute dx
        double dy = (pny.transpose() * ydata).mean();

        strs <<"meanPty=" << meanPty.transpose() << ", pny=" << pny.transpose() << ",  dy=" << dy << std::endl;

         // now using zdata 
        Eigen::MatrixXd  zdata = xyz_data.block(0, numXPts + numYPts, xyz_dof, numZPts);
        Eigen::VectorXd meanPtz = zdata.rowwise().mean();
        tmp = zdata.colwise() - meanPtz;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd2.matrixV();
        Eigen::Vector3d pnz = tmpV.col(2); 
         // X plane equation will be pnx^T X = dx, now compute dx
        double dz = (pnz.transpose() * zdata).mean();

        strs <<"meanPtz=" << meanPtz.transpose() << ", pnz=" << pnz.transpose() << ",  dz=" << dz << std::endl;
        
        // check the direction of normal, make sure they are outer-normal
        if (pnx.transpose() * meanPty >= dx) {
          pnx *= -1.0;
          dx *=  -1.0;
        }

        if (pny.transpose() * meanPtx >= dy) {
          pny *= -1.0;
          dy *= - 1.0; 
        }

        if (pnz.transpose() * meanPtx >= dz) {
          pnz *= - 1.0;
          dz *= -1.0;
        }

       
         
        Eigen::MatrixXd N(3, 3);
        N.row(0) = pnx.transpose();
        N.row(1) = pny.transpose();
        N.row(2) = pnz.transpose();

        Eigen::VectorXd b(3);
        b(0) = dx; b(1) = dy;  b(2) = dz;
        double det = N.determinant();

        if (fabs(det) <  CALIB_SINGULAR_CONST) {
          strs.str("");
          strs << GetName() << "determinant of N=" << det  << " is singular , in function" << __FUNCTION__
              << " at line "  << __LINE__ << std::endl; 
          LOG_ERROR(strs);
          return -2002;
        }

        Eigen::VectorXd  orig_wkobj = N.inverse() * b;
        strs << "N=" << N << ", b=" << b <<", orig_wkobj=" << orig_wkobj << std::endl;
        
        Vec vz(pnz), vx(pnx), vy(pny);
        // obtain the actual vx and vy using measured points
        vy = (vy - vy.dot(vz) * vz).NormalizeVec();
        vx = vy * vz;
        strs << ", final vz=" << vz.ToString() << ", vy=" << vy.ToString() << ", vx=" << vx.ToString() << std::endl;
        LOG_INFO(strs);
        // {vx, vy, vz forms a workobj coordinate frame
        Rotation newr;
        newr.UnitX(vx);
        newr.UnitY(vy);
        newr.UnitZ(vz);
    
        Quaternion q1;
        newr.GetQuaternion(&q1);

        Eigen::VectorXd  wkobj(9);
        wkobj.segment(0, 3) = orig_wkobj;
        wkobj(3) = q1.w();
        wkobj(4) = q1.x();
        wkobj(5) = q1.y();
        wkobj(6) = q1.z();
        wkobj.segment(7,2) = ur_vec;

        strs.str("");
        strs << "wkobj " << i << "=" << wkobj << std::endl;
        LOG_INFO(strs);
        workobj_uncal->col(i) = wkobj;
        workobj_cal->col(i) = wkobj; 
        strs.str("");
        //strs << "debug0" << std::endl;
        LOG_INFO(strs);
      }
      return 0;
    } catch(...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }

  int XYZ_UR::ManualPathToRelPath(const EigenDRef<Eigen::MatrixXd> &manPath,
                           const EigenDRef<Eigen::MatrixXd> &sub_tcp_cal,   // calibrated sub tcp
                           EigenDRef<Eigen::MatrixXd> *relPath) {
    std::ostringstream strs;
    if (!initialized_ || !relPath) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t numJnts = manPath.cols();
      size_t numWkObj = sub_tcp_cal.cols();
      // here orig_base is the workpiece frame, should be reachable by IK
      if (manPath.rows() < DoF_ || numJnts < 1 || numWkObj < 1) {
        strs.str("");
        strs << GetName() << ":" << "The input vector has wrong dimension in function, input jnt dim= "
            << manPath.rows() << ", numJnts=" << numJnts 
              << ", in " << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }
    
      
      Frame wkobj, subTCP;
      int wkobj_index = 0;
      Eigen::VectorXd wkobj_base, wkobj_UR;
      bool multiobj = numWkObj > 1 ? true : false;
      strs.str("");
      strs << GetName() << " input numJnts =" << numJnts << ", numWkOBj=" << numWkObj << ", multiobj=" << multiobj << std::endl;
      LOG_INFO(strs);
      if (!multiobj) { // if only one workobj, we need to use subTCP and UR frame to compute wkobj
        Eigen::VectorXd sub_tcp_cal_col = sub_tcp_cal.col(0);
        Vec t(sub_tcp_cal_col(0), sub_tcp_cal_col(1), sub_tcp_cal_col(2));
        Quaternion q(sub_tcp_cal_col(3), sub_tcp_cal_col(4), sub_tcp_cal_col(5), sub_tcp_cal_col(6));
        subTCP.setTranslation(t);
        subTCP.setQuaternion(q);
      } else { // multiLocWkObj
        wkobj_base = sub_tcp_cal.col(wkobj_index).segment(0, 7);
        wkobj_UR = sub_tcp_cal.col(wkobj_index).segment(7, 2); 
      }

      // define variables for vision frame calibration
      //std::vector<Eigen::Vector3d> origP(numJnts);
      relPath->resize(5, numJnts);
      //std::vector<Eigen::VectorXd>  jnt_ur(numJnts);  // two ur joints

      // compute the measured 8 pts using xyz kinematics
      Pose ps_xyz;
      Pose ps_ur;
      
      // We have to compute the uncalibrated coord, because version only provides x,y coordinates
      size_t xyz_dof = XYZ->GetDoF();
      size_t ur_dof = UR->GetDoF();
      XYZ->SetUsingCalibratedModel(true);
      UR->SetUsingCalibratedModel(true);
      for (size_t i=0; i<numJnts; i++) {
        std::vector<double> jnt_xyz(xyz_dof, 0);
        std::vector<double> jnt_ur(ur_dof, 0);
        Eigen::VectorXd jnt1 = manPath.col(i);
        Eigen::VectorXd jnt_xyz_eig = jnt1.segment(0, xyz_dof);
        EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
        Eigen::VectorXd jnt_ur_eig = jnt1.segment(xyz_dof, ur_dof);
        EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
        

        int ret = XYZ->JntToCart(jnt_xyz, &ps_xyz);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "XYZ FK error, code  " << ret
                  << "can not do error compensation in, input jnt=" << jnt_xyz_eig 
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        Vec xyz_vec = ps_xyz.getTranslation();

        if (!multiobj) { // if only one workobj, we need to use subTCP and UR frame to compute wkobj
          ret = UR->JntToCart(jnt_ur, &ps_ur);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
                  << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret; 
          }
          Frame tmp1;
          ps_ur.getFrame(&tmp1);
          wkobj =  tmp1 * subTCP;
        } else {
          Eigen::VectorXd diff_UR = jnt_ur_eig - wkobj_UR;
          if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
            wkobj_index++;
            if (wkobj_index >= numWkObj) {
                strs.str("");
                strs << GetName() << ":" << "wkobj_index=" << wkobj_index << ">= numWkObjs=" << numWkObj << std::endl;
                LOG_ERROR(strs);
                return -ERR_INPUT_PARA_WRONG_DIM;
            }

            wkobj_base = sub_tcp_cal.col(wkobj_index).segment(0, 7);
            wkobj_UR = sub_tcp_cal.col(wkobj_index).segment(7, 2);
            diff_UR = jnt_ur_eig - wkobj_UR;
            if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
                strs.str("");
                strs << GetName() << " wkobj data" << wkobj_UR << "not match with desire traj UR data " << jnt_ur_eig << std::endl;
                LOG_ERROR(strs);
                return -ERR_INPUT_PARA_WRONG_DIM;
            }
          }
          Vec t(wkobj_base(0), wkobj_base(1), wkobj_base(2));
          Quaternion q(wkobj_base(3), wkobj_base(4), wkobj_base(5), wkobj_base(6));
          wkobj.setTranslation(t);
          wkobj.setQuaternion(q);
        }

        Vec subtcp_vec =wkobj.getTranslation();
        Rotation subtcp_rot = wkobj.getRotation();

        Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
        Eigen::VectorXd relPathPt(5);
        relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
        relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
        relPath->col(i) = relPathPt;
        strs.str("");
        strs << ", xyz_vec[" << i << "]=" << xyz_vec.ToString()
            << ", subtcp_vec[" << i << "]=" << subtcp_vec.ToString()
            << ", subtcp_rot=" << subtcp_rot.ToEigenMat()
            << ", relPathPt=" << relPathPt.transpose() << std::endl;
        LOG_INFO(strs);
      }
      return 0;
    } catch (...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }

  int XYZ_UR::ManualPathToRelPath(const Eigen::MatrixXd &manPath,
                           const Eigen::MatrixXd &sub_tcp_cal,   // calibrated sub tcp
                           Eigen::MatrixXd *relPath) {
    std::ostringstream strs;
    if (!initialized_ || !relPath) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t numJnts = manPath.cols();
      size_t numWkObj = sub_tcp_cal.cols();
      // here orig_base is the workpiece frame, should be reachable by IK
      if (manPath.rows() < DoF_ || numJnts < 1 || numWkObj < 1) {
        strs.str("");
        strs << GetName() << ":" << "The input vector has wrong dimension in function, input jnt dim= "
            << manPath.rows() << ", numJnts=" << numJnts 
              << ", in " << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }
      
      Frame wkobj, subTCP;
      int wkobj_index = 0;
      Eigen::VectorXd wkobj_base, wkobj_UR;

      bool multiobj = numWkObj > 1 ? true : false;
      if (!multiobj) { // if only one workobj, we need to use subTCP and UR frame to compute wkobj
        Eigen::VectorXd sub_tcp_cal_col = sub_tcp_cal.col(0);
        Vec t(sub_tcp_cal_col(0), sub_tcp_cal_col(1), sub_tcp_cal_col(2));
        Quaternion q(sub_tcp_cal_col(3), sub_tcp_cal_col(4), sub_tcp_cal_col(5), sub_tcp_cal_col(6));
        subTCP.setTranslation(t);
        subTCP.setQuaternion(q);
      } else { // multiLocWkObj
        wkobj_base = sub_tcp_cal.col(wkobj_index).segment(0, 7);
        wkobj_UR = sub_tcp_cal.col(wkobj_index).segment(7, 2); 
      }
      // define variables for vision frame calibration
      //std::vector<Eigen::Vector3d> origP(numJnts);
      relPath->resize(5, numJnts);
      //std::vector<Eigen::VectorXd>  jnt_ur(numJnts);  // two ur joints

      // compute the absolute coordinates of path points using xyz and ur calibrated kinematics
      Pose ps_xyz;
      Pose ps_ur;
      
      // We have to compute the uncalibrated coord, because version only provides x,y coordinates
      size_t xyz_dof = XYZ->GetDoF();
      size_t ur_dof = UR->GetDoF();
      XYZ->SetUsingCalibratedModel(true);
      UR->SetUsingCalibratedModel(true);
      for (size_t i=0; i<numJnts; i++) {
        std::vector<double> jnt_xyz(xyz_dof, 0);
        std::vector<double> jnt_ur(ur_dof, 0);
        Eigen::VectorXd jnt1 = manPath.col(i);
        Eigen::VectorXd jnt_xyz_eig = jnt1.segment(0, xyz_dof);
        EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
        Eigen::VectorXd jnt_ur_eig = jnt1.segment(xyz_dof, ur_dof);
        EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
        

        int ret = XYZ->JntToCart(jnt_xyz, &ps_xyz);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "XYZ FK error, code  " << ret
                  << "can not do error compensation in, input jnt=" << jnt_xyz_eig 
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        Vec xyz_vec = ps_xyz.getTranslation();
        if (!multiobj) {
          ret = UR->JntToCart(jnt_ur, &ps_ur);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
                  << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret; 
          }
          Frame tmp1;
          ps_ur.getFrame(&tmp1);
          wkobj =  tmp1 * subTCP;
        } else {
          Eigen::VectorXd diff_UR = jnt_ur_eig - wkobj_UR;
          if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
            wkobj_index++;
            if (wkobj_index >= numWkObj) {
                strs.str("");
                strs << GetName() << ":" << "wkobj_index=" << wkobj_index << ">= numWkObjs=" << numWkObj << std::endl;
                LOG_ERROR(strs);
                return -ERR_INPUT_PARA_WRONG_DIM;
            }

            wkobj_base = sub_tcp_cal.col(wkobj_index).segment(0, 7);
            wkobj_UR = sub_tcp_cal.col(wkobj_index).segment(7, 2);
            diff_UR = jnt_ur_eig - wkobj_UR;
            if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
                strs.str("");
                strs << GetName() << " wkobj data" << wkobj_UR << "not match with desire traj UR data " << jnt_ur_eig << std::endl;
                LOG_ERROR(strs);
                return -ERR_INPUT_PARA_WRONG_DIM;
            }
          }
          Vec t(wkobj_base(0), wkobj_base(1), wkobj_base(2));
          Quaternion q(wkobj_base(3), wkobj_base(4), wkobj_base(5), wkobj_base(6));
          wkobj.setTranslation(t);
          wkobj.setQuaternion(q);
        }

        Vec subtcp_vec = wkobj.getTranslation();
        Rotation subtcp_rot = wkobj.getRotation();

        Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
        Eigen::VectorXd relPathPt(5);
        relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
        relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
        relPath->col(i) = relPathPt;
        strs.str("");
        strs << ", xyz_vec[" << i << "]=" << xyz_vec.ToString()
            << ", subtcp_vec[" << i << "]=" << subtcp_vec.ToString()
            << ", subtcp_rot=" << subtcp_rot.ToEigenMat()
            << ", relPathPt=" << relPathPt.transpose() << std::endl;
        LOG_INFO(strs);
      }
      return 0;
    } catch (...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }


  int XYZ_UR::ErrVisionUserBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,  //(x,y, *, u, r)
                                const Eigen::VectorXd &orig_tool,  // tcp of vision (maybe)
                                EigenDRef<Eigen::VectorXd> *sub_tcp_uncal,    // uncalibrated tcp of UR workobj w.r.t UR
                                EigenDRef<Eigen::VectorXd> *sub_tcp_cal    // calibrated tcp of UR workobj w.r.t. UR
                               ) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t numJnts = jnt_base_measures.cols();
      // here orig_base is the workpiece frame, should be reachable by IK
      if (jnt_base_measures.rows() < DoF_ || numJnts < 8) {
        strs.str("");
        strs << GetName() << ":" << "The input vector has wrong dimension in function, input jnt dim= "
            << jnt_base_measures.rows() << ", numJnts=" << numJnts 
              << ", in " << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }
      if (!sub_tcp_uncal || !sub_tcp_cal) {
        strs.str("");
        strs << GetName() << ":"  << "input subTCP pointer is null in function " << __FUNCTION__ 
            << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }

      // initialize relative orientation as 0 w.r.t. workpiece frame
      (*sub_tcp_uncal).setZero();
      (*sub_tcp_cal).setZero();


      // define variables for vision frame calibration
      std::vector<Eigen::Vector3d> origP(numJnts);
      std::vector<Eigen::Vector2d> po(numJnts);
      Eigen::MatrixXd A, AT, B;
      Eigen::VectorXd b;
      size_t num_ind_jnts = numJnts / 5;
      std::vector<Eigen::VectorXd>  jnt_ur(num_ind_jnts);  // two ur joints

      // need to do centering around UR center
      Eigen::Vector3d origin= sub_defaultBaseOff_.getTranslation().ToEigenVec();
      strs.str("");
      strs << GetName() << ": use ur base origin=" << origin << std::endl;
      LOG_INFO(strs);


      Frame userBase; // default base
      Vec t(orig_tool.segment(0, 3)); //here tool only contains the trans. 
      Quaternion q(orig_tool(3), orig_tool(4), orig_tool(5), orig_tool(6)); // rotation of tool
      Frame userTool;
      userTool.setTranslation(t);  // user Tool
      userTool.setQuaternion(q);
      // compute the measured 8 pts using xyz kinematics
      std::vector<Pose> ps(numJnts);
      std::vector<refPose> rps(numJnts);
      
      // We have to compute the uncalibrated coord, because version only provides x,y coordinates
      size_t xyz_dof = XYZ->GetDoF();
      size_t ur_dof = UR->GetDoF();
      XYZ->SetUsingCalibratedModel(true);
      for (size_t i=0; i<numJnts; i++) {
        std::vector<double> jnt(xyz_dof, 0);
        Eigen::VectorXd jnt1 = jnt_base_measures.col(i);
        Eigen::VectorXd jnt2 = jnt1.segment(0, xyz_dof);
        EigenVec2StdVec(jnt2, &jnt);
        if (i % 5==0) { // pick up first and 5th joints
          size_t ind = i / 5;
          if (ind < num_ind_jnts) {
            jnt_ur[ind] = jnt1.segment(xyz_dof, ur_dof);
            strs.str("");
            strs << "ur_jnt " << ind << " =" << jnt_ur[ind] << std::endl;
            LOG_INFO(strs);
          }
        }


        int ret = XYZ->JntToCart(jnt, &ps[i]);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "XYZ FK error, code  " << ret
                  << "can not do error compensation in"
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        refPose default_rps;
        default_rps.setDefaultPose(ps[i]);
        // get refPose under default base and userTool, and calibrated model
        default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
        strs.str("");
        strs << GetName() << ":" << "Canonical model: vision point " << i << ", rot = " << rps[i].getRotation().ToString() <<
            ", translation=" << rps[i].getTranslation().ToString() << std::endl;
        
        origP[i] = rps[i].getTranslation().ToEigenVec();
        strs << ", origP[" << i << "]=" << origP[i].transpose() << std::endl;
        LOG_INFO(strs);
      }
      
      // now assign position to  po(8)
      strs.str("");
      strs << GetName() << ":";
      for (size_t i=0; i< numJnts; i++) {
         po[i] = origP[i].segment(0, 2); 
         strs << "po[" << i << "]=" << po[i].transpose() << std::endl;
      }
      LOG_INFO(strs);

      // note: that calibrated po[i] makes no sense because we don't know z, so we have
      // to assume (x,y) from uncalibrated coordinates
      // now compute the  subTCP transformation
      B.resize(4 * num_ind_jnts, 2);
      A.resize(4 * num_ind_jnts, 3);
      AT.resize(2 * num_ind_jnts, 3);
      b.resize(2 * num_ind_jnts);

      strs.str("");
      strs << GetName() << ":";
      std::vector<Eigen::Vector2d> cent(num_ind_jnts);
      for (size_t j=0; j < num_ind_jnts; j++) {
          cent[j] = Eigen::Vector2d::Zero();
          for (size_t k=1; k<5; k++) {
            cent[j] += po[5*j + k];
          }
          cent[j] /= 4.0;
        //for (size_t i=1; i < 3; i++) {
          Eigen::Vector2d aa = (po[5*j + 2] + po[5*j + 3]) / 2.0;
          Eigen::Vector2d bb = (po[5*j + 1] + po[5*j + 4]) / 2.0;
          Eigen::Vector2d cc =  (po[5*j + 1] + po[5*j + 2]) / 2.0;
          Eigen::Vector2d dd = (po[5*j + 3] + po[5*j + 4]) / 2.0;

          Eigen::Vector2d diff1 =  aa - cent[j]; //po[5*j];   //po[5*j + 2] - po[5 * j + 1];    // x1+ - x1-
          Eigen::Vector2d diff2 =  cc - cent[j]; //po[5*j];  // po[5*j + 2] - po[5 * j + 3];  // y1+ - y1-
          strs << "first xy, ind=" << j << ", diff1=" << diff1.transpose() << ", diff2=" << diff2.transpose() 
          << std::endl;
          B.block(4 * j, 0, 2, 1) = diff1;
          B.block(4 * j, 1, 2, 1) = diff2;
          Eigen::Vector2d diff1_2 =  // po[5 * j] 
                                    cent[j] - bb; // po[5*j + 3] - po[5 * j + 4];    // x2+ - x2-
          Eigen::Vector2d diff2_2 =  // po[5 * j] 
                                    cent[j] - dd; // po[5*j + 1] - po[5 * j + 4];  // y2+ - y2-
          strs << "second xy, ind=" << j << ", diff1=" << diff1_2.transpose() << ", diff2=" << diff2_2.transpose() 
          << std::endl;
          B.block(4 * j + 2, 0, 2, 1) = diff1_2; // (diff1 + diff1_2) / 2.0;
          B.block(4 * j + 2, 1, 2, 1) = diff2_2; // (diff2 + diff2_2) / 2.0;
        //}
      }
      strs << ", final B=" << B << std::endl;
      LOG_INFO(strs);

      // now compute Ri, ti at two ur joint
      std::vector<Eigen::Matrix3d>  RR(num_ind_jnts);
      std::vector<Eigen::Vector3d> tt(num_ind_jnts);
      UR->SetUsingCalibratedModel(true);

      // UR using calibrated model (could bring some error here, lets hope XYZ are close to be perfect)
      strs.str("");
      strs << GetName() << ":";
      for (size_t i=0; i < num_ind_jnts; i++) {
        Pose ps1;
        std::vector<double> jnt(ur_dof, 0);
        EigenVec2StdVec(jnt_ur[i], &jnt);
        int ret = UR->JntToCart(jnt, &ps1);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur[i]
                << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret; 
        }
        RR[i] = ps1.getRotation().ToEigenMat();
        tt[i] = ps1.getTranslation().ToEigenVec();
        strs << " ind=" << i << ", ps=" << ", ur_jnt=" << jnt_ur[i] << ", ps1=" << ps1.ToString(true) << std::endl;
        strs << ", RRi=" << RR[i] << ", tti=" << tt[i].transpose() << std::endl;
      }
      LOG_INFO(strs);
      

      //strs.str("");
      //strs << " RR1T * (p1-p0) = " << RR[0].transpose() * (origP[1] - origP[0])  << ", RR2T * (P1'-p0')=" << RR[1].transpose() * (origP[5] - origP[4]) << std::endl;
      //strs << " RR1T * (p2-p0) = " << RR[0].transpose() * (origP[2] - origP[0])  << ", RR2T * (P2'-p0')=" << RR[1].transpose() * (origP[6] - origP[4]) << std::endl;
      //LOG_INFO(strs); 

      // CoefA matrix
      Eigen::MatrixXd  tmpId = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd  A1 = tmpId.block(0, 0, 2, 3);  // first 2 rows of identity matrix

      for (size_t i=0; i<num_ind_jnts; i++) {
        A.block(4 * i, 0, 2, 3) = A1 * RR[i];
        A.block(4 * i + 2, 0, 2, 3) = A1 * RR[i];
        AT.block(2 * i, 0, 2, 3) = A1 * RR[i];
        b.segment(2 * i, 2) = cent[i] //po[5*i] 
                             - tt[i].segment(0, 2);  // po[0] is origin, po[4] is the new origin
      }

      Eigen::MatrixXd  ATA = A.transpose() * A;
      double det = ATA.determinant();
      if (det < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << "determinant of ATA=" << det  << " is singular , in function" << __FUNCTION__
              << " at line "  << __LINE__ << std::endl; 
        LOG_ERROR(strs);
        return -2002;
      }
      Eigen::MatrixXd  ATA2 = AT.transpose() * AT;
      double det2 = ATA2.determinant();
      if (det2 < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << "determinant of ATA2=" << det2  << " is singular , in function" << __FUNCTION__
              << " at line "  << __LINE__ << std::endl; 
        LOG_ERROR(strs);
        return -2002;
      }
      
      strs.str("");
      strs << GetName() << ": A=" <<  A  << ", det(ATA)=" << det <<", B=" << B << std::endl;
      strs << GetName() << ": AT=" << AT << ", det(ATA2)=" << det2 << ", b=" << b << std::endl;
      LOG_INFO(strs);
      Eigen::MatrixXd relR = (ATA).inverse() * A.transpose() * B;
      Eigen::VectorXd relT = (ATA2).inverse() * AT.transpose() * b;
      strs.str("");
      strs << GetName() << ", relR=" << relR << ", relT=" << relT.transpose() << std::endl;
      Eigen::MatrixXd  relR_err = A * relR - B;
      Eigen::VectorXd  relT_err = AT * relT - b;
      Eigen::Vector2d  origin_offset =  Eigen::Vector2d::Zero();
      for (size_t i=0; i < num_ind_jnts; i++) {
          origin_offset -= relT_err.segment(2 * i, 2);
      }
      origin_offset /= num_ind_jnts;

      strs << "relR_err =" << relR_err << ", relT_err=" << relT_err << std::endl;
      strs << "origin_offset=" << origin_offset << ", using new offset, reT_err=" << relT_err +origin_offset << std::endl;
      LOG_INFO(strs);
      
      Eigen::MatrixXd  relR2;
      relR2.resize(3,3);
      
      Eigen::Vector3d v1 = relR.col(0);
      v1 /= v1.norm();
      Eigen::Vector3d v2 = relR.col(1);
      v2 /= v2.norm();
      Eigen::Vector3d v3 = v1.cross(v2);
      v3 /= v3.norm();
      relR2.col(0) = v1;
      relR2.col(1) = v2;
      relR2.col(2) = v3; 

      // projection of relR into the space of rotational matrix
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(relR2, Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpU = svd.matrixU();
      Eigen::MatrixXd tmpV = svd.matrixV();
      Eigen::MatrixXd finalR = tmpU * tmpV.transpose();
      strs.str("");
      strs << "sol1: using 2 axis: relR2=" << relR2 << "svd, tmpU=" << tmpU << ", tmpV=" << tmpV <<", finalR=" << finalR << std::endl;
      LOG_INFO(strs);
      /*
      Eigen::Matrix3d relR1;
      v3 = relR.col(2);
      v3 /= v3.norm();
      relR1.col(0) = v1;
      relR1.col(1) = v2;
      relR1.col(2) = v3;
      
      Eigen::JacobiSVD<Eigen::MatrixXd> svd1(relR1, Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpU = svd1.matrixU();
      tmpV = svd1.matrixV();
      Eigen::Matrix3d finalR1 = tmpU * tmpV.transpose();
      if (finalR1.determinant() < 0) {
          finalR1.col(2) = - finalR1.col(2);
      }
      strs.str("");
      strs << "sol2: using 3 axis: realR1=" <<  relR1 << ", svd, tmpU=" << tmpU << ", tmpV=" << tmpV << ", finalR1=" << finalR1 <<  std::endl;
      LOG_INFO(strs);
      */
      Vec xv(finalR.col(0));
      Vec yv(finalR.col(1));
      Vec zv(finalR.col(2));
      Rotation relRR(xv, yv, zv);
      Quaternion q1;
      relRR.GetQuaternion(&q1);

      sub_tcp_uncal->segment(0, 3) = relT;
      (*sub_tcp_uncal)(3) = q1.w();
      (*sub_tcp_uncal)(4) = q1.x();
      (*sub_tcp_uncal)(5) = q1.y();
      (*sub_tcp_uncal)(6) = q1.z();
      
      *sub_tcp_cal = *sub_tcp_uncal;  // set sub_tcp_cal and sub_tcp_uncal as same

      // now use subTCP doing some test
      strs.str("");
      for (size_t i=0; i<num_ind_jnts; i++) {
        Eigen::Vector3d  pred_o = tt[i] + RR[i] * relT;
        //Eigen::Vector3d  pred_op = tt[1] + RR[1] * relT;
        strs << " meausred o=" << origP[5 * i] << ", predicted o=" << pred_o  << std::endl;
              // ", measured op=" << origP[4] << ", predicted op=" << pred_op << std::endl;
        
        Eigen::Vector3d  pred_ox = RR[i] * relR.col(0);
        Eigen::Vector3d  pred_oy = RR[i] * relR.col(1);
        //Eigen::Vector3d  pred_oz = RR[i] * relR.col(2);

        strs << "measured ox=" << (origP[5*i + 2] - origP[5 *i + 1]) / 2.0 
             << ",measured ox 2=" <<  (origP[5*i + 3] - origP[5 *i + 4]) / 2.0 << ", predicted ox=" << pred_ox << std::endl;
        strs << "measured oy=" << (origP[5 *i + 2] - origP[5 * i + 3]) / 2.0
             << "measured oy2=" << (origP[5 *i + 1] - origP[5 * i + 4]) / 2.0  << ", predicted oy=" << pred_oy << std::endl;
        //strs << "measured oz=" << origP[4 *i + 3] - origP[4 * i]  << ", predicted oz=" << pred_oz << std::endl;
      }
      LOG_INFO(strs);
      return 0;
    } catch (...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }
  
  int XYZ_UR::ErrVisionUserBase(const Eigen::MatrixXd &jnt_base_measures,  //(x,y, *, u, r)
                                const Eigen::VectorXd &orig_tool,  // tcp of vision (maybe)
                                Eigen::VectorXd *sub_tcp_uncal,    // uncalibrated tcp of UR workobj w.r.t UR
                                Eigen::VectorXd *sub_tcp_cal    // calibrated tcp of UR workobj w.r.t. UR
                               ) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t numJnts = jnt_base_measures.cols();
      // here orig_base is the workpiece frame, should be reachable by IK
      if (jnt_base_measures.rows() < DoF_ || numJnts < 8) {
        strs.str("");
        strs << GetName() << ":" << "The input vector has wrong dimension in function, input jnt dim= "
            << jnt_base_measures.rows() << ", numJnts=" << numJnts 
              << ", in " << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }
      if (!sub_tcp_uncal || !sub_tcp_cal) {
        strs.str("");
        strs << GetName() << ":"  << "input subTCP pointer is null in function " << __FUNCTION__ 
            << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
      }

      // initialize relative orientation as 0 w.r.t. workpiece frame
      (*sub_tcp_uncal).setZero();
      (*sub_tcp_cal).setZero();


      // define variables for vision frame calibration
      std::vector<Eigen::Vector3d> origP(numJnts);
      std::vector<Eigen::Vector2d> po(numJnts);
      Eigen::MatrixXd A, AT, B;
      Eigen::VectorXd b;
      size_t num_ind_jnts = numJnts / 5;
      std::vector<Eigen::VectorXd>  jnt_ur(num_ind_jnts);  // two ur joints



      Frame userBase; // default base
      Vec t(orig_tool.segment(0, 3)); //here tool only contains the trans. 
      Quaternion q(orig_tool(3), orig_tool(4), orig_tool(5), orig_tool(6)); // rotation of tool
      Frame userTool;
      userTool.setTranslation(t);  // user Tool
      userTool.setQuaternion(q);
      // compute the measured 8 pts using xyz kinematics
      std::vector<Pose> ps(numJnts);
      std::vector<refPose> rps(numJnts);
      
      // We have to compute the uncalibrated coord, because version only provides x,y coordinates
      size_t xyz_dof = XYZ->GetDoF();
      size_t ur_dof = UR->GetDoF();
      XYZ->SetUsingCalibratedModel(false);
      for (size_t i=0; i<numJnts; i++) {
        std::vector<double> jnt(xyz_dof, 0);
        Eigen::VectorXd jnt1 = jnt_base_measures.col(i);
        Eigen::VectorXd jnt2 = jnt1.segment(0, xyz_dof);
        EigenVec2StdVec(jnt2, &jnt);
        if (i % 5==0) { // pick up first and 5th joints
          size_t ind = i / 5;
          if (ind < num_ind_jnts) {
            jnt_ur[ind] = jnt1.segment(xyz_dof, ur_dof);
            strs.str("");
            strs << "ur_jnt " << ind << " =" << jnt_ur[ind] << std::endl;
            LOG_INFO(strs);
          }
        }


        int ret = XYZ->JntToCart(jnt, &ps[i]);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "XYZ FK error, code  " << ret
                  << "can not do error compensation in"
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        refPose default_rps;
        default_rps.setDefaultPose(ps[i]);
        // get refPose under default base and userTool, and calibrated model
        default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
        strs.str("");
        strs << GetName() << ":" << "Canonical model: vision point " << i << ", rot = " << rps[i].getRotation().ToString() <<
            ", translation=" << rps[i].getTranslation().ToString() << std::endl;
        
        origP[i] = rps[i].getTranslation().ToEigenVec();
        strs << ", origP[" << i << "]=" << origP[i].transpose() << std::endl;
        LOG_INFO(strs);
      }
      
      // now assign position to  po(8)
      strs.str("");
      strs << GetName() << ":";
      for (size_t i=0; i< numJnts; i++) {
         po[i] = origP[i].segment(0, 2); 
         strs << "po[" << i << "]=" << po[i].transpose() << std::endl;
      }
      LOG_INFO(strs);

      // note: that calibrated po[i] makes no sense because we don't know z, so we have
      // to assume (x,y) from uncalibrated coordinates
      // now compute the  subTCP transformation
      B.resize(4 * num_ind_jnts, 2);
      A.resize(4 * num_ind_jnts, 3);
      AT.resize(2 * num_ind_jnts, 3);
      b.resize(2 * num_ind_jnts);

      strs.str("");
      strs << GetName() << ":";
      for (size_t j=0; j < num_ind_jnts; j++) {
        //for (size_t i=1; i < 3; i++) {
          Eigen::Vector2d aa = (po[5*j + 2] + po[5*j + 3]) / 2.0;
          Eigen::Vector2d bb = (po[5*j + 1] + po[5*j + 4]) / 2.0;
          Eigen::Vector2d cc =  (po[5*j + 1] + po[5*j + 2]) / 2.0;
          Eigen::Vector2d dd = (po[5*j + 3] + po[5*j + 4]) / 2.0;

          Eigen::Vector2d diff1 =  aa - po[5*j];   //po[5*j + 2] - po[5 * j + 1];    // x1+ - x1-
          Eigen::Vector2d diff2 =  cc - po[5*j];  // po[5*j + 2] - po[5 * j + 3];  // y1+ - y1-
          strs << "first xy, ind=" << j << ", diff1=" << diff1.transpose() << ", diff2=" << diff2.transpose() 
          << std::endl;
          B.block(4 * j, 0, 2, 1) = diff1;
          B.block(4 * j, 1, 2, 1) = diff2;
          Eigen::Vector2d diff1_2 =  po[5 * j] - bb; // po[5*j + 3] - po[5 * j + 4];    // x2+ - x2-
          Eigen::Vector2d diff2_2 =  po[5 * j] - dd; // po[5*j + 1] - po[5 * j + 4];  // y2+ - y2-
          strs << "second xy, ind=" << j << ", diff1=" << diff1_2.transpose() << ", diff2=" << diff2_2.transpose() 
          << std::endl;
          B.block(4 * j + 2, 0, 2, 1) = diff1_2; // (diff1 + diff1_2) / 2.0;
          B.block(4 * j + 2, 1, 2, 1) = diff2_2; // (diff2 + diff2_2) / 2.0;
        //}
      }
      strs << ", final B=" << B << std::endl;
      LOG_INFO(strs);

      // now compute Ri, ti at two ur joint
      std::vector<Eigen::Matrix3d>  RR(num_ind_jnts);
      std::vector<Eigen::Vector3d> tt(num_ind_jnts);
      UR->SetUsingCalibratedModel(true);

      // UR using calibrated model (could bring some error here, lets hope XYZ are close to be perfect)
      strs.str("");
      strs << GetName() << ":";
      for (size_t i=0; i < num_ind_jnts; i++) {
        Pose ps1;
        std::vector<double> jnt(ur_dof, 0);
        EigenVec2StdVec(jnt_ur[i], &jnt);
        int ret = UR->JntToCart(jnt, &ps1);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur[i]
                << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret; 
        }
        RR[i] = ps1.getRotation().ToEigenMat();
        tt[i] = ps1.getTranslation().ToEigenVec();
        strs << " ind=" << i << ", ps=" << ", ur_jnt=" << jnt_ur[i] << ", ps1=" << ps1.ToString(true) << std::endl;
        strs << ", RRi=" << RR[i] << ", tti=" << tt[i].transpose() << std::endl;
      }
      LOG_INFO(strs);
      

      //strs.str("");
      //strs << " RR1T * (p1-p0) = " << RR[0].transpose() * (origP[1] - origP[0])  << ", RR2T * (P1'-p0')=" << RR[1].transpose() * (origP[5] - origP[4]) << std::endl;
      //strs << " RR1T * (p2-p0) = " << RR[0].transpose() * (origP[2] - origP[0])  << ", RR2T * (P2'-p0')=" << RR[1].transpose() * (origP[6] - origP[4]) << std::endl;
      //LOG_INFO(strs); 

      // CoefA matrix
      Eigen::MatrixXd  tmpId = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd  A1 = tmpId.block(0, 0, 2, 3);  // first 2 rows of identity matrix

      for (size_t i=0; i<num_ind_jnts; i++) {
        A.block(4 * i, 0, 2, 3) = A1 * RR[i];
        A.block(4 * i + 2, 0, 2, 3) = A1 * RR[i];
        AT.block(2 * i, 0, 2, 3) = A1 * RR[i];
        b.segment(2 * i, 2) = po[5*i] - tt[i].segment(0, 2);  // po[0] is origin, po[4] is the new origin
      }

      Eigen::MatrixXd  ATA = A.transpose() * A;
      double det = ATA.determinant();
      if (det < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << "determinant of ATA=" << det  << " is singular , in function" << __FUNCTION__
              << " at line "  << __LINE__ << std::endl; 
        LOG_ERROR(strs);
        return -2002;
      }
      Eigen::MatrixXd  ATA2 = AT.transpose() * AT;
      double det2 = ATA2.determinant();
      if (det2 < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << "determinant of ATA2=" << det2  << " is singular , in function" << __FUNCTION__
              << " at line "  << __LINE__ << std::endl; 
        LOG_ERROR(strs);
        return -2002;
      }
      
      strs.str("");
      strs << GetName() << ": A=" <<  A  << ", det(ATA)=" << det <<", B=" << B << std::endl;
      strs << GetName() << ": AT=" << AT << ", det(ATA2)=" << det2 << ", b=" << b << std::endl;
      LOG_INFO(strs);
      Eigen::MatrixXd relR = (ATA).inverse() * A.transpose() * B;
      Eigen::VectorXd relT = (ATA2).inverse() * AT.transpose() * b;
      strs.str("");
      strs << GetName() << ", relR=" << relR << ", relT=" << relT.transpose() << std::endl;
      Eigen::MatrixXd  relR_err = A * relR - B;
      Eigen::VectorXd  relT_err = AT * relT - b;

      strs << "relR_err =" << relR_err << ", relT_err=" << relT_err << std::endl;
      LOG_INFO(strs);
     
      Eigen::MatrixXd  relR2;
      relR2.resize(3,3);
      
      Eigen::Vector3d v1 = relR.col(0);
      v1 /= v1.norm();
      Eigen::Vector3d v2 = relR.col(1);
      v2 /= v2.norm();
      Eigen::Vector3d v3 = v1.cross(v2);
      v3 /= v3.norm();
      relR2.col(0) = v1;
      relR2.col(1) = v2;
      relR2.col(2) = v3; 

      // projection of relR into the space of rotational matrix
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(relR2, Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpU = svd.matrixU();
      Eigen::MatrixXd tmpV = svd.matrixV();
      Eigen::MatrixXd finalR = tmpU * tmpV.transpose();
      strs.str("");
      strs << "sol1: using 2 axis: relR2=" << relR2 << "svd, tmpU=" << tmpU << ", tmpV=" << tmpV <<", finalR=" << finalR << std::endl;
      LOG_INFO(strs);
      /*
      Eigen::Matrix3d relR1;
      v3 = relR.col(2);
      v3 /= v3.norm();
      relR1.col(0) = v1;
      relR1.col(1) = v2;
      relR1.col(2) = v3;
      
      Eigen::JacobiSVD<Eigen::MatrixXd> svd1(relR1, Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpU = svd1.matrixU();
      tmpV = svd1.matrixV();
      Eigen::Matrix3d finalR1 = tmpU * tmpV.transpose();
      if (finalR1.determinant() < 0) {
          finalR1.col(2) = - finalR1.col(2);
      }
      strs.str("");
      strs << "sol2: using 3 axis: realR1=" <<  relR1 << ", svd, tmpU=" << tmpU << ", tmpV=" << tmpV << ", finalR1=" << finalR1 <<  std::endl;
      LOG_INFO(strs);
      */
      Vec xv(finalR.col(0));
      Vec yv(finalR.col(1));
      Vec zv(finalR.col(2));
      Rotation relRR(xv, yv, zv);
      Quaternion q1;
      relRR.GetQuaternion(&q1);

      sub_tcp_uncal->segment(0, 3) = relT;
      (*sub_tcp_uncal)(3) = q1.w();
      (*sub_tcp_uncal)(4) = q1.x();
      (*sub_tcp_uncal)(5) = q1.y();
      (*sub_tcp_uncal)(6) = q1.z();
      
      *sub_tcp_cal = *sub_tcp_uncal;  // set sub_tcp_cal and sub_tcp_uncal as same

      // now use subTCP doing some test
      strs.str("");
      for (size_t i=0; i<num_ind_jnts; i++) {
        Eigen::Vector3d  pred_o = tt[i] + RR[i] * relT;
        //Eigen::Vector3d  pred_op = tt[1] + RR[1] * relT;
        strs << " meausred o=" << origP[5 * i] << ", predicted o=" << pred_o  << std::endl;
              // ", measured op=" << origP[4] << ", predicted op=" << pred_op << std::endl;
        
        Eigen::Vector3d  pred_ox = RR[i] * relR.col(0);
        Eigen::Vector3d  pred_oy = RR[i] * relR.col(1);
        //Eigen::Vector3d  pred_oz = RR[i] * relR.col(2);

        strs << "measured ox=" << (origP[5*i + 2] - origP[5 *i + 1]) / 2.0 
             << ",measured ox 2=" <<  (origP[5*i + 3] - origP[5 *i + 4]) / 2.0 << ", predicted ox=" << pred_ox << std::endl;
        strs << "measured oy=" << (origP[5 *i + 2] - origP[5 * i + 3]) / 2.0
             << "measured oy2=" << (origP[5 *i + 1] - origP[5 * i + 4]) / 2.0  << ", predicted oy=" << pred_oy << std::endl;
        //strs << "measured oz=" << origP[4 *i + 3] - origP[4 * i]  << ", predicted oz=" << pred_oz << std::endl;
      }
      LOG_INFO(strs);
      return 0;
    } catch (...) {
      strs.str("");
      strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
      LOG_ERROR(strs);
      return -1111;
    }
  }

  int XYZ_UR::ErrCompensateBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 EigenDRef<Eigen::VectorXd> *comp_base_uncal,
                                 EigenDRef<Eigen::VectorXd> *comp_base,
                                 EigenDRef<Eigen::MatrixXd> *origCart,
                                 EigenDRef<Eigen::MatrixXd> *compCart,
                                 EigenDRef<Eigen::VectorXd> *tcp_UR_uncal,    // tcp of UR workobj w.r.t UR
                                 EigenDRef<Eigen::VectorXd> *tcp_UR_cal    // tcp of EOAT of xyz w.r.t. XYZ
                                 ) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    size_t numJnts = jnt_base_measures.cols();
    // here orig_base is the workpiece frame, should be reachable by IK
    if (jnt_base_measures.rows() < DoF_ || numJnts < 1) {
      strs.str("");
      strs << GetName() << ":" << "The input vector has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
    if (!comp_base_uncal || !comp_base || !origCart || !compCart || !tcp_UR_uncal || !tcp_UR_cal) {
      strs.str("");
      strs << GetName() << ":"  << "input comp_base is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }  
    
    Eigen::MatrixXd jnts_UR = jnt_base_measures.block(XYZ->GetDoF(),  0, UR->GetDoF(), numJnts);
    Eigen::MatrixXd jnts_XYZ = jnt_base_measures.block(0,  0, XYZ->GetDoF(), numJnts);

    // using XYZ 8pt method to compute the base 
    int ret = XYZ->ErrCompensateBase(jnts_XYZ, orig_tool, comp_base_uncal, comp_base,
                                     origCart, compCart, tcp_UR_uncal, tcp_UR_cal);
    if (ret < 0) {
      return ret;
    }

    // note tcp_UR and tcp_XYZ will be overrided by UR workobj calib
    (*tcp_UR_uncal).setZero();
    (*tcp_UR_cal).setZero();


    // first check if all jnts are same
    for (size_t j=1; j < numJnts; j++) {
      Eigen::VectorXd diff_jnt_UR = jnts_UR.col(j) - jnts_UR.col(0);
      if (diff_jnt_UR.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":" << "UR joints in each plane is not same"
                << " , diff_jnt_UR=" << diff_jnt_UR << ", in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return -2001; 
      }
    }
    // second, compute the FK of UR
    UR->SetUsingCalibratedModel(false);
    std::vector<double> jnt(UR->GetDoF(), 0);
    EigenVec2StdVec(jnts_UR.col(0), &jnt);
    Pose ps;
    ret = UR->JntToCart(jnt, &ps);
    if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << " FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
    }
    Frame flange;
    ps.getFrame(&flange);
    Eigen::VectorXd  v_uncal =  comp_base_uncal->segment(0, 3);
    Eigen::VectorXd q_uncal = comp_base_uncal->segment(3, 4);
    Vec v_u(v_uncal);
    Quaternion q_u = Quaternion::FromEigenVec(q_uncal);
    Frame block_u(q_u, v_u); 
    Frame rel_block_u = flange.Inverse() * block_u;

    strs.str("");
    strs << GetName() << ": comp_base_uncal=" << block_u.ToString(true) << std::endl;
    strs << " jnts_UR=" << jnts_UR << ", uncal_flage=" << flange.ToString(false) << ", in euler=" << flange.ToString(true) << std::endl;
    strs << "rel_block_u=" << rel_block_u.ToString(true) << std::endl;
    LOG_INFO(strs);
    tcp_UR_uncal->segment(0, 3) = rel_block_u.getTranslation().ToEigenVec();
    tcp_UR_uncal->segment(3, 4) = rel_block_u.getQuaternion().ToEigenVec();
    
    

    UR->SetUsingCalibratedModel(true);
    ret = UR->JntToCart(jnt, &ps);
    if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << " (calibrated) FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
    }
    ps.getFrame(&flange);
    Eigen::VectorXd  v_cal =  comp_base->segment(0, 3);
    Eigen::VectorXd q_cal = comp_base->segment(3, 4);
    Vec v_c(v_cal);
    Quaternion q_c = Quaternion::FromEigenVec(q_cal);
    Frame block_c(q_c, v_c); 
    Frame rel_block_c = flange.Inverse() * block_c;

    strs.str("");
    strs << GetName() << ":comp_base=" << block_c.ToString(true) << std::endl;
    strs << ", jnts_UR=" << jnts_UR << ", cal_flage=" << flange.ToString(false) << ", in euler=" << flange.ToString(true) << std::endl;
    strs << "rel_block_c=" << rel_block_c.ToString(true) << std::endl;
    LOG_INFO(strs);

    tcp_UR_cal->segment(0, 3) = rel_block_c.getTranslation().ToEigenVec();
    tcp_UR_cal->segment(3, 4) = rel_block_c.getQuaternion().ToEigenVec();
    return 0;
  }

  int XYZ_UR::ErrCompensateBase(const Eigen::MatrixXd &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 Eigen::VectorXd *tcp_UR_uncal,
                                 Eigen::VectorXd *tcp_UR_cal
                                 ) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    size_t numJnts = jnt_base_measures.cols();
    // here orig_base is the workpiece frame, should be reachable by IK
    if (jnt_base_measures.rows() < DoF_ || numJnts < 1) {
      strs.str("");
      strs << GetName() << ":" << "The input vector has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
    if (!tcp_UR_uncal || !tcp_UR_cal) {
      strs.str("");
      strs << GetName() << ":"  << "input comp_base is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }  
    
    Eigen::MatrixXd jnts_UR = jnt_base_measures.block(XYZ->GetDoF(),  0, UR->GetDoF(), numJnts);
    Eigen::MatrixXd jnts_XYZ = jnt_base_measures.block(0,  0, XYZ->GetDoF(), numJnts);

    // using XYZ 8pt method to compute the base
    //Eigen::MatrixXd  origCart(6, 8), compCart(6, 8);
    Eigen::VectorXd  comp_base(7), comp_base_uncal(7); 
    int ret = XYZ->ErrCompensateBase(jnts_XYZ, orig_tool, &comp_base_uncal, &comp_base);
    if (ret < 0) {
      return ret;
    }

    // note tcp_UR and tcp_XYZ will be overrided by UR workobj calib
    (*tcp_UR_uncal).setZero();
    (*tcp_UR_cal).setZero();


    // first check if all jnts are same
    for (size_t j=1; j < numJnts; j++) {
      Eigen::VectorXd diff_jnt_UR = jnts_UR.col(j) - jnts_UR.col(0);
      if (diff_jnt_UR.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":" << "UR joints in each plane is not same"
                << " , diff_jnt_UR=" << diff_jnt_UR << ", in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return -2001; 
      }
    }
    // second, compute the FK of UR
    SetUsingCalibratedModel(false);
    std::vector<double> jnt(UR->GetDoF(), 0);
    EigenVec2StdVec(jnts_UR.col(0), &jnt);
    Pose ps;
    ret = UR->JntToCart(jnt, &ps);
    if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << " FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
    }
    Frame flange;
    ps.getFrame(&flange);
    Eigen::VectorXd  v_uncal =  comp_base_uncal.segment(0, 3);
    Eigen::VectorXd q_uncal = comp_base_uncal.segment(3, 4);
    Vec v_u(v_uncal);
    Quaternion q_u = Quaternion::FromEigenVec(q_uncal);
    Frame block_u(q_u, v_u); 
    Frame rel_block_u = flange.Inverse() * block_u;

    strs.str("");
    strs << GetName() << ": comp_base_uncal=" << block_u.ToString(true) << std::endl;
    strs << " jnts_UR=" << jnts_UR << ", uncal_flage=" << flange.ToString(false) << ", in euler=" << flange.ToString(true) << std::endl;
    strs << "rel_block_u=" << rel_block_u.ToString(true) << std::endl;
    LOG_INFO(strs);
    tcp_UR_uncal->segment(0, 3) = rel_block_u.getTranslation().ToEigenVec();
    tcp_UR_uncal->segment(3, 4) = rel_block_u.getQuaternion().ToEigenVec();
    
    

    SetUsingCalibratedModel(true);
    ret = UR->JntToCart(jnt, &ps);
    if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << " (calibrated) FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
    }
    ps.getFrame(&flange);
    Eigen::VectorXd  v_cal =  comp_base.segment(0, 3);
    Eigen::VectorXd q_cal = comp_base.segment(3, 4);
    Vec v_c(v_cal);
    Quaternion q_c = Quaternion::FromEigenVec(q_cal);
    Frame block_c(q_c, v_c); 
    Frame rel_block_c = flange.Inverse() * block_c;

    strs.str("");
    strs << GetName() << ":comp_base=" << block_c.ToString(true) << std::endl;
    strs << ", jnts_UR=" << jnts_UR << ", cal_flage=" << flange.ToString(false) << ", in euler=" << flange.ToString(true) << std::endl;
    strs << "rel_block_c=" << rel_block_c.ToString(true) << std::endl;
    LOG_INFO(strs);

    tcp_UR_cal->segment(0, 3) = rel_block_c.getTranslation().ToEigenVec();
    tcp_UR_cal->segment(3, 4) = rel_block_c.getQuaternion().ToEigenVec();
    return 0;
  }

  // 5 axis compensation is divided into two steps
  // step 1: 8 pt to determine the TCP of block w.r.t. flange of UR mechanism
  // step 2: XYZ mechanism to follow desired traj given desired jnt (or desired orientation ) of
  // UR and desired relative XYZ w.r.t. block 
  int  XYZ_UR::ErrCompensationDH(
        const Eigen::VectorXd &calibTCP,   //  calibrated TCP of block w.r.t. UR flange 
        const Eigen::VectorXd &origTCP,    // original TCP of block w.r.t. UR flange
        const Eigen::VectorXd &bestTool,    // tool on XYZ, such as dispensing tool
        const EigenDRef<Eigen::MatrixXd> &d_traj,   // (x,y,z, U, R) ?  or (x,y,z, q0, q1, q2, q3)  (q=(q0, q1, q2, q3) if computed from roll, pitch, yaw is for flange)
        EigenDRef<Eigen::MatrixXd> *md_traj, 
        EigenDRef<Eigen::MatrixXd> *d_j_traj,
        EigenDRef<Eigen::MatrixXd> *md_j_traj,
        EigenDRef<Eigen::MatrixXd> *a_traj) {
    std::ostringstream strs;
    if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
      strs.str("");
      strs << GetName() << ":" << "Input pointer is null"
                << " so can not do compensation, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
       
    }
    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << " robot is not calibrated, "
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_WITHOUT_CALIB;  
    }
    // generate base and tool frame from the input data
    if (calibTCP.rows() !=7  || origTCP.rows() != 7 || bestTool.rows() !=7) {
      strs.str("");
      strs << GetName() << ":" << "Input Tcp/tool data is not translation + quaternion"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;   
    }
    strs.str("");
    strs << GetName() << ":" << "In traj. compensation: calib UR Tcp= " << calibTCP
         << ", uncalibrated UR tcp= " << origTCP
         << ", desired tool=" << bestTool
        << std::endl;
    LOG_INFO(strs);

    size_t numPts = d_traj.cols();
    size_t numRows = d_traj.rows();
    // resize output variables
    md_traj->resize(9, numPts);
    d_j_traj->resize(DoF_, numPts);
    md_j_traj->resize(DoF_, numPts);
    a_traj->resize(9, numPts);

    
    // XYZ tool
    Frame newXYZ_Tool;
    Vec t(bestTool.segment(0, 3));
    newXYZ_Tool.setTranslation(t);
    Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
    newXYZ_Tool.setQuaternion(q);

    // calibrated TCP of block
    Vec bv(calibTCP(0), calibTCP(1), calibTCP(2));
    Quaternion bq(calibTCP(3), calibTCP(4), calibTCP(5), calibTCP(6));
    Frame newBaseTCP(bq, bv);

    // original TCP of block
    Vec bv_old(origTCP(0), origTCP(1), origTCP(2));
    Quaternion bq_old(origTCP(3), origTCP(4), origTCP(5), origTCP(6));
    Frame oldBaseTCP(bq_old, bv_old);

    // At every pt in desired traj, we need to  compute ref UR pose in calibrated model, and in orig model
    refPose rf_pose_UR_calib, rf_pose_UR_orig;

    // default pose in orig and calib
    Pose df_pose_UR_orig, df_pose_UR_calib;

    // compensated UR joints
    Eigen::VectorXd act_UR;

    // At every pt in desired traj, what is calibrated (accurate) UR joint and orig (or errored/not accurate) UR joint
    std::vector<double> qUR_cal(UR->GetDoF(), 0), qUR_orig(UR->GetDoF(), 0);
    // next we need to compute, at very pt, the calib / orig base of block w.r.t. XYZ base using pose_UR_calib
    // and pose_UR_orig and calibTCP /origTCP
    Frame calibBase, origBase;

    int ret = 0;
    // boolean to denote whether desired traj file contains UR desired joint or desired orientation 
    bool URJnt = d_traj.rows() <= 5 ? true : false;
    for (size_t i=0; i < numPts; i++) {
      Eigen::VectorXd cur_pt = d_traj.col(i);
      strs.str("");
      strs << "cur_pt = " << cur_pt.transpose() << std::endl;
      LOG_INFO(strs);
      if (URJnt) { // if desired traj contains desired UR joint traj
        // note: here act_UR is already the modified UR traj
        act_UR = cur_pt.segment(XYZ->GetDoF(), UR->GetDoF());
        EigenVec2StdVec(act_UR, &qUR_cal);
  
        qUR_orig = qUR_cal;   // if desired traj was given as (x,y,z,Ud,Rd), then qUR_cal = qUR_orig={Ud, Rd}
      } else { // input traj contains UR desired orientation info w.r.t. current subTCP
        Quaternion qtmp(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
        Vec v(0, 0, 0);
        size_t inFlag = cur_pt(7);
        // for scara, there is only 1 branch flag: elbow (up or down), and for six-axis robot, there are 3 flags
        std::vector<int>  branchFlags;
        ConvertBranchFlag(inFlag, &branchFlags);
        // convert joint turn data into ikJointTurns
        size_t tFlag = cur_pt(8);
        std::vector<int> ikJointTurns(DoF_, 0);
        ConvertMultiTurnFlag(tFlag, &ikJointTurns);
        std::vector<int> ikTurns_UR(UR->GetDoF(), 0);
        for (size_t j=0; j < UR->GetDoF(); j++) {
            ikTurns_UR[j] =  ikJointTurns[j + XYZ->GetDoF()];
        }
        
        Frame tmp_UR_fm(qtmp, v);  // initially set v as 0, and just use ideal orientation to compute qUR_orig and qUR_cal
        rf_pose_UR_orig.setFrame(tmp_UR_fm);
        rf_pose_UR_orig.setBranchFlags(branchFlags);
        rf_pose_UR_orig.setJointTurns(ikTurns_UR);
        rf_pose_UR_orig.setTool(newBaseTCP);  // here we have use newBaseTCP, but not oldBaseTCP, because otherwise there will be more err.


        // default pose in default base and tool
        rf_pose_UR_orig.getDefaultPose(&df_pose_UR_orig);
        UR->SetUsingCalibratedModel(false);  // using uncalibrated model
        ret = UR->CartToJnt(df_pose_UR_orig, &qUR_orig); // compute ideal joint from idela orientation
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " IK error, code  " << ret << ", cart is " << df_pose_UR_orig.ToString(false)
              << "can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        Eigen::VectorXd  qUR_orig_vec;
        StdVec2EigenVec(qUR_orig, &qUR_orig_vec);
        strs.str("");
        strs << GetName() << ":, ideal UR joint=" << qUR_orig_vec << std::endl;
        LOG_INFO(strs); 
       
        /*
        strs.str("");
        strs << GetName() << " after canonical IK() pose_UR_orig  " << pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_INFO(strs);
         */

        // using uncalibrated model to compute the final ideal pose (because whose trans might not be 0)
        /*
        ret = UR->JntToCart(qUR_orig, &df_Pos_UR_orig);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " <<  qUR_orig_vec
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
        } */

        /*
        strs.str("");
        strs << GetName() << " after canonical FK() pose_UR_orig  " << pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_INFO(strs);
        */

        // set 2: compute the compensated jnt
        // refPose  ref_pose_UR_orig;
        //refPose  tmpRefOrig;
        //tmpRefOrig.setDefaultPose(df_pose_UR_orig);
        //tmpRefOrig.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
        ret = UR->OptimizeJntAfterCalib(qUR_orig, rf_pose_UR_orig, &qUR_cal);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "UR OptimizeJntAfterCalib error, code  " << ret
                << "can not do error compensation in "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        StdVec2EigenVec(qUR_cal, &act_UR);
        strs.str("");
        strs << GetName() << ":, compensated UR joint=" << act_UR << std::endl;
        LOG_INFO(strs); 
      }
      // using uncalibrated model to compute the comp (or modified) orientation
      UR->SetUsingCalibratedModel(false);
      ret = UR->JntToCart(qUR_cal, &df_pose_UR_calib);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      refPose  def_pose;
      def_pose.setDefaultPose(df_pose_UR_calib);
      def_pose.getPoseUnderNewRef(Frame(), oldBaseTCP, &rf_pose_UR_calib);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_calib inside URJnt=" << rf_pose_UR_calib.ToString(true) << std::endl;
      LOG_INFO(strs);

      // using calibrated model to compute the ideal UR pose 
      UR->SetUsingCalibratedModel(true);
      ret = UR->JntToCart(qUR_cal, &df_pose_UR_orig);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      def_pose.setDefaultPose(df_pose_UR_orig);
      def_pose.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true) << std::endl;
      LOG_INFO(strs); 

      // ideal UR turns and flags
      std::vector<int> UR_flags_orig, UR_flags_cal;
      std::vector<int> UR_turns_orig, UR_turns_cal;
      rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
      rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
      rf_pose_UR_calib.getBranchFlags(&UR_flags_cal);
      rf_pose_UR_calib.getJointTurns(&UR_turns_cal);

      
      
      Eigen::VectorXd  d_UR_j_traj(UR->GetDoF()), md_UR_j_traj(UR->GetDoF());
      StdVec2EigenVec(qUR_orig, &d_UR_j_traj); // ideal model  joint traj
      StdVec2EigenVec(qUR_cal, &md_UR_j_traj); // compensated joint traj

      
      // now we able able to compute calibBase and origBase
      // **** note *** python GUI need to add option to set up newBaseTCP and oldBaseTCP if 8pt is not used
      Frame tmp_orig;
      df_pose_UR_orig.getFrame(&tmp_orig);
      calibBase = tmp_orig * newBaseTCP;  // ideal base for XYZ
      origBase = tmp_orig * oldBaseTCP;    // error base for XYZ   (note: here we use both pose_UR_orig (ideal UR pose) to decouple UR and XYZ comp) 
      strs.str("");
      strs << "frame_UR_orig=" << tmp_orig.ToString(true) << std::endl;
      strs << GetName() << ": newBaseTCP=" << newBaseTCP.ToString(true)
      << ", oldBaseTCP=" << oldBaseTCP.ToString(true) << std::endl;
      strs << GetName() << ", calibBase=" << calibBase.ToString(true) <<
          ", origBase=" << origBase.ToString(true) << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);

      // the following is just simply using XYZ API ErrCompsationDH to compensate XYZ traj
      Vec v1(cur_pt.segment(0, 3));
      Frame d_traj_fm;
      d_traj_fm.setTranslation(v1);
      Rotation r =  calibBase.getRotation().Inverse(); // combined with calibBase, their rotation should be identity, because xyz only accepts identity orientation
      d_traj_fm.setRotation(r);

      std::vector<int> ikXYZTurns(XYZ->GetDoF(), 0);
      std::vector<int> branchXYZ(3, 0);
      std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(), ikXYZTurns.end());
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(), UR_turns_orig.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(), ikXYZTurns.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(), UR_turns_cal.end());


      // desired relative pose w.r.t. block base
      Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
      // modified pose relative to old block base/ actual pose relative calib block base
      Pose mdPose, aPose;
      // ideal uncompensated xyz traj / compensated xyz traj 
      Eigen::VectorXd d_XYZ_j_traj(XYZ->GetDoF());
      Eigen::VectorXd md_XYZ_j_traj(XYZ->GetDoF());

      // step 2: call XYZ->CompSateEachPt (to be done in serialArm.cpp)
      ret = XYZ->ErrCompensationDH(calibBase, origBase,
      newXYZ_Tool, dPose, &mdPose, &d_XYZ_j_traj,
      &md_XYZ_j_traj, &aPose);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " XYZ->ErrCompensationDH error  " << ret << ", cart is " << dPose.ToString(true)
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      d_j_traj->col(i).segment(0, XYZ->GetDoF()) = d_XYZ_j_traj;
      d_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = d_UR_j_traj;



      dPose.setQuaternion(df_pose_UR_orig.getQuaternion());
      dPose.setBranchFlags(UR_flags_orig);
      dPose.setJointTurns(XYZURTurns_orig);
      mdPose.setQuaternion(df_pose_UR_calib.getQuaternion());

      mdPose.setBranchFlags(UR_flags_cal);
      mdPose.setJointTurns(XYZURTurns_cal);
      aPose.setQuaternion(df_pose_UR_orig.getQuaternion());
      aPose.setBranchFlags(UR_flags_orig);
      aPose.setJointTurns(XYZURTurns_orig);
      md_traj->col(i) = mdPose.ToEigenVecPose(); 
      a_traj->col(i) = aPose.ToEigenVecPose();  // a_traj will return desired traj, aPose is just for logging purpose
      
      md_j_traj->col(i).segment(0, XYZ->GetDoF()) = md_XYZ_j_traj;
      md_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = md_UR_j_traj;
      
      strs.str("");
      strs << GetName() << " pt  " <<  i 
           << " desired joint " << d_j_traj->col(i).transpose() << ", comp joint " << md_j_traj->col(i).transpose() << std::endl
           << " desired cart " << dPose.ToString(true) <<  ", comp cart " << mdPose.ToString(true)
           << ", actual cart " << aPose.ToString(true) << std::endl
           << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);
    }
    return 0;
  }

  int  XYZ_UR::MultiLocFiltering(
        const Eigen::MatrixXd &calibWkObj,  // actual user base (or calibrated base) at each Loc of multiLocs, relative to which, desired relative path is based upon
        const Eigen::MatrixXd &origWkObj,  // uncalibrated base
        const Eigen::VectorXd &bestTool,  // used tool
        const EigenDRef<Eigen::MatrixXd> &d_traj,  // desired relative path (usually based upon CAD file)
        EigenDRef<Eigen::MatrixXd> *md_traj,
        EigenDRef<Eigen::MatrixXd> *d_j_traj,
        EigenDRef<Eigen::MatrixXd> *md_j_traj,
        EigenDRef<Eigen::MatrixXd> *a_traj) {
      std::ostringstream strs;
    if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
      strs.str("");
      strs << GetName() << ":" << "Input pointer is null"
                << " so can not do compensation, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
       
    }
    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << " robot is not calibrated, "
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_WITHOUT_CALIB;  
    }
    // generate base and tool frame from the input data
    if (calibWkObj.rows() < 7  || origWkObj.rows() < 7 || bestTool.rows() !=7) {
      strs.str("");
      strs << GetName() << ":" << "Input WkObj/tool data is not translation + quaternion"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;
    }
    
    // XYZ tool
    Frame newXYZ_Tool;
    Vec t(bestTool.segment(0, 3));
    newXYZ_Tool.setTranslation(t);
    Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
    newXYZ_Tool.setQuaternion(q);

    size_t numPts = d_traj.cols();
    size_t numWkObjs = calibWkObj.cols();
    if (numPts < 1 || numWkObjs < 1) {
      strs.str("");
      strs << GetName() << ":" << "Input desired traj or wkobj has too less number of points" << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;
    }
    size_t numRows = d_traj.rows();
    // resize output variables
    md_traj->resize(9, numPts);
    d_j_traj->resize(DoF_, numPts);
    md_j_traj->resize(DoF_, numPts);
    a_traj->resize(9, numPts);
    

     // next we need to compute, at very pt, the calib / orig base of block w.r.t. XYZ base using pose_UR_calib
    // and pose_UR_orig and calibTCP /origTCP
    Frame calibBase, origBase;
    size_t wkobj_index = 0;
    Eigen::VectorXd wkobj_base = calibWkObj.col(wkobj_index).segment(0, 7);
    Eigen::VectorXd wkobj_UR = calibWkObj.col(wkobj_index).segment(7, 2); 
    for (size_t i=0; i < numPts; i++) {
      Eigen::VectorXd cur_pt = d_traj.col(i);
      strs.str("");
      strs << "cur_pt = " << cur_pt.transpose() << std::endl;
      LOG_INFO(strs);
      Eigen::VectorXd act_UR = cur_pt.segment(XYZ->GetDoF(), UR->GetDoF());
    
      // check if act_UR is belong to current wkobj index
      Eigen::VectorXd diff_UR = act_UR - wkobj_UR;
      if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
         wkobj_index++;
         if (wkobj_index >= numWkObjs) {
            strs.str("");
            strs << GetName() << ":" << "wkobj_index=" << wkobj_index << ">= numWkObjs=" << numWkObjs << std::endl;
            LOG_ERROR(strs);
            return -ERR_INPUT_PARA_WRONG_DIM;
         }

         wkobj_base = calibWkObj.col(wkobj_index).segment(0, 7);
         wkobj_UR = calibWkObj.col(wkobj_index).segment(7, 2);
         diff_UR = act_UR - wkobj_UR;
         if (diff_UR.norm() >= MAX_THETA_DIFF_REG) {
            strs.str("");
            strs << GetName() << " wkobj data" << wkobj_UR << "not match with desire traj UR data " << act_UR << std::endl;
            LOG_ERROR(strs);
            return -ERR_INPUT_PARA_WRONG_DIM;
         }
      }
      
      Vec t(wkobj_base.segment(0, 3));
      calibBase.setTranslation(t);
      Quaternion q(wkobj_base(3), wkobj_base(4), wkobj_base(5), wkobj_base(6));
      calibBase.setQuaternion(q);
      origBase = calibBase;
      strs.str("");
      strs << GetName() << ", calibBase=" << calibBase.ToString(true) <<
          ", origBase=" << origBase.ToString(true) << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);

      // the following is just simply using XYZ API ErrCompsationDH to compensate XYZ traj
      Vec v1(cur_pt.segment(0, 3));
      Frame d_traj_fm;
      d_traj_fm.setTranslation(v1);
      Rotation r =  calibBase.getRotation().Inverse(); // combined with calibBase, their rotation should be identity, because xyz only accepts identity orientation
      d_traj_fm.setRotation(r);
      
      std::vector<int> ikXYZTurns(XYZ->GetDoF(), 0);
      std::vector<int> branchXYZ(3, 0);

      // desired relative pose w.r.t. block base
      Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
      // modified pose relative to old block base/ actual pose relative calib block base
      Pose mdPose, aPose;
      // ideal uncompensated xyz traj / compensated xyz traj 
      Eigen::VectorXd d_XYZ_j_traj(XYZ->GetDoF());
      Eigen::VectorXd md_XYZ_j_traj(XYZ->GetDoF());

      // step 2: call XYZ->CompSateEachPt (to be done in serialArm.cpp)
      int ret = XYZ->ErrCompensationDH(calibBase, origBase,
      newXYZ_Tool, dPose, &mdPose, &d_XYZ_j_traj,
      &md_XYZ_j_traj, &aPose);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " XYZ->ErrCompensationDH error  " << ret << ", cart is " << dPose.ToString(true)
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      d_j_traj->col(i).segment(0, XYZ->GetDoF()) = d_XYZ_j_traj;
      d_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = act_UR;


      md_traj->col(i) = mdPose.ToEigenVecPose(); 
      a_traj->col(i) = aPose.ToEigenVecPose();  // a_traj will return desired traj, aPose is just for logging purpose
      
      md_j_traj->col(i).segment(0, XYZ->GetDoF()) = md_XYZ_j_traj;
      md_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = act_UR;
      
      strs.str("");
      strs << GetName() << " pt  " <<  i 
           << " desired joint " << d_j_traj->col(i).transpose() << ", comp joint " << md_j_traj->col(i).transpose() << std::endl
           << " desired cart " << dPose.ToString(true) <<  ", comp cart " << mdPose.ToString(true)
           << ", actual cart " << aPose.ToString(true) << std::endl
           << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);
    }
    return 0;
  }

  int  XYZ_UR::PathFiltering(
        const Eigen::VectorXd &calibTCP,   //  calibrated TCP of block w.r.t. UR flange 
        const Eigen::VectorXd &origTCP,    // original TCP of block w.r.t. UR flange
        const Eigen::VectorXd &bestTool,
        const Eigen::MatrixXd &d_traj,
        Eigen::MatrixXd *md_traj,
        Eigen::MatrixXd *d_j_traj,
        Eigen::MatrixXd *md_j_traj,
        Eigen::MatrixXd *a_traj) {
    std::ostringstream strs;
    if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
      strs.str("");
      strs << GetName() << ":" << "Input pointer is null"
                << " so can not do compensation, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
       
    }
    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << " robot is not calibrated, "
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_WITHOUT_CALIB;  
    }
    // generate base and tool frame from the input data
    if (calibTCP.rows() !=7  || origTCP.rows() != 7 || bestTool.rows() !=7) {
      strs.str("");
      strs << GetName() << ":" << "Input Tcp/tool data is not translation + quaternion"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;   
    }
    strs.str("");
    strs << GetName() << ":" << "In traj. compensation: calib UR Tcp= " << calibTCP
         << ", uncalibrated UR tcp= " << origTCP
         << ", desired tool=" << bestTool
        << std::endl;
    LOG_INFO(strs);

    size_t numPts = d_traj.cols();
    size_t numRows = d_traj.rows();
    // resize output variables
    md_traj->resize(9, numPts);
    d_j_traj->resize(DoF_, numPts);
    md_j_traj->resize(DoF_, numPts);
    a_traj->resize(9, numPts);

    
    // XYZ tool
    Frame newXYZ_Tool;
    Vec t(bestTool.segment(0, 3));
    newXYZ_Tool.setTranslation(t);
    Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
    newXYZ_Tool.setQuaternion(q);

    // calibrated TCP of block
    Vec bv(calibTCP(0), calibTCP(1), calibTCP(2));
    Quaternion bq(calibTCP(3), calibTCP(4), calibTCP(5), calibTCP(6));
    Frame newBaseTCP(bq, bv);

    // original TCP of block
    Vec bv_old(origTCP(0), origTCP(1), origTCP(2));
    Quaternion bq_old(origTCP(3), origTCP(4), origTCP(5), origTCP(6));
    Frame oldBaseTCP(bq_old, bv_old);

    // At every pt in desired traj, we need to  compute ref UR pose in calibrated model, and in orig model
    refPose rf_pose_UR_calib, rf_pose_UR_orig;

    // default pose in orig and calib
    Pose df_pose_UR_orig, df_pose_UR_calib;

    // compensated UR joints
    Eigen::VectorXd act_UR;

    // At every pt in desired traj, what is calibrated (accurate) UR joint and orig (or errored/not accurate) UR joint
    std::vector<double> qUR_cal(UR->GetDoF(), 0), qUR_orig(UR->GetDoF(), 0);
    // next we need to compute, at very pt, the calib / orig base of block w.r.t. XYZ base using pose_UR_calib
    // and pose_UR_orig and calibTCP /origTCP
    Frame calibBase, origBase;

    int ret = 0;
    // boolean to denote whether desired traj file contains UR desired joint or desired orientation 
    bool URJnt = d_traj.rows() <= 5 ? true : false;
    for (size_t i=0; i < numPts; i++) {
      Eigen::VectorXd cur_pt = d_traj.col(i);
      strs.str("");
      strs << "cur_pt = " << cur_pt.transpose() << std::endl;
      LOG_INFO(strs);
      if (URJnt) { // if desired traj contains desired UR joint traj
        // note: here act_UR is already the modified UR traj
        act_UR = cur_pt.segment(XYZ->GetDoF(), UR->GetDoF());
        EigenVec2StdVec(act_UR, &qUR_cal);
  
        qUR_orig = qUR_cal;   // if desired traj was given as (x,y,z,Ud,Rd), then qUR_cal = qUR_orig={Ud, Rd}
      } else { // input traj contains UR desired orientation info w.r.t. current subTCP
        Quaternion qtmp(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
        Vec v(0, 0, 0);
        size_t inFlag = cur_pt(7);
        // for scara, there is only 1 branch flag: elbow (up or down), and for six-axis robot, there are 3 flags
        std::vector<int>  branchFlags;
        ConvertBranchFlag(inFlag, &branchFlags);
        // convert joint turn data into ikJointTurns
        size_t tFlag = cur_pt(8);
        std::vector<int> ikJointTurns(DoF_, 0);
        ConvertMultiTurnFlag(tFlag, &ikJointTurns);
        std::vector<int> ikTurns_UR(UR->GetDoF(), 0);
        for (size_t j=0; j < UR->GetDoF(); j++) {
            ikTurns_UR[j] =  ikJointTurns[j + XYZ->GetDoF()];
        }
        
        Frame tmp_UR_fm(qtmp, v);  // initially set v as 0, and just use ideal orientation to compute qUR_orig and qUR_cal
        rf_pose_UR_orig.setFrame(tmp_UR_fm);
        rf_pose_UR_orig.setBranchFlags(branchFlags);
        rf_pose_UR_orig.setJointTurns(ikTurns_UR);
        rf_pose_UR_orig.setTool(newBaseTCP);  // here we have use newBaseTCP, but not oldBaseTCP, because otherwise there will be more err.


        // default pose in default base and tool
        rf_pose_UR_orig.getDefaultPose(&df_pose_UR_orig);
        UR->SetUsingCalibratedModel(false);  // using uncalibrated model
        ret = UR->CartToJnt(df_pose_UR_orig, &qUR_orig); // compute ideal joint from idela orientation
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " IK error, code  " << ret << ", cart is " << df_pose_UR_orig.ToString(false)
              << "can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        Eigen::VectorXd  qUR_orig_vec;
        StdVec2EigenVec(qUR_orig, &qUR_orig_vec);
        strs.str("");
        strs << GetName() << ":, ideal UR joint=" << qUR_orig_vec << std::endl;
        LOG_INFO(strs); 
       
        /*
        strs.str("");
        strs << GetName() << " after canonical IK() pose_UR_orig  " << pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_INFO(strs);
         */

        // using uncalibrated model to compute the final ideal pose (because whose trans might not be 0)
        /*
        ret = UR->JntToCart(qUR_orig, &df_Pos_UR_orig);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " <<  qUR_orig_vec
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
        } */

        /*
        strs.str("");
        strs << GetName() << " after canonical FK() pose_UR_orig  " << pose_UR_orig.ToString(true) << ", quR_orig= " <<  qUR_orig_vec
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_INFO(strs);
        */

        // set 2: compute the compensated jnt
        // refPose  ref_pose_UR_orig;
        //refPose  tmpRefOrig;
        //tmpRefOrig.setDefaultPose(df_pose_UR_orig);
        //tmpRefOrig.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
        ret = UR->OptimizeJntAfterCalib(qUR_orig, rf_pose_UR_orig, &qUR_cal);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "UR OptimizeJntAfterCalib error, code  " << ret
                << "can not do error compensation in "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
        StdVec2EigenVec(qUR_cal, &act_UR);
        strs.str("");
        strs << GetName() << ":, compensated UR joint=" << act_UR << std::endl;
        LOG_INFO(strs); 
      }
      // using uncalibrated model to compute the comp (or modified) orientation
      UR->SetUsingCalibratedModel(false);
      ret = UR->JntToCart(qUR_cal, &df_pose_UR_calib);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      refPose  def_pose;
      def_pose.setDefaultPose(df_pose_UR_calib);
      def_pose.getPoseUnderNewRef(Frame(), oldBaseTCP, &rf_pose_UR_calib);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_calib inside URJnt=" << rf_pose_UR_calib.ToString(true) << std::endl;
      LOG_INFO(strs);

      // using calibrated model to compute the ideal UR pose 
      UR->SetUsingCalibratedModel(true);
      ret = UR->JntToCart(qUR_cal, &df_pose_UR_orig);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << act_UR
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      def_pose.setDefaultPose(df_pose_UR_orig);
      def_pose.getPoseUnderNewRef(Frame(), newBaseTCP, &rf_pose_UR_orig);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true) << std::endl;
      LOG_INFO(strs); 

      // ideal UR turns and flags
      std::vector<int> UR_flags_orig, UR_flags_cal;
      std::vector<int> UR_turns_orig, UR_turns_cal;
      rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
      rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
      rf_pose_UR_calib.getBranchFlags(&UR_flags_cal);
      rf_pose_UR_calib.getJointTurns(&UR_turns_cal);

      
      
      Eigen::VectorXd  d_UR_j_traj(UR->GetDoF()), md_UR_j_traj(UR->GetDoF());
      StdVec2EigenVec(qUR_orig, &d_UR_j_traj); // ideal model  joint traj
      StdVec2EigenVec(qUR_cal, &md_UR_j_traj); // compensated joint traj

      
      // now we able able to compute calibBase and origBase
      // **** note *** python GUI need to add option to set up newBaseTCP and oldBaseTCP if 8pt is not used
      Frame tmp_orig;
      df_pose_UR_orig.getFrame(&tmp_orig);
      calibBase = tmp_orig * newBaseTCP;  // ideal base for XYZ
      origBase = tmp_orig * oldBaseTCP;    // error base for XYZ   (note: here we use both pose_UR_orig (ideal UR pose) to decouple UR and XYZ comp) 
      strs.str("");
      strs << "frame_UR_orig=" << tmp_orig.ToString(true) << std::endl;
      strs << GetName() << ": newBaseTCP=" << newBaseTCP.ToString(true)
      << ", oldBaseTCP=" << oldBaseTCP.ToString(true) << std::endl;
      strs << GetName() << ", calibBase=" << calibBase.ToString(true) <<
          ", origBase=" << origBase.ToString(true) << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);

      // the following is just simply using XYZ API ErrCompsationDH to compensate XYZ traj
      Vec v1(cur_pt.segment(0, 3));
      Frame d_traj_fm;
      d_traj_fm.setTranslation(v1);
      Rotation r =  calibBase.getRotation().Inverse(); // combined with calibBase, their rotation should be identity, because xyz only accepts identity orientation
      d_traj_fm.setRotation(r);
      
      std::vector<int> ikXYZTurns(XYZ->GetDoF(), 0);
      std::vector<int> branchXYZ(3, 0);
      std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(), ikXYZTurns.end());
      XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(), UR_turns_orig.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(), ikXYZTurns.end());
      XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(), UR_turns_cal.end());


      // desired relative pose w.r.t. block base
      Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
      // modified pose relative to old block base/ actual pose relative calib block base
      Pose mdPose, aPose;
      // ideal uncompensated xyz traj / compensated xyz traj 
      Eigen::VectorXd d_XYZ_j_traj(XYZ->GetDoF());
      Eigen::VectorXd md_XYZ_j_traj(XYZ->GetDoF());

      // step 2: call XYZ->CompSateEachPt (to be done in serialArm.cpp)
      ret = XYZ->ErrCompensationDH(calibBase, origBase,
      newXYZ_Tool, dPose, &mdPose, &d_XYZ_j_traj,
      &md_XYZ_j_traj, &aPose);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " XYZ->ErrCompensationDH error  " << ret << ", cart is " << dPose.ToString(true)
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      d_j_traj->col(i).segment(0, XYZ->GetDoF()) = d_XYZ_j_traj;
      d_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = d_UR_j_traj;



      dPose.setQuaternion(df_pose_UR_orig.getQuaternion());
      dPose.setBranchFlags(UR_flags_orig);
      dPose.setJointTurns(XYZURTurns_orig);
      mdPose.setQuaternion(df_pose_UR_calib.getQuaternion());

      mdPose.setBranchFlags(UR_flags_cal);
      mdPose.setJointTurns(XYZURTurns_cal);
      aPose.setQuaternion(df_pose_UR_orig.getQuaternion());
      aPose.setBranchFlags(UR_flags_orig);
      aPose.setJointTurns(XYZURTurns_orig);
      md_traj->col(i) = mdPose.ToEigenVecPose(); 
      a_traj->col(i) = aPose.ToEigenVecPose();  // a_traj will return desired traj, aPose is just for logging purpose
      
      md_j_traj->col(i).segment(0, XYZ->GetDoF()) = md_XYZ_j_traj;
      md_j_traj->col(i).segment(XYZ->GetDoF(), UR->GetDoF()) = md_UR_j_traj;
      
      strs.str("");
      strs << GetName() << " pt  " <<  i 
           << " desired joint " << d_j_traj->col(i).transpose() << ", comp joint " << md_j_traj->col(i).transpose() << std::endl
           << " desired cart " << dPose.ToString(true) <<  ", comp cart " << mdPose.ToString(true)
           << ", actual cart " << aPose.ToString(true) << std::endl
           << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);
    }
    return 0;
  }


  double XYZ_UR::CalibrateLaserOrientation(
       const EigenDRef<Eigen::MatrixXd> &jnt_measure, //  | DoF * 6 | DoF * 6 | DoF * 6 | DoF * 6 | ..... Every surface: 1 column points
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // | 8 * 6| 8 * 6| 8 * 6| 8 * 36|  .... every surface:  numPtsInEachPlane pts,  so here each group is 8 * (2 numPtsInEachPlane
                                                        // | p1_low, p2 _low, p3_low, p1_high, p2_high, p3_high | 
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,   // not used
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       // const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const double max_laser_dist, // maximal laser distance, under which, laser reading is 0
       const int numPtsInEachPlane,
       const std::vector<int> surfaceArrays) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    // now processing  cart_measure and laserMat_Z_measure to generate cart sequences in planes
    size_t num_row_jnt = jnt_measure.rows();
    size_t num_col_jnt = jnt_measure.cols();

    size_t num_row_cart = cart_measure.rows();
    size_t num_col_cart = cart_measure.cols();

    size_t num_col_laser = laserMat_z_measure.size();

    
    // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
    bool sizeOK = ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) && (numPtsInEachPlane >= 3) && (laser_channel >=0 && laser_channel <3));
    if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 || (num_col_jnt % numPtsInEachPlane != 0)) {
      strs.str("");
      strs << GetName() << ":" << "CalibrateLaserOrientation: input data dimension is not matching"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ 
                << "num_col_cart=" << num_col_cart << ", num_col_jnt=" << num_col_jnt  << ", num_col_laser=" <<
                num_col_laser << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt << 
                ", value of modular=" << num_col_jnt % numPtsInEachPlane << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    size_t numPlanes = num_col_jnt / numPtsInEachPlane;
    Eigen::MatrixXd UR_jnt_measure(UR->GetDoF(), numPlanes);
    Eigen::MatrixXd UR_cart_measure(num_row_cart, numPtsInEachPlane * numPlanes);

    double laser0 = laser_value;
    // std::vector<Eigen::Vector3d>  ni(numPlanes);
    // std::vector<double>   bi(numPlanes, 0);
    for (size_t i=0; i < numPlanes; i++) {
      Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachPlane, UR->GetDoF(), numPtsInEachPlane);
      Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachPlane, num_row_cart, numPtsInEachPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachPlane, numPtsInEachPlane);
      // first check if all jnts are same
      for (size_t j=1; j < numPtsInEachPlane; j++) {
        Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
        if (diff_jnt.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":" << "CalibrateLaserOrientation: UR joints in each plane is not same"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return -2001; 
        }
      }

      // now modify the carts to make sure they are in the same plane
      UR_jnt_measure.col(i) = jnts.col(0);
      for (size_t j=0; j < numPtsInEachPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - laser0);
        Eigen::VectorXd tmpJnt, tmpCart;
        XYZ->SetUsingCalibratedModel(false);
        if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
               << " at line "  << __LINE__ << std::endl; 
          LOG_ERROR(strs);
          return -2002;
        }
        XYZ->SetUsingCalibratedModel(true);
        if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
               << " at line "  << __LINE__ << std::endl; 
          LOG_ERROR(strs);
          return -2002;
        }
        UR_cart_measure.col(i * numPtsInEachPlane + j) = cart;
      }
    }

    XYZ->SetUsingCalibratedModel(false); // reset into uncalibrated mode
    double ret = UR->CalibrateLaserOrientation(UR_jnt_measure, UR_cart_measure, laserMat_z_measure, 
                 laser_channel, laser_scale, laser_value, max_laser_dist,  numPtsInEachPlane, surfaceArrays);
    if (ret >= 0) { // means UR has been calibrated
      if (XYZ->isCalibrated()) {
        isDHCalibrated_ = true;  // set entire mechanisms as calibrated
      }
    }
    return ret;
  }


  int XYZ_UR::GenerateOriginMeasures(const EigenDRef<Eigen::MatrixXd> &jnt_in,
                                     const int numPtsInPlanes,
                                    EigenDRef<Eigen::MatrixXd> *jnt_out) {
    std::ostringstream strs;
    if (!initialized_ || !jnt_out) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    size_t numPts = jnt_in.cols();
    if (numPts <= numPtsInPlanes) {
      strs.str("");
      strs << GetName() <<  "Number of Input joints " << numPts << " is less than " << numPtsInPlanes << " points in a plane"
                << ", in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -1030; 
    }
    size_t numRows = jnt_in.rows();
    size_t numPlanes = numPts /  numPtsInPlanes;
    jnt_out->resize(numRows, numPts);
    
    // first of all, using first plane to compute the relative coordinates
    Eigen::MatrixXd  relPt(numRows, numPtsInPlanes);
    size_t xyz_dof = XYZ->GetDoF();
    size_t ur_dof = UR->GetDoF();
    // grab the first 4 ur_jnts and xyz_jnts
    Eigen::MatrixXd ur_jnts = jnt_in.block(xyz_dof, 0, ur_dof, numPtsInPlanes);
    Eigen::VectorXd jnt_ur_eig = ur_jnts.col(0);
    Eigen::MatrixXd xyz_jnts = jnt_in.block(0, 0, xyz_dof, numPtsInPlanes);
    Eigen::VectorXd jnt_xyz_eig = xyz_jnts.col(0);
    // get first ur vector
    std::vector<double> jnt_xyz, jnt_ur;
    EigenVec2StdVec(jnt_ur_eig, &jnt_ur);

    Pose  ps_ur, ps_xyz;
    // set as using calibrated models
    XYZ->SetUsingCalibratedModel(true);
    UR->SetUsingCalibratedModel(true); 
    // compute the first flange frame
    int ret = UR->JntToCart(jnt_ur, &ps_ur);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
            << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret; 
    }
    Frame tmp;
    ps_ur.getFrame(&tmp);
    Vec subtcp_vec =tmp.getTranslation();
    Rotation subtcp_rot = tmp.getRotation();

    // now  maps first 4 xyz vectors back to w.r.t. flange frame
    for (size_t i=0; i< numPtsInPlanes; i++) {
      jnt_xyz_eig = xyz_jnts.col(i);
      EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
      ret = XYZ->JntToCart(jnt_xyz, &ps_xyz);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "XYZ FK error, code  " << ret
                << "can not do error compensation in, input jnt=" << jnt_xyz_eig 
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      Vec xyz_vec = ps_xyz.getTranslation();
      Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
      Eigen::VectorXd relPathPt(5), absPathPt(5);
      relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
      relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      relPt.col(i) = relPathPt;
      absPathPt.segment(0, xyz_dof) = jnt_xyz_eig;
      absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      jnt_out->col(i) = absPathPt;
    }
    strs.str("");
    strs << GetName() << ": relPt=" << relPt << std::endl;
    LOG_INFO(strs);
    //  reassign xyz_jnts as relative pts
    xyz_jnts = relPt.block(0, 0, xyz_dof, numPtsInPlanes);
    // then using relative coordinates maps to designated probing points
    for (size_t i=1; i < numPlanes; i++) {
      ur_jnts = jnt_in.block(xyz_dof, i * numPtsInPlanes, ur_dof, numPtsInPlanes);
      //xyz_jnts = jnt_in.block(0, i * numPtsInPlanes, xyz_dof, numPtsInPlanes);
      jnt_ur_eig = ur_jnts.col(0);
      EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
      // using calibrated model to compute the ideal UR pose 
      ret = UR->JntToCart(jnt_ur, &ps_ur);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << jnt_ur_eig
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      refPose rf_pose_UR_orig;
      rf_pose_UR_orig.setDefaultPose(ps_ur);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true) << std::endl;
      LOG_INFO(strs); 

      // ideal UR turns and flags
      std::vector<int> UR_flags_orig, UR_flags_cal;
      std::vector<int> UR_turns_orig, UR_turns_cal;
      rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
      rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
      rf_pose_UR_orig.getBranchFlags(&UR_flags_cal);
      rf_pose_UR_orig.getJointTurns(&UR_turns_cal);
      
      //Eigen::VectorXd  d_UR_j_traj(UR->GetDoF()), md_UR_j_traj(UR->GetDoF());
      //StdVec2EigenVec(jnt_ur, &d_UR_j_traj); // compensated joint traj
      //StdVec2EigenVec(jnt_ur, &md_UR_j_traj); // compensated joint traj

      
      // now we able able to compute calibBase and origBase
      Frame calibBase, origBase;
      rf_pose_UR_orig.getFrame(&calibBase);
      origBase = calibBase;
      strs.str("");
      strs << GetName() << ", calibBase=" << calibBase.ToString(true) <<
          ", origBase=" << origBase.ToString(true) << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);

      // the following is just simply using XYZ API ErrCompsationDH to compensate XYZ traj
      for (size_t j=0; j<numPtsInPlanes; j++) {
          jnt_xyz_eig = xyz_jnts.col(j);
          Vec v1(jnt_xyz_eig);
          Frame d_traj_fm;
          d_traj_fm.setTranslation(v1);
          std::vector<int> ikXYZTurns(XYZ->GetDoF(), 0);
          std::vector<int> branchXYZ(3, 0);
          std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
          XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(), ikXYZTurns.end());
          XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(), UR_turns_orig.end());
          XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(), ikXYZTurns.end());
          XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(), UR_turns_cal.end());


          // desired relative pose w.r.t. block base
          Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
          // modified pose relative to old block base/ actual pose relative calib block base
          Pose mdPose, aPose;
          // ideal uncompensated xyz traj / compensated xyz traj 
          Eigen::VectorXd d_XYZ_j_traj(XYZ->GetDoF());
          Eigen::VectorXd md_XYZ_j_traj(XYZ->GetDoF());

          // step 2: call XYZ->CompSateEachPt (to be done in serialArm.cpp)
          ret = XYZ->ErrCompensationDH(calibBase, origBase,
          Frame(), dPose, &mdPose, &d_XYZ_j_traj,
          &md_XYZ_j_traj, &aPose);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << " XYZ->ErrCompensationDH error  " << ret << ", cart is " << dPose.ToString(true)
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret;  
          }
        

          dPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          dPose.setBranchFlags(UR_flags_orig);
          dPose.setJointTurns(XYZURTurns_orig);
          mdPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          mdPose.setBranchFlags(UR_flags_cal);
          mdPose.setJointTurns(XYZURTurns_cal);
          aPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          aPose.setBranchFlags(UR_flags_orig);
          aPose.setJointTurns(XYZURTurns_orig);
          Eigen::VectorXd absPathPt(5);
          absPathPt.segment(0, xyz_dof) = md_XYZ_j_traj;
          absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
          jnt_out->col(numPtsInPlanes * i + j) = absPathPt;
          
          strs.str("");
          strs << GetName() << " pt ( " <<  i  << "," << j <<")"
              << " desired joint " << absPathPt.transpose() << std::endl
              << " desired cart " << dPose.ToString(true) <<  ", comp cart " << mdPose.ToString(true)
              << ", actual cart " << aPose.ToString(true) << std::endl
              << ", at function " << 
              __FUNCTION__ << ", line" << __LINE__ << std::endl;
          LOG_INFO(strs);
      }
    }
    return 0;
  }

  int  XYZ_UR::GenerateOriginMeasures(const Eigen::MatrixXd &jnt_in,
                                      const int numPtsInPlanes,
                                      Eigen::MatrixXd *jnt_out) {
    std::ostringstream strs;
    if (!initialized_ || !jnt_out) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    size_t numPts = jnt_in.cols();
    if (numPts <= numPtsInPlanes) {
      strs.str("");
      strs << GetName() <<  "Number of Input joints " << numPts << " is less than " << numPtsInPlanes << " points in a plane"
                << ", in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -1030; 
    }
    size_t numRows = jnt_in.rows();
    size_t numPlanes = numPts /  numPtsInPlanes;
    jnt_out->resize(numRows, numPts);
    
    // first of all, using first plane to compute the relative coordinates
    Eigen::MatrixXd  relPt(numRows, numPtsInPlanes);
    size_t xyz_dof = XYZ->GetDoF();
    size_t ur_dof = UR->GetDoF();
    // grab the first 4 ur_jnts and xyz_jnts
    Eigen::MatrixXd ur_jnts = jnt_in.block(xyz_dof, 0, ur_dof, numPtsInPlanes);
    Eigen::VectorXd jnt_ur_eig = ur_jnts.col(0);
    Eigen::MatrixXd xyz_jnts = jnt_in.block(0, 0, xyz_dof, numPtsInPlanes);
    Eigen::VectorXd jnt_xyz_eig = xyz_jnts.col(0);
    // get first ur vector
    std::vector<double> jnt_xyz, jnt_ur;
    EigenVec2StdVec(jnt_ur_eig, &jnt_ur);

    Pose  ps_ur, ps_xyz;
    // set as using calibrated models
    XYZ->SetUsingCalibratedModel(true);
    UR->SetUsingCalibratedModel(true); 
    // compute the first flange frame
    int ret = UR->JntToCart(jnt_ur, &ps_ur);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":" << "UR FK error, code  " << ret << ", input jnt= " << jnt_ur_eig
            << ", in function "  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret; 
    }
    Frame tmp;
    ps_ur.getFrame(&tmp);
    Vec subtcp_vec =tmp.getTranslation();
    Rotation subtcp_rot = tmp.getRotation();

    // now  maps first 4 xyz vectors back to w.r.t. flange frame
    for (size_t i=0; i< numPtsInPlanes; i++) {
      jnt_xyz_eig = xyz_jnts.col(i);
      EigenVec2StdVec(jnt_xyz_eig, &jnt_xyz);
      ret = XYZ->JntToCart(jnt_xyz, &ps_xyz);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "XYZ FK error, code  " << ret
                << "can not do error compensation in, input jnt=" << jnt_xyz_eig 
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      Vec xyz_vec = ps_xyz.getTranslation();
      Vec rel_pt = subtcp_rot.Transpose() * (xyz_vec - subtcp_vec);
      Eigen::VectorXd relPathPt(5), absPathPt(5);
      relPathPt.segment(0, xyz_dof) = rel_pt.ToEigenVec();
      relPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      relPt.col(i) = relPathPt;
      absPathPt.segment(0, xyz_dof) = jnt_xyz_eig;
      absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
      jnt_out->col(i) = absPathPt;
    }

    //  reassign xyz_jnts as relative pts
    xyz_jnts = relPt.block(0, 0, xyz_dof, numPtsInPlanes);
    // then using relative coordinates maps to designated probing points
    for (size_t i=1; i < numPlanes; i++) {
      ur_jnts = jnt_in.block(xyz_dof, i * numPtsInPlanes, ur_dof, numPtsInPlanes);
      //xyz_jnts = jnt_in.block(0, i * numPtsInPlanes, xyz_dof, numPtsInPlanes);
      jnt_ur_eig = ur_jnts.col(0);
      EigenVec2StdVec(jnt_ur_eig, &jnt_ur);
      // using calibrated model to compute the ideal UR pose 
      ret = UR->JntToCart(jnt_ur, &ps_ur);
      if (ret < 0) {
          strs.str("");
          strs << GetName() << " FK error, code  " << ret << ", jnt is " << jnt_ur_eig
              << ", can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
      }
      refPose rf_pose_UR_orig;
      rf_pose_UR_orig.setDefaultPose(ps_ur);
      strs.str("");
      strs << GetName() << ", rf_pose_UR_orig inside URJnt=" << rf_pose_UR_orig.ToString(true) << std::endl;
      LOG_INFO(strs); 

      // ideal UR turns and flags
      std::vector<int> UR_flags_orig, UR_flags_cal;
      std::vector<int> UR_turns_orig, UR_turns_cal;
      rf_pose_UR_orig.getBranchFlags(&UR_flags_orig);
      rf_pose_UR_orig.getJointTurns(&UR_turns_orig);
      rf_pose_UR_orig.getBranchFlags(&UR_flags_cal);
      rf_pose_UR_orig.getJointTurns(&UR_turns_cal);
      
      //Eigen::VectorXd  d_UR_j_traj(UR->GetDoF()), md_UR_j_traj(UR->GetDoF());
      //StdVec2EigenVec(jnt_ur, &d_UR_j_traj); // compensated joint traj
      //StdVec2EigenVec(jnt_ur, &md_UR_j_traj); // compensated joint traj

      
      // now we able able to compute calibBase and origBase
      Frame calibBase, origBase;
      rf_pose_UR_orig.getFrame(&calibBase);
      origBase = calibBase;
      strs.str("");
      strs << GetName() << ", calibBase=" << calibBase.ToString(true) <<
          ", origBase=" << origBase.ToString(true) << ", at function " << 
          __FUNCTION__ << ", line" << __LINE__ << std::endl;
      LOG_INFO(strs);

      // the following is just simply using XYZ API ErrCompsationDH to compensate XYZ traj
      for (size_t j=0; j<numPtsInPlanes; j++) {
          jnt_xyz_eig = xyz_jnts.col(j);
          Vec v1(jnt_xyz_eig);
          Frame d_traj_fm;
          d_traj_fm.setTranslation(v1);
          std::vector<int> ikXYZTurns(XYZ->GetDoF(), 0);
          std::vector<int> branchXYZ(3, 0);
          std::vector<int> XYZURTurns_orig, XYZURTurns_cal;
          XYZURTurns_orig.insert(XYZURTurns_orig.end(), ikXYZTurns.begin(), ikXYZTurns.end());
          XYZURTurns_orig.insert(XYZURTurns_orig.end(), UR_turns_orig.begin(), UR_turns_orig.end());
          XYZURTurns_cal.insert(XYZURTurns_cal.end(), ikXYZTurns.begin(), ikXYZTurns.end());
          XYZURTurns_cal.insert(XYZURTurns_cal.end(), UR_turns_cal.begin(), UR_turns_cal.end());


          // desired relative pose w.r.t. block base
          Pose dPose(d_traj_fm, branchXYZ, ikXYZTurns);
          // modified pose relative to old block base/ actual pose relative calib block base
          Pose mdPose, aPose;
          // ideal uncompensated xyz traj / compensated xyz traj 
          Eigen::VectorXd d_XYZ_j_traj(XYZ->GetDoF());
          Eigen::VectorXd md_XYZ_j_traj(XYZ->GetDoF());

          // step 2: call XYZ->CompSateEachPt (to be done in serialArm.cpp)
          ret = XYZ->ErrCompensationDH(calibBase, origBase,
          Frame(), dPose, &mdPose, &d_XYZ_j_traj,
          &md_XYZ_j_traj, &aPose);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << " XYZ->ErrCompensationDH error  " << ret << ", cart is " << dPose.ToString(true)
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret;  
          }
        

          dPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          dPose.setBranchFlags(UR_flags_orig);
          dPose.setJointTurns(XYZURTurns_orig);
          mdPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          mdPose.setBranchFlags(UR_flags_cal);
          mdPose.setJointTurns(XYZURTurns_cal);
          aPose.setQuaternion(rf_pose_UR_orig.getQuaternion());
          aPose.setBranchFlags(UR_flags_orig);
          aPose.setJointTurns(XYZURTurns_orig);
          Eigen::VectorXd absPathPt(5);
          absPathPt.segment(0, xyz_dof) = md_XYZ_j_traj;
          absPathPt.segment(xyz_dof, ur_dof) = jnt_ur_eig;
          jnt_out->col(numPtsInPlanes * i + j) = absPathPt;
          
          strs.str("");
          strs << GetName() << " pt ( " <<  i  << "," << j <<")"
              << " desired joint " << absPathPt.transpose() << std::endl
              << " desired cart " << dPose.ToString(true) <<  ", comp cart " << mdPose.ToString(true)
              << ", actual cart " << aPose.ToString(true) << std::endl
              << ", at function " << 
              __FUNCTION__ << ", line" << __LINE__ << std::endl;
          LOG_INFO(strs);
      }
    }
    return 0;
  }

  double XYZ_UR::LaserCalibrateOrientation(
       const Eigen::MatrixXd &jnt_measure,
       const Eigen::MatrixXd &cart_measure,
       const Eigen::VectorXd &laserMat_z_measure,
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser dist, under which, laser reading is 0
       const int numPtsInEachPlane,
       const std::vector<int> surfaceArrays) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    // now processing  cart_measure and laserMat_Z_measure to generate cart sequences in planes
    size_t num_row_jnt = jnt_measure.rows();
    size_t num_col_jnt = jnt_measure.cols();

    size_t num_row_cart = cart_measure.rows();
    size_t num_col_cart = cart_measure.cols();

    size_t num_col_laser = laserMat_z_measure.size();

    
    // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
    bool sizeOK = ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) && (numPtsInEachPlane >= 3) && (laser_channel >=0 && laser_channel <3));
    if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 || (num_col_jnt % numPtsInEachPlane != 0)) {
      strs.str("");
      strs << GetName() << ":" << "CalibrateLaserOrientation: input data dimension is not matching"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ 
                << "num_col_cart=" << num_col_cart << ", num_col_jnt=" << num_col_jnt  << ", num_col_laser=" <<
                num_col_laser << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt << 
                ", value of modular=" << num_col_jnt % numPtsInEachPlane << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    size_t numPlanes = num_col_jnt / numPtsInEachPlane;
    Eigen::MatrixXd UR_jnt_measure(UR->GetDoF(), numPlanes);
    Eigen::MatrixXd UR_cart_measure(num_row_cart, numPtsInEachPlane * numPlanes);

    double laser0 = laser_value;
    // std::vector<Eigen::Vector3d>  ni(numPlanes);
    // std::vector<double>   bi(numPlanes, 0);
    for (size_t i=0; i < numPlanes; i++) {
      Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachPlane, UR->GetDoF(), numPtsInEachPlane);
      Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachPlane, num_row_cart, numPtsInEachPlane);
      Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachPlane, numPtsInEachPlane);
      // first check if all jnts are same
      for (size_t j=1; j < numPtsInEachPlane; j++) {
        Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
        if (diff_jnt.norm() > K_EPSILON) {
          strs.str("");
          strs << GetName() << ":" << "CalibrateLaserOrientation: UR joints in each plane is not same"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return -2001; 
        }
      }

      // now modify the carts to make sure they are in the same plane
      UR_jnt_measure.col(i) = jnts.col(0);
      for (size_t j=0; j < numPtsInEachPlane; j++) {
        Eigen::VectorXd cart = carts.col(j);
        cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - laser0);
        Eigen::VectorXd tmpJnt, tmpCart;
        XYZ->SetUsingCalibratedModel(false);
        if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
          strs.str("");
          strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
               << " at line "  << __LINE__ << std::endl; 
          LOG_ERROR(strs);
          return -2002;
        }
        XYZ->SetUsingCalibratedModel(true);
        if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
          strs.str("");
          strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
               << " at line "  << __LINE__ << std::endl; 
          LOG_ERROR(strs);
          return -2002;
        }
        UR_cart_measure.col(i * numPtsInEachPlane + j) = cart;
      }
    }

    XYZ->SetUsingCalibratedModel(false); // reset into uncalibrated mode
    double ret = UR->LaserCalibrateOrientation(UR_jnt_measure, UR_cart_measure, laserMat_z_measure, 
                 laser_channel, laser_scale, laser_value, max_laser_dist,  numPtsInEachPlane, surfaceArrays);
    if (ret >= 0) { // means UR has been calibrated
      if (XYZ->isCalibrated()) {
        isDHCalibrated_ = true;  // set entire mechanisms as calibrated
      }
    }
    return ret;
  }

  double XYZ_UR::CalibrateLaserOrigin(
       // jnt measure matrix  j1 / j2 /j3 /j4 /j5
       const EigenDRef<Eigen::MatrixXd> &jnt_measure,
       // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
       const EigenDRef<Eigen::MatrixXd> &cart_measure,
       // z laser values
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,
       const int laser_channel,
       const double laser_scale,  // laser scale for the above laser channel
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser distance, under which, laser reading is 0
       // 4 in a plane in each direction or 8 for a complete plane, then totally 8 * 3, 24 pts
       const int numPtsInEachOrientPlane) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      // now processing  cart_measure and laserMat_Z_measure to generate cart sequences in planes
      size_t num_row_jnt = jnt_measure.rows();
      size_t num_col_jnt = jnt_measure.cols();

      size_t num_row_cart = cart_measure.rows();
      size_t num_col_cart = cart_measure.cols();

      size_t num_col_laser = laserMat_z_measure.size();

      
      // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
      bool sizeOK = ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) && (numPtsInEachOrientPlane >= 4) && (laser_channel >=0 && laser_channel <3));
      if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 || (num_col_jnt % numPtsInEachOrientPlane != 0)) {
        strs.str("");
        strs << GetName() << ":" << "CalibrateLaserOrigin: input data dimension is not matching"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ 
                  << "num_col_cart=" << num_col_cart << ", num_col_jnt=" << num_col_jnt  << ", num_col_laser=" <<
                  num_col_laser << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt << 
                  ", value of modular=" << num_col_jnt % (2 * numPtsInEachOrientPlane) << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      }
      size_t numPlanes = num_col_jnt / numPtsInEachOrientPlane;
      if (numPlanes < 5) {
        strs.str("");
        strs << GetName() << ":" << "CalibrateLaserOrigin: there are only " << numPlanes << " planes in origin measuring, in " 
                  << __FUNCTION__ << ", line " << __LINE__  << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
      }

      // try to make sure all points have same z laser reading values
      double max_laser =  laser_value; // laserMat_z_measure.maxCoeff();  // need to be changed into configurable value
      double laser_length =  max_laser_dist - laser_value;
      strs.str("");
      strs << GetName() << ": max laser=" << max_laser << ", laser scale= " << laser_scale << 
             ", laser_length=" << laser_length << std::endl;
      LOG_INFO(strs);

      // data for compute o, a1, vd (new algorithm)
      Eigen::MatrixXd MM(numPlanes, 5);
      Eigen::VectorXd bb(numPlanes);

      
      Eigen::VectorXd  UR_jnt;
      UR->SetUsingCalibratedModel(true);
      Eigen::Vector3d dv1, dv2, cN, cN1, cN2, cN3;
      std::vector<Eigen::Vector3d> pp(numPtsInEachOrientPlane);
      for (size_t i=0; i < numPlanes; i++) {
        Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachOrientPlane, 
                                                   num_row_cart, numPtsInEachOrientPlane);
        Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachOrientPlane, 
                                                          numPtsInEachOrientPlane);
        Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachOrientPlane, 
                                              UR->GetDoF(), numPtsInEachOrientPlane);
        Eigen::MatrixXd  jntxyz = jnt_measure.block(0, i * numPtsInEachOrientPlane, 
                                              XYZ->GetDoF(), numPtsInEachOrientPlane);
        UR_jnt = jnts.rowwise().mean(); //jnts.col(0);
        // first check if all jnts are same for half points in the plane
        for (size_t j=1; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
          if (diff_jnt.norm() > K_EPSILON) {
            strs.str("");
            strs << GetName() << ":" << "CalibrateLaserOrigin: UR joints in each plane is not same, epsilon="
                  << diff_jnt.norm() << ", which is greater than " << K_EPSILON
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            //return -2001; 
          }
        }
        
        // modify the cart so that they are in the same with same z value
        // compute the first corner of plane 1
          
        std::vector<Vec> p(numPtsInEachOrientPlane);
        for (size_t j=0; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd cart = carts.col(j);
          cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - max_laser - laser_length);  // make sure laser light length = 0
          strs.str("");
          Eigen::VectorXd tmpJnt;
          XYZ->SetUsingCalibratedModel(false);
          if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
            strs.str("");
            strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          strs << "xyzjnt=" << jntxyz.col(j) << ", original cart="  << cart.segment(0, 3) << ", tmpJnt=" << tmpJnt << std::endl;
          XYZ->SetUsingCalibratedModel(true);
          if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
            strs.str("");
            strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          p[j] = Vec(cart.segment(0, 3));
          pp[j] = p[j].ToEigenVec();
          strs << " pj =" << p[j].ToString() << ", pp[j]=" << pp[j] << std::endl;
          LOG_INFO(strs);
        }
       
        dv1 = pp[1] - pp[0];
        dv2 = pp[2] - pp[1];
        cN = dv1.cross(dv2);
        double norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        if (cN(laser_channel) < 0) { // if cN is not facing same direction as positive Z, then reverse cN
          cN = - cN;
        }
        strs.str("");
        strs << "dv1=" << dv1 << ", dv2=" << dv2 << ",cN=" << cN << ", norm_nd=" << norm_nd << std::endl;
        LOG_INFO(strs);

        dv1 = pp[3] - pp[2];
        dv2 = pp[0] - pp[3];
        cN1 = dv1.cross(dv2);
        norm_nd = cN1.norm();
        cN1 /= norm_nd;
        if (cN1(laser_channel) < 0) {
          cN1 =-cN1;
        }
        
        dv1 = pp[2] - pp[1];
        dv2 = pp[3] - pp[2];
        cN2 = dv1.cross(dv2);
        norm_nd = cN2.norm();
        cN2 /= norm_nd;
        if (cN2(laser_channel) < 0) {
          cN2 =-cN2;
        }

        dv1 = pp[0] - pp[3];
        dv2 = pp[1] - pp[0];
        cN3 = dv1.cross(dv2);
        norm_nd = cN3.norm();
        cN3 /= norm_nd;
        if (cN3(laser_channel) < 0) {
          cN3 =-cN3;
        }
        
        strs.str("");
        strs << "second normal: dv1=" << dv1 << ", dv2=" << dv2 << ",cN1=" << cN1 << ", cN2=" << cN2 << ", cN3=" << cN3 << std::endl;
        // now doing average of the two normal
        cN = (cN + cN1 + cN2 + cN3) / 4.0;
        norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        strs << "average cN=" << cN << "average norm=" << norm_nd << std::endl;
        LOG_INFO(strs);

        Frame fm2;
        std::vector<double> ur_jnt_vec;
        EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
        if (!UR->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
              strs.str("");
              strs << GetName() << "UR->GetDHFrame error with jnt=" << UR_jnt << ", at index 1"
                  << ", in function" << __FUNCTION__
                  << " at line "  << __LINE__ << std::endl; 
              LOG_ERROR(strs);
              return -2002;
        }
        Rotation r1 = fm2.getRotation();
        Vec x1 = r1.UnitX();
        Eigen::VectorXd eigx1 =  x1.ToEigenVec();
        
        Eigen::VectorXd  mcol(5);
        mcol.segment(0, 3) =  cN;
        mcol(3) = cN.dot(eigx1);
        mcol(4) = 1.0;

        MM.row(i) = mcol.transpose();
        Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
        // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out because pp[i] has already been set to laser light length= 0
        // note above trying to consider laser tcp with laser light length=0    ****** very important *****
        // then 8pt user frame, also adjust pt coordinate so that laser light length = 0
        // then in traj compensation, we set tcp(2) = - |laser_scale| * (35-laser_reading) in user tool setting
        bb(i) = cN.dot(meanPt);

        strs.str("");
        strs << "i row of MM = " << mcol.transpose() << ", eigx1=" << eigx1 << ", meanPt=" << meanPt 
        << ", bb(" << i << ")=" << bb(i) << ",meanpt off z=" << laser_length * (1 - cN(2)) / cN(2) <<  std::endl;
        LOG_INFO(strs);
      }

      // 3rd method
      Eigen::MatrixXd MMTMM = MM.transpose() * MM;
      strs.str("");
      strs << "MM = " << MM << ", MMTMM=" << MMTMM <<  ", bb=" << bb.transpose() << std::endl;
      LOG_INFO(strs);
      double det = MMTMM.determinant();
      if (det < CALIB_SINGULAR_CONST) {
         strs.str("");
         strs << GetName() << "determinant of MMTMM (method 4)=" << det  << " is singular , in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
         LOG_ERROR(strs);
         return -2002;
      }

      // now using 3 rotation matrix, and 12 corner points, compute the center of rotation
      Eigen::VectorXd origin_f3 = MMTMM.inverse() * MM.transpose() * bb;
      
      // we will reuse the x of this origin
      Eigen::Vector3d origin = origin_f3.segment(0, 3);
      //double d1 = origin_f3(4);
      double a1 = origin_f3(3);
      double v1 = origin_f3(4);
      //double ox = origin(0); // from here we get x coordinates

      // now from circ_data, compute the center point
      strs.str("");
      strs << GetName() << " final origin_f3 from transform method 4=" << origin_f3 << std::endl;
      Eigen::VectorXd errorOrig = MM * origin_f3 - bb;
      strs << "errorOrigf3 =" << errorOrig << ", norm=" << errorOrig.norm() << std::endl;
      strs <<"a1=" << a1  << ", v1=" << v1 << std::endl;
      LOG_INFO(strs);
      
      Eigen::MatrixXd planeCenter(5, numPlanes);
      Eigen::MatrixXd planeNormal(3, numPlanes);
      // now verify the results
      for (size_t i=0; i < numPlanes; i++) {
        Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachOrientPlane, 
                                                   num_row_cart, numPtsInEachOrientPlane);
        Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachOrientPlane, 
                                                          numPtsInEachOrientPlane);
        Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachOrientPlane, 
                                              UR->GetDoF(), numPtsInEachOrientPlane);
        UR_jnt = jnts.rowwise().mean();  //jnts.col(0);
        // modify the cart so that they are in the same with same z value
        // compute the first corner of plane 1
          
        std::vector<Vec> p(numPtsInEachOrientPlane);
        for (size_t j=0; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd cart = carts.col(j);
          cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - max_laser - laser_length);  // make sure laser light length = 0

          Eigen::VectorXd tmpJnt;
          XYZ->SetUsingCalibratedModel(false);
          if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
            strs.str("");
            strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          XYZ->SetUsingCalibratedModel(true);
          if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
            strs.str("");
            strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          p[j] = Vec(cart.segment(0, 3));
          pp[j] = p[j].ToEigenVec();
        }
       
        dv1 = pp[1] - pp[0];
        dv2 = pp[2] - pp[1];
        cN = dv1.cross(dv2);
        double norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        if (cN(laser_channel) < 0) { // if cN is not facing same direction as positive Z, then reverse cN
          cN = - cN;
        }

        dv1 = pp[3] - pp[2];
        dv2 = pp[0] - pp[3];
        cN1 = dv1.cross(dv2);
        norm_nd = cN1.norm();
        cN1 /= norm_nd;
        if (cN1(laser_channel) < 0) {
          cN1 =-cN1;
        }
        
        dv1 = pp[2] - pp[1];
        dv2 = pp[3] - pp[2];
        cN2 = dv1.cross(dv2);
        norm_nd = cN2.norm();
        cN2 /= norm_nd;
        if (cN2(laser_channel) < 0) {
          cN2 =-cN2;
        }

        dv1 = pp[0] - pp[3];
        dv2 = pp[1] - pp[0];
        cN3 = dv1.cross(dv2);
        norm_nd = cN3.norm();
        cN3 /= norm_nd;
        if (cN3(laser_channel) < 0) {
          cN3 =-cN3;
        }

        // now doing average of the two normal
        cN = (cN + cN1 + cN2 + cN3) / 4.0;
        norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        planeNormal.col(i) = cN;

        Frame fm2;
        std::vector<double> ur_jnt_vec;
        EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
        if (!UR->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
              strs.str("");
              strs << GetName() << "UR->GetDHFrame error with jnt=" << UR_jnt << ", at index 1"
                  << ", in function" << __FUNCTION__
                  << " at line "  << __LINE__ << std::endl; 
              LOG_ERROR(strs);
              return -2002;
        }
        Rotation r1 = fm2.getRotation();
        Vec x1 = r1.UnitX();
        Eigen::VectorXd eigx1 =  x1.ToEigenVec();
            
        Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
        // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out because pp[i] has already been set to laser light length= 0
        // note above trying to consider laser tcp with laser light length=0    ****** very important *****
        // then 8pt user frame, also adjust pt coordinate so that laser light length = 0
        // then in traj compensation, we set tcp(2) = - |laser_scale| * (35-laser_reading) in user tool setting

        Eigen::Vector3d  pred_pt = origin + a1 * eigx1; //  + cN * v1;

        strs.str("");
        strs << "plane " << i << ", eigx1=" << eigx1 <<  ", cN=" << cN << ", meanPt=" << meanPt 
        << ", pred_pt =" << pred_pt << ", actual-pt=" << meanPt << ", error=" << meanPt - pred_pt 
        << ", error_proj=" << cN.dot(meanPt-pred_pt) - v1 <<   std::endl;
        
        LOG_INFO(strs);
      
        // now put center data into matrix, and then save into csv file
        Eigen::VectorXd  cT(5);
        cT.segment(0, 3) = pred_pt + cN * v1;
        cT.segment(3, 2) = UR_jnt;
        planeCenter.col(i) = cT;
      }
   
      // now writing planceCenter to csv file
      RPE_Utility util;
      std::string centerFile("planeCenter.txt");
      util.WriteCSVFile(centerFile, planeCenter);
      std::string normalFile("planeNormal.txt");
      util.WriteCSVFile(normalFile, planeNormal);

      // now compute U axis direction, for now just log it
      size_t numGroups = numPlanes / 4; // each group contains 4 planes
      Eigen::MatrixXd  ptGroup(3, 4), ptGroup2(3, 4);
      Eigen::MatrixXd  uAxis(3, 4), uAxis2(3, 4);
      for (size_t j=0; j< numGroups; j++) {
        for (size_t i=0; i < 4; i++) {
          ptGroup.col(i) = planeCenter.col(4*i + j).segment(0, 3);
          ptGroup2.col(i) = planeNormal.col(4*i + j).segment(0, 3);
        }
        
        // now computing the unit norm of ptGroup
        Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
        Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXd tmpV = svd2.matrixV();
        Eigen::Vector3d pnz = tmpV.col(2);
        strs.str("");
        strs << "U group" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        uAxis.col(j) = pnz;

        meanPt = ptGroup2.rowwise().mean();
        tmp = ptGroup2.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd22(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd22.matrixV();
        pnz = tmpV.col(2);
        strs.str("");
        strs << "U group using normal" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        uAxis2.col(j) = pnz;
      }
      std::string uAxisFile("uAxis.txt");
      util.WriteCSVFile(uAxisFile, uAxis);
      Eigen::Vector3d uFinalAxis =  uAxis.rowwise().mean();
      uFinalAxis /= uFinalAxis.norm();

      std::string uAxisFile2("uAxis2.txt");
      util.WriteCSVFile(uAxisFile2, uAxis2);

      Eigen::Vector3d uFinalAxis2 = uAxis2.rowwise().mean();
      uFinalAxis2 /= uFinalAxis2.norm();

      strs.str("");
      strs << "uFinalAxis=" << uFinalAxis.transpose() << std::endl;
      strs << "uFinalAxis2=" << uFinalAxis2.transpose() << std::endl;
      LOG_INFO(strs);

      // now compute the zero of U axis
      // first compute the direction of all rAxis with given fixed Us
      Eigen::MatrixXd  rAxis(3, 4), rAxis2(3, 4);
      Eigen::VectorXd angleRU(4), angleRU2(4); // angles between U and R
      for (size_t j=0; j < numGroups; j++) {
        for (size_t i=0; i < 4; i++) {
          ptGroup.col(i) = planeCenter.col(4 * j + i).segment(0, 3);
          ptGroup2.col(i) = planeNormal.col(4 * j + i).segment(0, 3);
        }
        
        // now computing the unit norm of ptGroup
        Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
        Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXd tmpV = svd2.matrixV();
        Eigen::Vector3d pnz = tmpV.col(2);
        strs.str("");
        strs << "R group" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        rAxis.col(j) = pnz;
        angleRU(j) = acos(pnz.dot(uFinalAxis));

        meanPt = ptGroup2.rowwise().mean();
        tmp = ptGroup2.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd22(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd22.matrixV();
        pnz = tmpV.col(2);
        strs.str("");
        strs << "R group normal" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        rAxis2.col(j) = pnz;
        angleRU2(j) = acos(pnz.dot(uFinalAxis2));
      }
      std::string rAxisFile("rAxis.txt");
      util.WriteCSVFile(rAxisFile, rAxis);

      std::string rAxisFile2("rAxis2.txt");
      util.WriteCSVFile(rAxisFile2, rAxis2);

      std::string angleRUFile("angleRU.txt");
      util.WriteCSVFile(angleRUFile, angleRU);

      std::string angleRUFile2("angleRU2.txt");
      util.WriteCSVFile(angleRUFile2, angleRU2);

      Eigen::VectorXd urDH;
      UR->GetCalibParamSet(&urDH);
      urDH(3) = a1;  // alpha0, alpha1, a0, a1, t0, t1, d0, d1, b0, b1
      //urDH(7) = d1;
      std::vector<double> urDHVec;
      EigenVec2StdVec(urDH, &urDHVec);
      UR->LoadCalibParamSet(urDHVec);  // update calibrated DH parameter 

      Frame tmp_baseoff, tmp_subbaseoff;
      UR->GetDefaultBaseOffFrame(&tmp_baseoff, &tmp_subbaseoff);
      
      
      // now UR mechanism set translational part of baseOffset
      // Frame tmpBase = sub_defaultBaseOff_;
      strs.str("");
      strs << GetName() << ": UR orig defaultSubBaseOff=" <<  tmp_baseoff.ToString(true) << std::endl;
      LOG_INFO(strs);
      Vec t(origin);
      tmp_baseoff.setTranslation(t);
      Eigen::VectorXd tmpBaseVec = tmp_baseoff.ToEigenVecQuat();
      strs.str("");
      strs << GetName() << ": UR baseoff is changed into " << tmp_baseoff.ToString(true)
      << ", in eigen vector=" << tmpBaseVec.transpose() << std::endl;
      LOG_INFO(strs);
      UR->SetDefaultBaseOff(tmpBaseVec, tmpBaseVec);  // modify the default base of UR obj
      
      //Vec t1(tmp_baseoff.segment(0,3));
      //Quaternion q1(tmp_baseoff(3), tmp_baseoff(4), tmp_baseoff(5), tmp_baseoff(6));
      sub_defaultBaseOff_ = tmp_baseoff; // modify sub base off of XYZUR too
      //sub_defaultBaseOff_.setQuaternion(q1);
      strs.str("");
      strs << GetName() << ": modify subdefulatBaseoff into" << sub_defaultBaseOff_.ToString(true) 
      <<", while defualtbaseoff is still " << defaultBaseOff_.ToString(true) << std::endl;
      LOG_INFO(strs);
      
      UR->updateCalibrateStatus(true);
      if (XYZ->isCalibrated()) {
        isDHCalibrated_ = true;  // set entire mechanism is calibrated
      }

      //sub_defaultBaseOff_ = tmpBase;  // modify the default subBase of XYZ_UR obj, defaultBase for XYZ is not changed
      return 0;
    } catch(...) {
       strs.str("");
       strs << GetName() << " gets exception in function " << __FUNCTION__ << std::endl;
       LOG_ERROR(strs);
       return -1;
    }
  }

  double XYZ_UR::LaserCalibrateOrigin(
       // jnt measure matrix  j1 / j2 /j3 /j4 /j5
       const Eigen::MatrixXd &jnt_measure,
       // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
       const Eigen::MatrixXd &cart_measure,
       // z laser values
       const Eigen::VectorXd &laserMat_z_measure,
       const int laser_channel,  // using which channel of lasr (x or y or z) for measuring
       const double laser_scale,  // laser scale for the above laser channel
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser dist, under which, laser reading is 0
       // 4 in a plane in each direction or 8 for a complete plane, then totally 8 * 3, 24 pts
       const int numPtsInEachOrientPlane) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      // now processing  cart_measure and laserMat_Z_measure to generate cart sequences in planes
      size_t num_row_jnt = jnt_measure.rows();
      size_t num_col_jnt = jnt_measure.cols();

      size_t num_row_cart = cart_measure.rows();
      size_t num_col_cart = cart_measure.cols();

      size_t num_col_laser = laserMat_z_measure.size();

      
      // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
      bool sizeOK = ((num_col_cart == num_col_jnt) && (num_col_cart == num_col_laser) && (numPtsInEachOrientPlane >= 4) && (laser_channel >=0 && laser_channel <3));
      if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 16 || (num_col_jnt % numPtsInEachOrientPlane != 0)) {
        strs.str("");
        strs << GetName() << ":" << "CalibrateLaserOrigin: input data dimension is not matching"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ 
                  << "num_col_cart=" << num_col_cart << ", num_col_jnt=" << num_col_jnt  << ", num_col_laser=" <<
                  num_col_laser << ", num_row_cart=" << num_row_cart << ", num_row_jnt=" << num_row_jnt << 
                  ", value of modular=" << num_col_jnt % (2 * numPtsInEachOrientPlane) << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      }
      size_t numPlanes = num_col_jnt / numPtsInEachOrientPlane;
      if (numPlanes < 5) {
        strs.str("");
        strs << GetName() << ":" << "CalibrateLaserOrigin: there are only " << numPlanes << " planes in origin measuring, in " 
                  << __FUNCTION__ << ", line " << __LINE__  << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
      }

      // try to make sure all points have same z laser reading values
      double max_laser =  laser_value; // laserMat_z_measure.maxCoeff();  // need to be changed into configurable value
      double laser_length =  max_laser_dist - laser_value;
      strs.str("");
      strs << GetName() << ": max laser=" << max_laser << ", laser scale= " << laser_scale << 
                        ", laser_length=" << laser_length << std::endl;
      LOG_INFO(strs);

      // data for compute o, a1, vd (new algorithm)
      Eigen::MatrixXd MM(numPlanes, 5);
      Eigen::VectorXd bb(numPlanes);

      
      Eigen::VectorXd  UR_jnt;
      UR->SetUsingCalibratedModel(true);
      Eigen::Vector3d dv1, dv2, cN, cN1, cN2, cN3;
      std::vector<Eigen::Vector3d> pp(numPtsInEachOrientPlane);
      for (size_t i=0; i < numPlanes; i++) {
        Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachOrientPlane, 
                                                   num_row_cart, numPtsInEachOrientPlane);
        Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachOrientPlane, 
                                                          numPtsInEachOrientPlane);
        Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachOrientPlane, 
                                              UR->GetDoF(), numPtsInEachOrientPlane);
        Eigen::MatrixXd  jntxyz = jnt_measure.block(0, i * numPtsInEachOrientPlane, 
                                              XYZ->GetDoF(), numPtsInEachOrientPlane);
        UR_jnt = jnts.rowwise().mean(); // UR_jnt = jnts.col(0);
        // first check if all jnts are same for half points in the plane
        for (size_t j=1; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd diff_jnt = jnts.col(j) - jnts.col(0);
          if (diff_jnt.norm() > K_EPSILON) {
            strs.str("");
            strs << GetName() << ":" << "CalibrateLaserOrigin: UR joints in each plane is not same, epsilon="
                  << diff_jnt.norm() << ", which is greater than " << K_EPSILON
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            // return -2001; 
          }
        }
        
        // modify the cart so that they are in the same with same z value
        // compute the first corner of plane 1
          
        std::vector<Vec> p(numPtsInEachOrientPlane);
        for (size_t j=0; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd cart = carts.col(j);
          cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - max_laser - laser_length);  // make sure laser light length = 0
          strs.str("");
          Eigen::VectorXd tmpJnt;
          XYZ->SetUsingCalibratedModel(false);
          if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
            strs.str("");
            strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          strs << "xyzjnt=" << jntxyz.col(j) << ", original cart="  << cart.segment(0, 3) << ", tmpJnt=" << tmpJnt << std::endl;
          XYZ->SetUsingCalibratedModel(true);
          if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
            strs.str("");
            strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          p[j] = Vec(cart.segment(0, 3));
          pp[j] = p[j].ToEigenVec();

          strs << " pj =" << p[j].ToString() << ", pp[j]=" << pp[j] << std::endl;
          LOG_INFO(strs);
        }
       
        dv1 = pp[1] - pp[0];
        dv2 = pp[2] - pp[1];
        cN = dv1.cross(dv2);
        double norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        if (cN(laser_channel) < 0) { // if cN is not facing same direction as positive Z, then reverse cN
          cN = - cN;
        }
        strs.str("");
        strs << "dv1=" << dv1 << ", dv2=" << dv2 << ",cN=" << cN << ", norm_nd=" << norm_nd << std::endl;
        LOG_INFO(strs);

        dv1 = pp[3] - pp[2];
        dv2 = pp[0] - pp[3];
        cN1 = dv1.cross(dv2);
        norm_nd = cN1.norm();
        cN1 /= norm_nd;
        if (cN1(laser_channel) < 0) {
          cN1 =-cN1;
        }
        
        dv1 = pp[2] - pp[1];
        dv2 = pp[3] - pp[2];
        cN2 = dv1.cross(dv2);
        norm_nd = cN2.norm();
        cN2 /= norm_nd;
        if (cN2(laser_channel) < 0) {
          cN2 =-cN2;
        }

        dv1 = pp[0] - pp[3];
        dv2 = pp[1] - pp[0];
        cN3 = dv1.cross(dv2);
        norm_nd = cN3.norm();
        cN3 /= norm_nd;
        if (cN3(laser_channel) < 0) {
          cN3 =-cN3;
        }
        
        strs.str("");
        strs << "second normal: dv1=" << dv1 << ", dv2=" << dv2 << ",cN1=" << cN1 << ", cN2=" << cN2 << ", cN3=" << cN3 << std::endl;
        // now doing average of the two normal
        cN = (cN + cN1 + cN2 + cN3) / 4.0;
        norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        strs << "average cN=" << cN << "average norm=" << norm_nd << std::endl;
        LOG_INFO(strs);

        Frame fm2;
        std::vector<double> ur_jnt_vec;
        EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
        if (!UR->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
              strs.str("");
              strs << GetName() << "UR->GetDHFrame error with jnt=" << UR_jnt << ", at index 1"
                  << ", in function" << __FUNCTION__
                  << " at line "  << __LINE__ << std::endl; 
              LOG_ERROR(strs);
              return -2002;
        }
        Rotation r1 = fm2.getRotation();
        Vec x1 = r1.UnitX();
        Eigen::VectorXd eigx1 =  x1.ToEigenVec();
        
        Eigen::VectorXd  mcol(5);
        mcol.segment(0, 3) =  cN;
        mcol(3) = cN.dot(eigx1);
        mcol(4) = 1.0;

        MM.row(i) = mcol.transpose();
        Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
        // meanPt(2) -= fabs(laser_scale) * laser_length;  this is commented out because pp[i] has already been set to laser light length= 0
        // note above trying to consider laser tcp with laser light length=0    ****** very important *****
        // then 8pt user frame, also adjust pt coordinate so that laser light length = 0
        // then in traj compensation, we set tcp(2) = - |laser_scale| * (35-laser_reading) in user tool setting
        bb(i) = cN.dot(meanPt);

        strs.str("");
        strs << "i row of MM = " << mcol.transpose() << ", eigx1=" << eigx1 << ", meanPt=" << meanPt 
        << ", bb(" << i << ")=" << bb(i) << ",meanpt off z=" << laser_length * (1 - cN(2)) / cN(2) <<  std::endl;
        LOG_INFO(strs);
      }

      // 3rd method
      Eigen::MatrixXd MMTMM = MM.transpose() * MM;
      strs.str("");
      strs << "MM = " << MM << ", MMTMM=" << MMTMM <<  ", bb=" << bb.transpose() << std::endl;
      LOG_INFO(strs);
      double det = MMTMM.determinant();
      if (det < CALIB_SINGULAR_CONST) {
         strs.str("");
         strs << GetName() << "determinant of MMTMM (method 4)=" << det  << " is singular , in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
         LOG_ERROR(strs);
         return -2002;
      }

      // now using 3 rotation matrix, and 12 corner points, compute the center of rotation
      Eigen::VectorXd origin_f3 = MMTMM.inverse() * MM.transpose() * bb;
      
      // we will reuse the x of this origin
      Eigen::Vector3d origin = origin_f3.segment(0, 3);
      //double d1 = origin_f3(4);
      double a1 = origin_f3(3);
      double v1 = origin_f3(4);
      //double ox = origin(0); // from here we get x coordinates

      // now from circ_data, compute the center point
      strs.str("");
      strs << GetName() << " final origin_f3 from transform method 4=" << origin_f3 << std::endl;
      Eigen::VectorXd errorOrig = MM * origin_f3 - bb;
      strs << "errorOrigf3 =" << errorOrig << ", norm=" << errorOrig.norm() << std::endl;
      strs <<"a1=" << a1  << ", v1=" << v1 << std::endl;
      LOG_INFO(strs);
      
      
      // now verify the results
      Eigen::MatrixXd planeCenter(5, numPlanes);
      Eigen::MatrixXd  planeNormal(3, numPlanes);
      for (size_t i=0; i < numPlanes; i++) {
        Eigen::MatrixXd carts = cart_measure.block(0, i * numPtsInEachOrientPlane, 
                                                   num_row_cart, numPtsInEachOrientPlane);
        Eigen::MatrixXd laser = laserMat_z_measure.segment(i * numPtsInEachOrientPlane, 
                                                          numPtsInEachOrientPlane);
        Eigen::MatrixXd jnts = jnt_measure.block(XYZ->GetDoF(), i * numPtsInEachOrientPlane, 
                                              UR->GetDoF(), numPtsInEachOrientPlane);
        UR_jnt = jnts.rowwise().mean(); // UR_jnt = jnts.col(0);
        // modify the cart so that they are in the same with same z value
        // compute the first corner of plane 1
          
        std::vector<Vec> p(numPtsInEachOrientPlane);
        for (size_t j=0; j < numPtsInEachOrientPlane; j++) {
          Eigen::VectorXd cart = carts.col(j);
          cart(laser_channel) = cart(laser_channel) - laser_scale * (laser(j) - max_laser - laser_length);   // make sure laser light length = 0

          Eigen::VectorXd tmpJnt;
          XYZ->SetUsingCalibratedModel(false);
          if (!XYZ->GetJntFromPose(cart, &tmpJnt)) {
            strs.str("");
            strs << "XYZ->GetJntFromPose error with cart=" << cart << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          XYZ->SetUsingCalibratedModel(true);
          if (!XYZ->GetPoseFromJnt(tmpJnt, &cart)) {
            strs.str("");
            strs << "XYZ->GetPoseFromJnt error with jnt=" << tmpJnt << ", in function" << __FUNCTION__
                << " at line "  << __LINE__ << std::endl; 
            LOG_ERROR(strs);
            return -2002;
          }
          p[j] = Vec(cart.segment(0, 3));
          pp[j] = p[j].ToEigenVec();
        }
       
        dv1 = pp[1] - pp[0];
        dv2 = pp[2] - pp[1];
        cN = dv1.cross(dv2);
        double norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        if (cN(laser_channel) < 0) { // if cN is not facing same direction as positive Z, then reverse cN
          cN = - cN;
        }

        dv1 = pp[3] - pp[2];
        dv2 = pp[0] - pp[3];
        cN1 = dv1.cross(dv2);
        norm_nd = cN1.norm();
        cN1 /= norm_nd;
        if (cN1(laser_channel) < 0) {
          cN1 =-cN1;
        }
        
        dv1 = pp[2] - pp[1];
        dv2 = pp[3] - pp[2];
        cN2 = dv1.cross(dv2);
        norm_nd = cN2.norm();
        cN2 /= norm_nd;
        if (cN2(laser_channel) < 0) {
          cN2 =-cN2;
        }

        dv1 = pp[0] - pp[3];
        dv2 = pp[1] - pp[0];
        cN3 = dv1.cross(dv2);
        norm_nd = cN3.norm();
        cN3 /= norm_nd;
        if (cN3(laser_channel) < 0) {
          cN3 =-cN3;
        }

        // now doing average of the two normal
        cN = (cN + cN1 + cN2 + cN3) / 4.0;
        norm_nd = cN.norm();
        cN /= norm_nd;   // measured normal
        planeNormal.col(i) = cN;


        Frame fm2;
        std::vector<double> ur_jnt_vec;
        EigenVec2StdVec(UR_jnt, &ur_jnt_vec);
        if (!UR->GetDHFrame(ur_jnt_vec, 0, &fm2)) {
              strs.str("");
              strs << GetName() << "UR->GetDHFrame error with jnt=" << UR_jnt << ", at index 1"
                  << ", in function" << __FUNCTION__
                  << " at line "  << __LINE__ << std::endl; 
              LOG_ERROR(strs);
              return -2002;
        }
        Rotation r1 = fm2.getRotation();
        Vec x1 = r1.UnitX();
        Eigen::VectorXd eigx1 =  x1.ToEigenVec();
            
        Eigen::Vector3d meanPt = (pp[0] + pp[1] + pp[2] + pp[3]) / 4.0;
        // meanPt(2)  -= fabs(laser_scale) * laser_length;   this is commented out because pp[i] has already been set to laser light length= 0
        // note above trying to consider laser tcp with laser light length=0    ****** very important *****
        // then 8pt user frame, also adjust pt coordinate so that laser light length = 0
        // then in traj compensation, we set tcp(2) = - |laser_scale| * (35-laser_reading) in user tool setting

        Eigen::Vector3d  pred_pt = origin + a1 * eigx1; //  + cN * v1;

        strs.str("");
        strs << "plane " << i << ", eigx1=" << eigx1 <<  ", cN=" << cN << ", meanPt=" << meanPt 
        << ", pred_pt =" << pred_pt << ", actual-pt=" << meanPt << ", error=" << meanPt - pred_pt 
        << ", error_proj=" << cN.dot(meanPt-pred_pt) - v1 <<   std::endl;
        
        LOG_INFO(strs);
         // now put center data into matrix, and then save into csv file
        Eigen::VectorXd  cT(5);
        cT.segment(0, 3) = pred_pt + cN * v1;
        cT.segment(3, 2) = UR_jnt;
        planeCenter.col(i) = cT;
      }
   
      // now writing planceCenter to csv file
      RPE_Utility util;
      std::string normalFile("planeNormal.txt");
      util.WriteCSVFile(normalFile, planeNormal);
      std::string centerFile("planeCenter.txt");
      util.WriteCSVFile(centerFile, planeCenter);

      // now compute U axis direction, for now just log it
      size_t numGroups = numPlanes / 4; // each group contains 4 planes
      Eigen::MatrixXd  ptGroup(3, 4), ptGroup2(3, 4);
      Eigen::MatrixXd  uAxis(3, 4), uAxis2(3, 4);
      for (size_t j=0; j< numGroups; j++) {
        for (size_t i=0; i < 4; i++) {
          ptGroup.col(i) = planeCenter.col(4*i + j).segment(0, 3);
          ptGroup2.col(i) = planeCenter.col(4*i + j).segment(0, 3);
        }
        
        // now computing the unit norm of ptGroup
        Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
        Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXd tmpV = svd2.matrixV();
        Eigen::Vector3d pnz = tmpV.col(2);
        strs.str("");
        strs << "U group" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        uAxis.col(j) = pnz;

        meanPt = ptGroup2.rowwise().mean();
        tmp = ptGroup2.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd22(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd22.matrixV();
        pnz = tmpV.col(2);
        strs.str("");
        strs << "U group using normal" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        uAxis2.col(j) = pnz;
      }
      std::string uAxisFile("uAxis.txt");
      util.WriteCSVFile(uAxisFile, uAxis);

      std::string uAxisFile2("uAxis2.txt");
      util.WriteCSVFile(uAxisFile2, uAxis2);
      Eigen::Vector3d uFinalAxis =  uAxis.rowwise().mean();
      uFinalAxis /= uFinalAxis.norm();

      Eigen::Vector3d uFinalAxis2 = uAxis2.rowwise().mean();
      uFinalAxis2 /= uFinalAxis2.norm();

      strs.str("");
      strs << "uFinalAxis=" << uFinalAxis.transpose() << std::endl;
      strs << "uFinalAxis2=" << uFinalAxis2.transpose() << std::endl;
      LOG_INFO(strs);

      // now compute the zero of U axis
      // first compute the direction of all rAxis with given fixed Us
      Eigen::MatrixXd  rAxis(3, 4), rAxis2(3, 4);
      Eigen::VectorXd angleRU(4), angleRU2(4); // angles between U and R
      for (size_t j=0; j < numGroups; j++) {
        for (size_t i=0; i < 4; i++) {
          ptGroup.col(i) = planeCenter.col(4 * j + i).segment(0, 3);
          ptGroup2.col(i) = planeNormal.col(4 * j + i).segment(0, 3);
        }
        
        // now computing the unit norm of ptGroup
        Eigen::VectorXd meanPt = ptGroup.rowwise().mean();
        Eigen::MatrixXd tmp = ptGroup.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::MatrixXd tmpV = svd2.matrixV();
        Eigen::Vector3d pnz = tmpV.col(2);
        strs.str("");
        strs << "R group" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        rAxis.col(j) = pnz;
        angleRU(j) = acos(pnz.dot(uFinalAxis));


        meanPt = ptGroup2.rowwise().mean();
        tmp = ptGroup2.colwise() - meanPt;

        // using svd decompsition to find the normal of 8-pt plane
        Eigen::JacobiSVD<Eigen::MatrixXd> svd22(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
        tmpV = svd22.matrixV();
        pnz = tmpV.col(2);
        strs.str("");
        strs << "R group normal" << j << ", meanPtz=" << meanPt.transpose() << ", pnz=" << pnz.transpose() << std::endl;
        LOG_INFO(strs);
        rAxis2.col(j) = pnz;
        angleRU2(j) = acos(pnz.dot(uFinalAxis2));
      }
      std::string rAxisFile("rAxis.txt");
      util.WriteCSVFile(rAxisFile, rAxis);

      std::string rAxisFile2("rAxis2.txt");
      util.WriteCSVFile(rAxisFile2, rAxis2);

      std::string angleRUFile("angleRU.txt");
      util.WriteCSVFile(angleRUFile, angleRU);

      std::string angleRUFile2("angleRU2.txt");
      util.WriteCSVFile(angleRUFile2, angleRU2);
      

      Eigen::VectorXd urDH;
      UR->GetCalibParamSet(&urDH);
      urDH(3) = a1;  // alpha0, alpha1, a0, a1, t0, t1, d0, d1, b0, b1
      //urDH(7) = d1;
      std::vector<double> urDHVec;
      EigenVec2StdVec(urDH, &urDHVec);
      UR->LoadCalibParamSet(urDHVec);  // update calibrated DH parameter 

      Frame tmp_baseoff, tmp_subbaseoff;
      UR->GetDefaultBaseOffFrame(&tmp_baseoff, &tmp_subbaseoff);
      
      
      // now UR mechanism set translational part of baseOffset
      // Frame tmpBase = sub_defaultBaseOff_;
      strs.str("");
      strs << GetName() << ": UR orig defaultSubBaseOff=" <<  tmp_baseoff.ToString(true) << std::endl;
      LOG_INFO(strs);
      Vec t(origin);
      tmp_baseoff.setTranslation(t);
      Eigen::VectorXd tmpBaseVec = tmp_baseoff.ToEigenVecQuat();
      strs.str("");
      strs << GetName() << ": UR baseoff is changed into " << tmp_baseoff.ToString(true)
      << ", in eigen vector=" << tmpBaseVec.transpose() << std::endl;
      LOG_INFO(strs);
      UR->SetDefaultBaseOff(tmpBaseVec, tmpBaseVec);  // modify the default base of UR obj
      
      //Vec t1(tmp_baseoff.segment(0,3));
      //Quaternion q1(tmp_baseoff(3), tmp_baseoff(4), tmp_baseoff(5), tmp_baseoff(6));
      sub_defaultBaseOff_ = tmp_baseoff; // modify sub base off of XYZUR too
      //sub_defaultBaseOff_.setQuaternion(q1);
      strs.str("");
      strs << GetName() << ": modify subdefulatBaseoff into" << sub_defaultBaseOff_.ToString(true) 
      <<", while defualtbaseoff is still " << defaultBaseOff_.ToString(true) << std::endl;
      LOG_INFO(strs);

      UR->updateCalibrateStatus(true);
      if (XYZ->isCalibrated()) {
        isDHCalibrated_ = true;  // set entire mechanism is calibrated
      }
      //sub_defaultBaseOff_ = tmpBase;  // modify the default subBase of XYZ_UR obj, defaultBase for XYZ is not changed
      return 0;
    } catch(...) {
       strs.str("");
       strs << GetName() << " gets exception in function " << __FUNCTION__ << std::endl;
       LOG_ERROR(strs);
       return -1;
    }
  }

  void XYZ_UR::SetDefaultBaseOff(const EigenDRef<Eigen::VectorXd> &baseoff,
                                 const EigenDRef<Eigen::VectorXd> &subbaseoff) {
     std::ostringstream strs;
     if (!initialized_) {
       strs.str("");
       strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
     }
     UR->SetDefaultBaseOff(subbaseoff, subbaseoff);
     strs.str("");
     strs << GetName() << ": UR Set defaultBaesoff=" << subbaseoff.transpose() << std::endl;

  
     XYZ->SetDefaultBaseOff(baseoff, baseoff);
     strs.str("");
     strs << GetName() << ": XYZ Set defaultBaesoff=" << baseoff.transpose() << std::endl;
     // also set itself
     BaseKinematicMap::SetDefaultBaseOff(baseoff, subbaseoff);
  }

    void XYZ_UR::SetDefaultBaseOff(const Eigen::VectorXd &baseoff,
                                 const Eigen::VectorXd &subbaseoff) {
     std::ostringstream strs;
     if (!initialized_) {
       strs.str("");
       strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
     }
     UR->SetDefaultBaseOff(subbaseoff, subbaseoff);
     strs.str("");
     strs << GetName() << ": UR Set defaultBaesoff=" << subbaseoff.transpose() << std::endl;

  
     XYZ->SetDefaultBaseOff(baseoff, baseoff);
     strs.str("");
     strs << GetName() << ": XYZ Set defaultBaesoff=" << baseoff.transpose() << std::endl;
     // also set itself
     BaseKinematicMap::SetDefaultBaseOff(baseoff, subbaseoff);
  }

  void XYZ_UR::GetDefaultBaseOff(EigenDRef<Eigen::VectorXd> *baseoff,
                                 EigenDRef<Eigen::VectorXd> *subbaseoff) {
     std::ostringstream strs;
     if (!initialized_) {
       strs.str("");
       strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
     }
     Eigen::VectorXd tmp_baseoff, tmp_subbaseoff;
     XYZ->GetDefaultBaseOff(baseoff, subbaseoff);
     tmp_baseoff = *baseoff;
     UR->GetDefaultBaseOff(baseoff, subbaseoff);
     tmp_subbaseoff = *baseoff;
     *baseoff = tmp_baseoff;
     *subbaseoff = tmp_subbaseoff;
     strs.str("");
     strs << GetName() << ": baseoff=" << *baseoff << ", subbaseoff=" << *subbaseoff << std::endl;
     LOG_INFO(strs);
  }

  int XYZ_UR::SetDependJacobianColumns(const std::vector<size_t> & d_cols) {
     std::ostringstream strs;
     if (!initialized_) {
       strs.str("");
       strs << GetName() << ":" << "XYZ_UR geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return -1;
     }
     std::vector<size_t> d_col_base, d_col_XYZ, d_col_UR, d_col_matA_XYZ, d_col_matA_UR;
     for (size_t i=0; i < d_cols.size(); i++) {
       size_t col = d_cols[i];
       if (col < 6) {  //base
          d_col_base.push_back(col);
       } else if (col < 6 + XYZ->GetDoF() * 5) {
          d_col_XYZ.push_back(col);
       } else if (col < 6 + DoF_ * 5) {
          d_col_UR.push_back(col - XYZ->GetDoF() * 5);
       } else {
          d_col_matA_XYZ.push_back(col - UR->GetDoF() * 5);
          d_col_matA_UR.push_back(col - XYZ->GetDoF() * 5);
       }
     }

     std::vector<size_t> d_XYZ, d_UR;
     d_XYZ.insert(d_XYZ.end(), d_col_base.begin(), d_col_base.end());
     d_XYZ.insert(d_XYZ.end(), d_col_XYZ.begin(), d_col_XYZ.end());
     d_XYZ.insert(d_XYZ.end(), d_col_matA_XYZ.begin(), d_col_matA_XYZ.end());

     // d_col_base.clear(); // UR needs to update base
     d_UR.insert(d_UR.end(), d_col_base.begin(), d_col_base.end());
     d_UR.insert(d_UR.end(), d_col_UR.begin(), d_col_UR.end());
     d_UR.insert(d_UR.end(), d_col_matA_UR.begin(), d_col_matA_UR.end());
  
     // logging d_XYZ, and d_UR
     // Eigen::VectorXd vec_XYZ, vec_UR;
     // StdVec2EigenVec(d_XYZ, &vec_XYZ);
     // StdVec2EigenVec(d_UR, &vec_UR);
     /*
     strs.str("");
     strs << "dep_col_XYZ=";
     for (size_t i=0; i < d_XYZ.size(); i++) {
         strs << d_XYZ[i] << ",";
     }
     strs <<", dep_col_UR=";
     for (size_t i=0; i < d_UR.size(); i++) {
         strs << d_UR[i] << ",";
     }
     strs << std::endl;
     LOG_INFO(strs);
     */
     XYZ->SetDependJacobianColumns(d_XYZ);
     UR->SetDependJacobianColumns(d_UR);
     return 0;
  }

  void XYZ_UR::SanityLooseBound(const double coef) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    XYZ->SanityLooseBound(coef);
    UR->SanityLooseBound(coef);
  }

  void XYZ_UR::DecayParam(const double coef) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    XYZ->DecayParam(coef);
    UR->DecayParam(coef);
  }

  void XYZ_UR::SetPitchCoef(const EigenDRef<Eigen::VectorXd> &pitch) {
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

  void XYZ_UR::SetRidgeCoef(const double linear_coef, const double ang_coef, const int cyc_mod, const double sam_region_scale,
                            const int coplanar_normal_option) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    XYZ->SetRidgeCoef(linear_coef, ang_coef, cyc_mod, sam_region_scale, coplanar_normal_option);
    UR->SetRidgeCoef(linear_coef, ang_coef, cyc_mod, sam_region_scale, coplanar_normal_option);
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

  //! calibration sanity check
  int XYZ_UR::CalibSanityCheck(const std::vector<double> &kine_para) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() <<  " geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -2000; 
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
    dh_UR.insert(dh_UR.end(), alpha.begin() + 3, alpha.end());
    dh_UR.insert(dh_UR.end(), a.begin() + 3, a.end());
    dh_UR.insert(dh_UR.end(), theta.begin() + 3, theta.end());
    dh_UR.insert(dh_UR.end(), d.begin() + 3, d.end());
    dh_UR.insert(dh_UR.end(), beta.begin() + 3, beta.end());
    int ret = XYZ->CalibSanityCheck(dh_XYZ);
    if (ret < 0) {
      return ret;
    }
    ret = UR->CalibSanityCheck(dh_UR);
    if (ret < 0) {
      return ret;
    }
    return 0;
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