#include "UJNT.hpp"
#include <common/pose.hpp>
namespace kinematics_lib {

UJNT::UJNT(): serialArm(2)  {
}
  
UJNT::UJNT(const std::vector<double> &kine_para): serialArm(kine_para) {
}

// nominal IK
int UJNT::CartToJnt(const Pose &pos, std::vector<double> *q) {
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
      strs << "UJNT geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }

    std::vector<int> branch; 
    pos.getBranchFlags(&branch);
    if (branch.empty()) {
       strs.str("");
       strs << "input pose has no branch information"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return -ERR_ROB_NO_BRANCH_INFO;
    }

    //r.GetEulerZYX(&yaw, &pitch, &roll);
    if (q->size() != DoF_) {
      q->resize(DoF_);
    }
  
    //get default frame
    Frame tip;
    pos.getFrame(&tip);

    // Rotation rr = tip.getRotation();
    //strs.str("");
    //strs << "IK of UJNT: orig orient= " << rr.ToEigenMat() <<  ", euler=" << rr.ToString() << std::endl;
    
    // recall only orientation w.r.t. world frame (adjoint to world frame)
    //Frame baseOrient;
    // baseOrient.setRotation(defaultBaseOff_.getRotation());

    // get relative frame
    Frame relTip = defaultBaseOff_.Inverse() *  tip; // * baseOrient;

    // get rotation matrix of relTip = Rot(z, yaw) * Rot(y, pitch)
    //Rotation r0 = Rotation::RotX(alpha_[0]);  // if alpha_[0] is not 0, then we need to
    // premultiply r0^{-1}   so that r = R_z(theta_1 or yaw) R_y(theta_2 or pitch)
    Rotation r =  relTip.getRotation(); //r0.Inverse() * relTip.getRotation() * r0;

    //strs << "r0= " << r0.ToEigenMat() << ", euler=" << r0.ToString() << std::endl;
    //strs << " exclude r0, r=" << r.ToEigenMat() << ", euler=" << r.ToString() << std::endl;
    Vec rx = r.UnitX();   // we solve yaw and pitch from rx
    Vec z1(0,0,1);

    double calpha = rx.dot(z1);
    double yaw, pitch;
    pitch = acos(calpha) - M_PI / 2.0 ;
 
    Vec rx_proj = rx - calpha * z1;
    yaw = atan2(rx_proj.y(), rx_proj.x());
    //strs << "calpha=" << calpha << ", yaw=" << yaw << ", pitch=" << pitch << std::endl;
    if (branch[2]) {   // nonflip
       q->at(0) = yaw;
       q->at(1) = -pitch;
    } else {   // flip solution
       q->at(1) = -(M_PI - pitch);
       if (yaw >= 0) {
          q->at(0) = yaw - M_PI;
       } else {
          q->at(0) = yaw + M_PI;
       }
    }
   
    std::vector<int>  jointTurns;
    pos.getJointTurns(&jointTurns);
  
    for (size_t i = 0; i < DoF_; i++) {
         // note sometimes, joint 0 could be running more than [-pi, pi]
         double turn = std::floor(q->at(i) / (2 * M_PI));
         double tmp_q = q->at(i) - turn * 2 * M_PI;
         // then make sure [-PI, PI]
         // to have 0 turn here
        if (tmp_q > M_PI) {
          turn += 1;
        }
        // q->at(3) -= jointTurns3 * 2 * PI;
        q->at(i) += (jointTurns[i] - turn) * 2 * M_PI;  // recall only joint 3 might moving more than 1 turn
        // subtract theta offset
        q->at(i) = (q->at(i) - theta_[i]) / pitch_(i);
    }
    //strs << "branch[2]=" << branch[2] << ", jnt0=" << q->at(0) << ", jnt1=" << q->at(1) << std::endl;    // set pitch_(R) = -1
    //LOG_INFO(strs);
    return 0;
}

/*
bool UJNT::GetDHFrame(const std::vector<double> & q,
                           const int  jnt_index, Frame *fm) const {
  std::ostringstream strs;
  if (!fm) {
    strs.str("");
    strs << GetName() << ":" << "input pose pointer is null in " << __FUNCTION__
              << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  int sz_q = q.size();
  if (jnt_index < 0 || jnt_index >= DoF_ || sz_q != DoF_)  {
    strs.str("");
    strs << GetName() << ":" << "jnt_index= " << jnt_index
         << ", sz_q=" << sz_q << " in function " __FUNCTION__
              << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  std::vector<double> qtmp(jnt_index+1);
  for (size_t i=0; i < jnt_index+1; i++) {
    qtmp[i] = q[i];
  }

  
  std::vector<double> a_tmp, alpha_tmp, beta_tmp,  d_tmp, theta_tmp;
  if (useCalibrated_) {  // if use calibrated parameters
      a_tmp = a_c_;
      alpha_tmp = alpha_c_;
      beta_tmp = beta_c_;
      d_tmp = d_c_;
      theta_tmp = theta_c_;
  } else {
      a_tmp = a_;
      alpha_tmp = alpha_;
      beta_tmp = beta_;
      d_tmp = d_;
      theta_tmp = theta_;
  }
  Eigen::VectorXd qv;
  StdVec2EigenVec(qtmp, &qv);
  // update dh based upon joint feedback
  UpdateDH(qv, &theta_tmp, &d_tmp);

  Frame tmp = defaultBaseOff_;
  for (size_t i=0; i < jnt_index+1; i++) {
    tmp = tmp * Frame::DH_Craig1989_EX(a_tmp[i],
                                    alpha_tmp[i],
                                    beta_tmp[i],
                                    d_tmp[i],
                                    theta_tmp[i]);
    
  }

  *fm = tmp;
  return true;
}
*/

int UJNT::JntToCart(const std::vector<double> & q,
                       Pose *p) {
    std::ostringstream strs;
    if (!p) {
        strs.str("");
        strs << "input pose pointer is null in " << __FUNCTION__
                 << ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_POINTER_NULL;
    }
    std::vector<double> a_tmp, alpha_tmp, beta_tmp,  d_tmp, theta_tmp;
    if (useCalibrated_) {  // if use calibrated parameters
       a_tmp = a_c_;
       alpha_tmp = alpha_c_;
       beta_tmp = beta_c_;
       d_tmp = d_c_;
       theta_tmp = theta_c_;
    } else {
       a_tmp = a_;
       alpha_tmp = alpha_;
       beta_tmp = beta_;
       d_tmp = d_;
       theta_tmp = theta_;
    }
    Eigen::VectorXd qv;
    StdVec2EigenVec(q, &qv);
    // update dh based upon joint feedback
    UpdateDH(qv, &theta_tmp, &d_tmp);

    Frame tmp = defaultBaseOff_;
    for (size_t i=0; i < DoF_; i++) {
      tmp = tmp * Frame::DH_Craig1989_EX(a_tmp[i],
                                      alpha_tmp[i],
                                      beta_tmp[i],
                                      d_tmp[i],
                                      theta_tmp[i]);
    }

    // continue testing afternoon
    /*
    Eigen::VectorXd  a_eg, alpha_eg, beta_eg, d_eg, theta_eg;
    StdVec2EigenVec(a_tmp, &a_eg);
    StdVec2EigenVec(alpha_tmp, &alpha_eg);
    StdVec2EigenVec(theta_tmp, &theta_eg);
    StdVec2EigenVec(d_tmp, &d_eg);
    StdVec2EigenVec(beta_tmp, &beta_eg);

    strs.str("");
    strs << "defaultBase=" << defaultBaseOff_.ToString(true) << std::endl;
    strs << "alpha=" << alpha_eg << std::endl;
    strs << "a=" << a_eg << std::endl;
    strs << "theta=" << theta_eg << std::endl;
    strs << "d=" << d_eg << std::endl;
    strs << "beta=" << beta_eg << std::endl;
    */
    // ...............................

    // multiply this to make sure  the final rotation is R_x(alpha[0])R_z(yaw)R_y(pitch) for easy IK above
    Rotation r = Rotation::RotX(-M_PI / 2.0);
    //Rotation r0 = Rotation::RotX(alpha_[0]);  // if alpha_[0] is not 0, then we need to
    //Rotation r1 = defaultBaseOff_.getRotation();
    Frame tmp1; //, tmp2, tmp3;
    //Rotation rtmp = tmp.getRotation();
    //strs << "before multiply Rotx, tmp= " << tmp.ToString(true)  << ", rtmp=" << rtmp.ToString() << std::endl;
    tmp1.setRotation(r);
    //tmp2.setRotation(r0.Inverse());
    //tmp3.setRotation(r1.Inverse());
    tmp = tmp * tmp1; // * tmp2 * tmp3;
    //rtmp = tmp.getRotation();
    //strs << "after multiply Rotx, final tmp =" << tmp.ToString(true) << ", rtmp=" << rtmp.ToString() << std::endl;
    //LOG_INFO(strs);

    // config flags
    std::vector<int>  branchFlags;
    //turn flags
    std::vector<int>  jointTurns;
    // get flags and turn from final theta and d including their initial values
    // plus joint displacement
    UpdateConfigTurn(theta_tmp, d_tmp, &branchFlags, &jointTurns);
    /*
    strs.str("");
    strs << " tmp.v " << tmp.getTranslation().ToString() << std::endl;
    strs << " tmp.q " << tmp.getQuaternion().ToString(false) << std::endl;
    strs << " tmp.r "  << tmp.getRotation().ToString() << ", mat=" << tmp.getRotation().ToEigenMat()  << std::endl;
    LOG_INFO(strs);
    */
    p->setFrame(tmp);
    //strs.str("");
    //strs << "tmp=" << tmp.ToString(true) << std::endl;
    //LOG_INFO(strs);
    p->setBranchFlags(branchFlags);
    p->setJointTurns(jointTurns);
    return 0;
}

int UJNT::JntToCart(const std::vector<double> &q,
                     const std::vector<double> &qdot,
                     Pose *p, Twist *v) {
    std::ostringstream strs;
    if (!p || !v) {
      strs.str("");
      strs << GetName() << "input pose and twist parameter is null in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
    if (!initialized_) {
      strs.str("");
      strs << GetName() << "Scara geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (q.size() != DoF_ || qdot.size() != DoF_) {
      strs.str("");
      strs << GetName() << "Input q and qdot dimension does not match with the robot"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM; 
    }
    /*
    int ret = JntToCart(q, p);
    if (ret < 0) {
      return ret;
    } */
    // need to compute the Jacobian
    std::vector<double> a_tmp, alpha_tmp, beta_tmp, d_tmp, theta_tmp;
    if (useCalibrated_) {  // if use calibrated parameters
       a_tmp = a_c_;
       alpha_tmp = alpha_c_;
       beta_tmp = beta_c_;
       d_tmp = d_c_;
       theta_tmp = theta_c_;
    } else {
       a_tmp = a_;
       alpha_tmp = alpha_;
       beta_tmp = beta_;
       d_tmp = d_;
       theta_tmp = theta_;
    }
    Eigen::VectorXd qv;
    StdVec2EigenVec(q, &qv);
    // update dh based upon joint feedback
    UpdateDH(qv, &theta_tmp, &d_tmp);

    std::vector<double> kine_para;
    kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
    kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
    kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
    kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
    kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
    // Pose tmp_p;
    Eigen::MatrixXd Jp_t, Jp_r;
    // Pose p0;
    int ret = CalcJacobian(kine_para, p, &Jp_t, &Jp_r);
    if (ret < 0) {
        return ret;
    }
    // picking submatrix of Jp_t and Jp_r, and then multiplying qdot
    // to obtain twist
    Eigen::MatrixXd Mt(3, DoF_), Mr(3, DoF_);
    if (PickSubJacobian(Jp_t, Jp_r, &Mt, &Mr)) {
      Eigen::VectorXd vdot(DoF_);
      StdVec2EigenVec(qdot, &vdot);
      Eigen::Vector3d gvt = Mt * vdot ;
      Eigen::Vector3d gvr = Mr * vdot;
      v->setLinearVel(Vec(gvt(0), gvt(1), gvt(2)));
      v->setAngularVel(Vec(gvr(0), gvr(1), gvr(2)));
      return 0;
    }
    return -1;
}


int UJNT::CalcJacobian(const std::vector<double> &kine_para,
                    Pose *p,
                    Eigen::MatrixXd * Jp_t,
                    Eigen::MatrixXd * Jp_r,
                    const bool reduction) {
  int ret = serialArm::CalcJacobian(kine_para, p, Jp_t, Jp_r, reduction);
  if (ret < 0) {
    return ret;
  }
  
  Frame tmp;
  p->getFrame(&tmp);
  Rotation r = Rotation::RotX(-M_PI / 2.0);
  //Rotation r0 = Rotation::RotX(alpha_[0]);  // if alpha_[0] is not 0, then we need to
  //Rotation r1 = defaultBaseOff_.getRotation();
  Frame tmp1; //, tmp2, tmp3;
  tmp1.setRotation(r);
  //tmp2.setRotation(r0.Inverse());
  //tmp3.setRotation(r1.Inverse());
  tmp = tmp * tmp1; // * tmp2 * tmp3;
  p->setFrame(tmp);

  return 0;
}

int UJNT::CalibSanityCheck(const std::vector<double> &cal_DH) {
    std::ostringstream strs;
    if (cal_DH.size() < 5 * DoF_) {
      strs << GetName() << ":" << "input calibrated DH has wrong dimension " << cal_DH.size() <<
           " which is less than minimal dimension " << 5 * DoF_ << std::endl;
      LOG_ERROR(strs);
      return -2000;
    }
    std::vector<double> inalpha(DoF_), ina(DoF_), intheta(DoF_), ind(DoF_), inbeta(DoF_);
    for (size_t k=0; k<DoF_; k++) {
       inalpha[k] = cal_DH[k];
       ina[k] = cal_DH[DoF_ + k];
       intheta[k] = cal_DH[2 * DoF_ + k];
       ind[k] = cal_DH[3 * DoF_ + k];
       inbeta[k] = cal_DH[4 * DoF_ + k];
    }
    for (size_t k=0; k < DoF_; k++) {
      double dalpha = fabs(inalpha[k] -alpha_[k]);
      double limit_alpha = sanity_loose_coef_ * std::max(MAX_ALPHA_DIFF_REG, fabs(alpha_[k]) * MAX_DH_PERC / 100.0);
      if (dalpha > limit_alpha) {
        strs << "";
        strs << GetName() << ":" << "Calib. santity check fails: alpha_c_= " << inalpha[k] << " diffs from alpha_= " << alpha_[k] 
             << " by " << dalpha <<  " which is greater than limit value " << limit_alpha << std::endl;
        LOG_ERROR(strs);
        return -2001;
      }
    }

    for (size_t k=0; k < DoF_; k++) {
      double dt = fabs(intheta[k] - theta_[k]);
      double limit_t = sanity_loose_coef_ * std::max(MAX_THETA_DIFF_REG, fabs(theta_[k]) * MAX_DH_PERC / 100.0);
      if (dt > limit_t) {
        strs << "";
        strs << GetName() << ":" << "Calib. santity check fails: theta_c_ " << intheta[k] << " diffs from theta_ " << theta_[k] 
        << " by " << dt << " which greater than limit value " << limit_t << std::endl;
        LOG_ERROR(strs);
        return -2003;
      }
    }
    
    for (size_t k=0; k < DoF_; k++) {
      double dbeta = fabs(inbeta[k] - beta_[k]);
      double limit_beta = sanity_loose_coef_ * std::max(MAX_BETA_DIFF_REG, fabs(beta_[k]) * MAX_DH_PERC / 100.0);
      if (dbeta > limit_beta) {
        strs << "";
        strs << GetName() << ":" << "Calib. santity check fails: d_c_ " << inbeta[k] << " diffs from beta_ " << beta_[k] 
        << " by " << dbeta << " which greater than limit value " << limit_beta << std::endl;
        LOG_ERROR(strs);
        return -2005;
      }
    }
    return 0;
}


// nominal velocity IK
int UJNT::CartToJnt(const Pose &p, const Twist &v,
                     std::vector<double> *q,
                     std::vector<double> *qdot) {
    std::ostringstream strs;
    if (!q || !qdot) {
       strs.str("");
       strs << "Input q and qdot vectors are null"
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
    // this analytic IK is using canonical robot model, for non-canonical 
    // model (e.g., model after calibration) requires using GI-based iteration
    // method
    if (useCalibrated_) {
      strs.str("");
      strs << "default CartToJnt function must using canonical kinematic"
               " model in" << __FUNCTION__
                << ", at line " << __LINE__  << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_DEFAULT_IK_NOT_CANO;
    }
    int ret = CartToJnt(p, q);
    if (ret < 0) {
      return ret;
    }
    if (qdot->size() != DoF_) {
      qdot->resize(DoF_);
    }
    // need to compute the Jacobian
    std::vector<double> a_tmp, alpha_tmp, beta_tmp, d_tmp, theta_tmp;
    a_tmp = a_;
    alpha_tmp = alpha_;
    beta_tmp = beta_;
    d_tmp = d_;
    theta_tmp = theta_;
    Eigen::VectorXd qv;
    StdVec2EigenVec(*q, &qv);
    UpdateDH(qv, &theta_tmp, &d_tmp); 
    std::vector<double> kine_para;
    kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
    kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
    kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
    kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
    kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
    
    Pose tmp_p;
    Eigen::MatrixXd Jp_t, Jp_r;
    ret = CalcJacobian(kine_para, &tmp_p, &Jp_t, &Jp_r);   // here no need to use reduction, because we use angular velocity inverse kinematics
    if (ret < 0) {
        return ret;
    }
    // picking submatrix of Jp_t and Jp_r, and then multiplying qdot
    // to obtain twist
    Eigen::MatrixXd M;
    Eigen::VectorXd b;
    Eigen::MatrixXd Js_t, Js_r;
    PickRotSubJacobian(Jp_t, Jp_r, &Js_t, &Js_r);

    size_t rowTrans = Js_t.rows();
    size_t  rowRot = Js_r.rows();
    M.resize(rowTrans + rowRot, rowTrans+rowRot);
    b.resize(rowTrans + rowRot);
    if (rowTrans > 0) {
      M.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
    }
    if (rowRot > 0) {
      M.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;
    }
    Eigen::MatrixXd MTM = M.transpose() * M;
    double det = MTM.determinant();
    if (fabs(det) < K_EPSILON) {
        std::ostringstream strs;
        strs << GetName() <<  " is singular, can not compute IK "
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        return -ERR_ROB_JACOBIAN_IK_SINGULAR;
    }
     
    Vec t_v = v.getLinearVel();
    Vec t_w = v.getAngularVel();
    
    double spdNorm = PickRotCartErr(t_v.ToEigenVec(),
                                 t_w.ToEigenVec(), 
                                  &b);
    Eigen::VectorXd qdot_tmp = MTM.inverse() * M.transpose() * b;
    EigenVec2StdVec(qdot_tmp, qdot);
    return 0;
}

void  UJNT::UpdateConfigTurn(const std::vector<double> & theta,
                              const std::vector<double> &d,
                              std::vector<int>  *branchFlags,
                              std::vector<int>  *jointTurns) const {
    std::ostringstream strs;
    if (!branchFlags || !jointTurns) {
        strs.str("");
        strs << GetName() << "Input branchFlags and jointTurns are null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return;
    }
    // for UJNT, there are 0 flags, so all set to 0 (eBranchLeft)
    branchFlags->resize(3, eBranchLeft);
    Rotation r=Rotation::RotY(theta[1]);
    Vec v = r.UnitX();
    if (v.x() >= 0) {   //  non-flip  (right half circle, in zx plane)
      branchFlags->at(2) = eBranchRight;
    } else {  // flip (or left half circle in zx plane)
      branchFlags->at(2) = eBranchLeft;
    }
    // DoF turn flags, shall all be 0, because it is prismatic
    jointTurns->resize(DoF_, 0);
    std::vector<double> q_tmp = theta;
    double tmp_q = 0;
    for (size_t i=0; i < DoF_; i++) {
      jointTurns->at(i) = std::floor(q_tmp[i] / (2 * M_PI));
      tmp_q = q_tmp[i] - jointTurns->at(i) * 2 * M_PI;
      // then make sure [-PI, PI]
      // to have 0 turn here
      if (tmp_q > M_PI) {
        jointTurns->at(i) += 1;
      }
    }            
}

// using ballbar
bool UJNT::PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r,
                                const bool reduction) {
     std::ostringstream strs;
     if (!Js_t || !Js_r) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
     size_t row_t = Jp_t.rows();
     size_t col_t = Jp_t.cols();
     size_t row_r = Jp_r.rows();
     size_t col_r = Jp_r.cols();

     if (row_t < 3 || col_t < DoF_ || row_r < 3 || col_r < DoF_) {
        strs.str("");
        strs << GetName() << " input Jacobian matrices have wrong dimension "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
     }
    
    Js_t->resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*Js_t).col(i) = Jp_t.col(5 * i + 2) * pitch_(i);
    }
    Js_r->resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*Js_r).col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
    }      
    return true;
}


bool UJNT::PickRotSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r) {
    std::ostringstream strs;
    if (!Js_t || !Js_r) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    size_t row_r = Jp_r.rows();
    size_t col_r = Jp_r.cols();

    if (row_r < 3 || col_r < DoF_) {    //|| row_r <3 || col_r < DoF_) {
        strs.str("");
        strs << GetName() << " input Jacobian matrices have wrong dimension "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }

    Js_r->resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*Js_r).col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
    }      
    return true;
}
   
// for  UJNT + a ballbar for measuring
bool UJNT::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                   const Eigen::MatrixXd &Jp_r, 
                                   Eigen::MatrixXd *Js_t1, 
                                   Eigen::MatrixXd *Js_r1,
                                   const bool reduction) {
     std::ostringstream strs;
     if (!Js_t1 || !Js_r1) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    size_t row_t = Jp_t.rows();
    size_t col_t = Jp_t.cols();
    size_t row_r = Jp_r.rows();
    size_t col_r = Jp_r.cols();
    
    Js_t1->resize(row_t, col_t);
    *Js_t1 = Jp_t;
    for (size_t i=0; i < DoF_; i++) {
      (*Js_t1).col(5 * i + 2) = Jp_t.col(5 * i + 2) * pitch_(i);
    }
    Js_r1->resize(row_r, col_r);
    *Js_r1 = Jp_r;
    for (size_t i=0; i < DoF_; i++) {
      (*Js_r1).col(5 * i + 2) = Jp_r.col(5 * i + 2) * pitch_(i);
    }
    return true;
}

bool UJNT::PickRotSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r, 
                                       Eigen::MatrixXd *Js_t1, 
                                       Eigen::MatrixXd *Js_r1) {
    std::ostringstream strs;
    if (!Js_t1 || !Js_r1) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    size_t row_r = Jp_r.rows();
    size_t col_r = Jp_r.cols();
    //size_t row_r = Jp_r.rows();
    //size_t col_r = Jp_r.cols();
    
    Js_r1->resize(row_r, col_r);
    *Js_r1 = Jp_r;
    for (size_t i=0; i < DoF_; i++) {
      (*Js_r1).col(5 * i + 2) = Jp_r.col(5 * i + 2) * pitch_(i);
    }
    return true;
}
   
// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm (using ballbar)
double  UJNT::PickCartErr(const Eigen::Vector3d &errT,
                           const Eigen::Vector3d &errR, 
                           Eigen::VectorXd *b,
                           const bool reduction) {
    std::ostringstream strs;
    double val = errT.norm();
    if (!b) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    b->resize(3);
    b->block(0, 0, 3, 1) = errT;
    return val;
}


double  UJNT::PickRotCartErr(const Eigen::Vector3d &errT,
                               const Eigen::Vector3d &errR, 
                               Eigen::VectorXd *b) {
    std::ostringstream strs;
    double val = errR.norm();
    if (!b) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    b->resize(3);
    b->block(0, 0, 3, 1) = errR;
    return val;
}
   
   //! virtual function for updating actual DH parameters based upon joint feedback
   // orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k>
   // jnt angles def. depends on specific robot type
void UJNT::UpdateDH(const std::vector<double> &orig_dh,
                     const Eigen::VectorXd &jnt,
                     std::vector<double> *new_dh) const {
    std::ostringstream strs;
    if (!new_dh) {
        strs.str("");
        strs << GetName() << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    if (orig_dh.size() != 5 * DoF_ ||  jnt.size() < DoF_) {
        strs.str("");
        strs << GetName() << " input parameters have wrong size, origin dh size= " << 
             orig_dh.size() << ", jnt size =" << jnt.size()   
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    *new_dh = orig_dh;
    for (size_t i=0; i < DoF_; i++) {
       new_dh->at(2 * DoF_ + i) = orig_dh[2 * DoF_ + i] + jnt(i) * pitch_(i);
    }
}

   //! virtual function for updating actual DH parameters based upon joint feedback
   // Note: alpha, a, theta, d, beta has their initial values, which will be updated
   // based upon jnt input
void UJNT::UpdateDH(const Eigen::VectorXd &jnt,
              std::vector<double> *theta,
              std::vector<double> *d) const {
  std::ostringstream strs;
  if (!theta || !d) {
        strs.str("");
        strs << GetName() <<" input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }
  if (jnt.size() < DoF_ ||  theta->size() != DoF_ || d->size() != DoF_) {
       strs.str("");
       strs << GetName() << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }

  for (size_t i=0; i < DoF_; i++) {
    theta->at(i) += jnt(i) * pitch_(i);
  }
}


int UJNT::HomotopyAlg1(const std::vector<double> &init_jnt0,
        const Frame &userTool, std::vector<double> *init_jnt) {
    std::ostringstream strs;
    if (!init_jnt) {
      strs.str("");
      strs << GetName() << ":" << "Input final_plane or final_tool_offset pointer is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
       
    }
    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << "serial robot is not calibrated, "
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_WITHOUT_CALIB;
    }

    Eigen::VectorXd p_alpha_c(DoF_), p_alpha(DoF_), p_a_c(DoF_), p_a(DoF_),
                    p_theta_c(DoF_),
                    p_theta(DoF_), p_d_c(DoF_), p_d(DoF_), p_beta(DoF_),
                    p_beta_c(DoF_), jnt0(DoF_);
    for (size_t i=0; i < DoF_; i++) {
      p_alpha_c(i) = this->alpha_c_[i];
      p_alpha(i) = this->alpha_[i];
      p_a_c(i) = this->a_c_[i];
      p_a(i) = this->a_[i];
      p_theta_c(i) = this->theta_c_[i];
      p_theta(i) = this->theta_[i];
      p_d_c(i) = this->d_c_[i];
      p_d(i) = this->d_[i];
      p_beta_c(i) = this->beta_c_[i];
      p_beta(i) = this->beta_[i];
      jnt0(i) = init_jnt0[i];
    }
    // compute the diff of 
    Eigen::VectorXd diff_alpha = p_alpha_c - p_alpha;
    Eigen::VectorXd diff_a = p_a_c - p_a;
    Eigen::VectorXd diff_theta = p_theta_c - p_theta;
    Eigen::VectorXd diff_d = p_d_c - p_d;
    Eigen::VectorXd diff_beta = p_beta_c - p_beta;
    double norm_dalpha = diff_alpha.norm();
    double norm_da = diff_a.norm();
    double norm_dtheta = diff_theta.norm();
    double norm_dd = diff_d.norm();
    double norm_dbeta = diff_beta.norm();
    
    int step1 = std::floor(std::max(std::max(norm_dalpha, norm_dtheta), norm_dbeta) 
                            / DH_ANGULAR_EPSILON);
    int step2 = std::floor(std::max(norm_da, norm_dd) / DH_LINEAR_EPSILON);
    int step = std::max(step1, step2);
    Eigen::VectorXd step_diff_alpha = diff_alpha / step;
    Eigen::VectorXd step_diff_a = diff_a / step;
    Eigen::VectorXd step_diff_beta = diff_beta / step;
    Eigen::VectorXd step_diff_theta = diff_theta / step;
    Eigen::VectorXd step_diff_d = diff_d / step;
    
    // we need a diff para vector to put all above variation together
    Eigen::VectorXd var_para(5 * DoF_);
    for (size_t i=0; i < DoF_; i++) {
        var_para(5 * i + 0) = step_diff_alpha(i);
        var_para(5 * i + 1) = step_diff_a(i);
        var_para(5 * i + 2) = step_diff_theta(i);
        var_para(5 * i + 3) = step_diff_d(i);
        var_para(5 * i + 4) = step_diff_beta(i);
    }

    Vec init_normal(orient_normal_);  // initial normal
    strs.str("");
    strs << " orient_normal_=" << orient_normal_ << ", in " << __FUNCTION__ << ", line=" << __LINE__ <<  std::endl;
    LOG_INFO(strs);
    std::vector<double> kine_para(5 * DoF_, 0), tmp_para; // clearing kine_para
    // A djnt + B dpara = 0
    Eigen::MatrixXd A, B;
    for (int j=0; j<step; j++) {
      // fill in the value of p_alpha, p_a, p_theta, p_d
      for (size_t i=0; i<DoF_; i++) {
        kine_para[i] = p_alpha(i) + step_diff_alpha(i) * j;
        kine_para[DoF_ + i] = p_a(i) + step_diff_a(i) * j;
        kine_para[2 * DoF_ + i] = p_theta(i) + step_diff_theta(i) * j;
        kine_para[3 * DoF_ + i] = p_d(i) + step_diff_d(i) * j;
        kine_para[4 * DoF_ + i] = p_beta(i) + step_diff_beta(i) * j;
      } 
      UpdateDH(kine_para, jnt0, &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << " CalJacobian got error " << ret
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
        LOG_ERROR(strs); 
        return ret;
      }
      Pose p1 = p * userTool;
      // we get R_04, the rotation matrix between 0 and 4
      // Rotation r_04 = p.getRotation();  
      // Vec t_04 = p.getTranslation();
      Vec t_e = p1.getTranslation();  //r_04 * toolOffset + t_04;
      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      // compute the translational Jacobian w.r.t. origin of default T.
      Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;
      
      Eigen::MatrixXd  Js_t, Js_t1;
      Vec c_normal = p1.getRotation() * init_normal;  // -current normal
      Eigen::Matrix3d  currentNMat;
      (-c_normal).ToHat(&currentNMat);  // hat(currentN)
      Eigen::MatrixXd Jp_r_new = currentNMat * Jp_r; // (-N)^ * Jp_r 
      // pick joint angle related sub jacobians, because we want to 
      // find joint angles correspond to calibrated parameters
      PickRotSubJacobian(Jt_tmp, Jp_r_new, &Js_t, &A);

      // pick translation and rotational jacobian for parameter part.
      PickRotSubJacobianForPara(Jt_tmp, Jp_r_new, &Js_t1, &B);

      Eigen::MatrixXd ATA = A.transpose() * A;
      double detA = ATA.determinant();
      if (fabs(detA) < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << ":" << "Scara calibration regression matrix A is singular "
              << " even with depend columns removed "
              << " with detA= " << detA << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_JAC_SINGULAR;
      }
      Eigen::VectorXd delta_t = -ATA.inverse() * A.transpose() * B * var_para;
      jnt0 += delta_t;
    }
    EigenVec2StdVec(jnt0, init_jnt);
    return 0;
}

int UJNT::OptimizeJntAfterCalib(const std::vector<double> &init_jnt0,
                                 const refPose &ps,
                                 std::vector<double> *opt_jnt) {
    std::ostringstream strs;
    if (!opt_jnt) {
      strs.str("");
      strs << GetName() << ":" << "Input final_plane or final_tool_offset pointer is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
    }
    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << "Scara robot is not calibrated, "
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_WITHOUT_CALIB;  
    }
    // retrieve tool info, and tool offset vector
    Frame userTool;
    ps.getTool(&userTool);
    // Vec userToolOffset = userTool.getTranslation();
  
    // convert ps to  cps (under default base frame, but under the same userTool)
    refPose cps;
    // base_off_c_ replaced by default base, because in traj. compensation, the desired traj
    // is always w.r.t. robot base (or default frame). Sensor frame will not be used anymore
    ps.getPoseUnderNewRef(Frame(), userTool, &cps);
    strs.str("");
    strs << GetName() << ":" << "before comp, under default base and  new tool, desired rps is " << cps.ToString(false) << std::endl;
    LOG_INFO(strs);
    // using homotopty method to find a rough solution to 
    // f(calib_para, mid_jnt) ~= f(old_para, init_jnt)
    std::vector<double> init_jnt(DoF_, 0);
    if (HomotopyAlg1(init_jnt0, userTool, &init_jnt) < 0) {
      strs.str("");
      strs << GetName() << ":" << "homotopy algorithm fails in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_COMP_HOMOTOPY_FAIL;
    }

    // first make a copy of init_jnt
    Eigen::VectorXd jnt_tmp, jnt_iter, jnt_best_iter;
    StdVec2EigenVec(init_jnt, &jnt_tmp);
    // desired rotation
    Rotation d_r = cps.getRotation();
    // desired yaw
    // double yaw_d, pitch_d, roll_d;
    // if (!d_r.GetEulerZYX(&yaw_d, &pitch_d, &roll_d)) {
          //return -61;
    //}
    strs.str("");
    strs << " orient_normal_=" << orient_normal_ << ", in " << __FUNCTION__ << ", line=" << __LINE__ <<  std::endl;
    LOG_INFO(strs);
    Vec init_normal(orient_normal_);
    // Rotation r_tmp = userTool.getRotation();
    // init_normal = r_tmp.Inverse() * init_normal;
    Vec  c_normal_d =  d_r *  init_normal;  // desired normal

    // desired translation
    Vec d_t = cps.getTranslation();
    // step 1, Define the regression matrix   A dJoint = b
    // where \partial f / \partial Joint d\Joint = desired_pose - f(\joint), where desired_pose  and f(Joint) are using trans+euler repre.
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    double estimation_err = std::numeric_limits<double>::max();
    // diff of previous_err - current_err
    //double gap = std::numeric_limits<double>::max();
    // initial step size, always set 1.0
    unsigned int numSteps = CALIB_LINE_SEARCH_STEPS;
    double step_size = 1.0 / numSteps;
    int cur_iter = 0;
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    while (estimation_err > MAX_CALIB_MATCHING_ERR &&
            cur_iter < MAX_CALIB_ITER) {
      cur_iter++;
      UpdateDH(kine_para, jnt_tmp,  &tmp_para); 
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
           return ret;
      }

      Pose p1 = p * userTool;
      // we get R_04, the rotation matrix between 0 and flage
      // Rotation r_04 = p.getRotation();
      Rotation r_final = p1.getRotation();
      Vec  c_normal = r_final * init_normal;
      /*
      double yaw, pitch, roll;
      if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
        strs.str("");
        strs << GetName() << ":" << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line " << __LINE__
        << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROT2EULER_SINGULAR;
      }
      */
      
      //Vec t_04 = p.getTranslation();
      Vec t_e =  p1.getTranslation();  // r_04 * userToolOffset + t_04;
      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      // compute the translational Jacobian w.r.t. origin of default T.
      Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;
      Eigen::Vector3d errT= (d_t - t_e).ToEigenVec();
      Vec error_normal = c_normal_d - c_normal;
      Eigen::Vector3d errR= error_normal.ToEigenVec();

      /*
      Eigen::Vector3d Eub(roll, pitch, yaw); 
      Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
      Eigen::Vector3d col1(0, 0, 1);
      EulerDiff.col(2) = col1;
      Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
      EulerDiff.col(1) = col2;
      Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
      EulerDiff.col(0) = col3;
      
      double detEulerDiff = EulerDiff.determinant();
      if (fabs(detEulerDiff) < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << ":" << "Euler spatial jacobian is singular"
              << " with detA= " << detEulerDiff << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_JAC_SINGULAR;
      }
      */
      Eigen::Matrix3d  currentNMat;
      (-c_normal).ToHat(&currentNMat);  // hat(currentN)
      Eigen::MatrixXd Jr_tmp =  currentNMat * Jp_r;   // EulerDiff.inverse() * Jp_r;  //  Jr_tmp * d para = errR
      Eigen::MatrixXd Js_t;
      // get joint vector related jacobian
      PickRotSubJacobian(Jt_tmp, Jr_tmp, &Js_t, &A);
      
      double tmp_err = PickRotCartErr(errT, errR, &b); // given full error vector, pick those mathing with robot type
      Eigen::VectorXd delta_t(DoF_);

      Eigen::MatrixXd ATA = A.transpose() * A;
      double detA = ATA.determinant();
      if (fabs(detA) < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << ":" << "Scara calibration regression matrix ATA is singular "
              << " even with depend columns removed "
              << " with detA= " << detA << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_JAC_SINGULAR;
      }
      delta_t = ATA.inverse() * A.transpose() * b;
      jnt_best_iter = jnt_tmp;
      // dynamic step size adjustment
      for (size_t i=0; i < numSteps; i++) {
         jnt_iter = jnt_tmp + (i+1) * step_size * delta_t;
         UpdateDH(kine_para, jnt_iter,  &tmp_para); 
         Eigen::MatrixXd Jp_t1, Jp_r1;
         Pose p2;
         // compute the expected values from known canonical kinematic parameters
         // i.e., not-calibrated parameter set
         ret = CalcJacobian(tmp_para, &p2, &Jp_t1, &Jp_r1);
         if (ret < 0) {
           return ret;
         }
         
         p1 = p2 * userTool;
         r_final = p1.getRotation();
         c_normal = r_final * init_normal;
         error_normal = c_normal_d - c_normal;
         errR= error_normal.ToEigenVec();
         // we get R_04, the rotation matrix between 0 and flange
         //Rotation r = p1.getRotation();
         //Rotation r_final = r * userTool.getRotation();
         /*
         if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
           strs.str("");
           strs << GetName() << ":" << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line " << __LINE__
           << std::endl;
           LOG_ERROR(strs);
           return -ERR_ROT2EULER_SINGULAR;
         }
         t_e = r * userToolOffset + p1.getTranslation();
         */
         t_e = p1.getTranslation();
         errT= (d_t - t_e).ToEigenVec();
         
         double err = PickRotCartErr(errT, errR, &b); // given full error vector, pick those mathing with robot type
         if (err < tmp_err) {
             jnt_best_iter = jnt_iter;
             tmp_err = err;
         }
      }
      if (tmp_err >= estimation_err - MAX_CALIB_MATCHING_ERR) {
        if (numSteps < MAX_CALIB_STEPS) {
          step_size /= 2.0;
          numSteps *= 2;
        } else {
          break;
        }
      } else {
          estimation_err = tmp_err;
          jnt_tmp = jnt_best_iter;
      }

      strs.str("");
      strs << GetName() << ":"  << "tmp_err=" << tmp_err 
            << ", while estimation_err " <<  estimation_err << " at iteration " << cur_iter << std::endl;
      LOG_INFO(strs);
      // jnt_tmp += delta_t;
    }
    if (estimation_err > MAX_CALIB_MATCHING_ERR) { // not convergent
        strs.str("");
        strs << GetName() << ":"  << "parameter not convergent within maximal iter limit,"
                << "estimation_err at iter " << cur_iter << " is " <<
              estimation_err << std::endl;
        LOG_ERROR(strs);
        //std::cout << "The desire pose is " << ps.ToString(false) << std::endl;
        return -ERR_COMP_ALG_DIVERGE;
    } else {
        strs.str("");
        strs << GetName() << ":" << "parameter convergent within maximal iter limit,"
                << "estimation_err at iter " << cur_iter << " is " <<
              estimation_err << std::endl;
        LOG_INFO(strs);
    }
    // return jnt_tmp
    EigenVec2StdVec(jnt_tmp, opt_jnt);
    return 0;
}

}