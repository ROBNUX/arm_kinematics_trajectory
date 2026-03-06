#include "robnux_kinematics_map/serialArm.hpp"
#include "robnux_kdl_common/pose.hpp"
#include <string>

namespace kinematics_lib {

serialArm::serialArm(size_t DoF): BaseKinematicMap(DoF, DoF),
    alpha_(Eigen::VectorXd::Zero(DoF)),
    alpha_c_(alpha_),
    a_(Eigen::VectorXd::Zero(DoF)),
    a_c_(a_),
    d_(Eigen::VectorXd::Zero(DoF)),
    d_c_(d_),
    theta_(Eigen::VectorXd::Zero(DoF)),
    theta_c_(theta_) {
    jnt_names_.resize(DoF);
    for (size_t i=0; i < DoF; i++) {
      jnt_names_[i]="JOINT_" + std::to_string(i) + "_ACT";
    }
}
  

// kine_para =[ [alpha], [a], [theta], [d] ]
void serialArm::SetGeometry(const Eigen::VectorXd& kine_para) {
    std::ostringstream strs;
    if (initialized_) {
        strs.str("");
        strs << GetName() << ":" << "has already been initialized in " << __FUNCTION__
                 << " and at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return;
    }

    if (kine_para.size() >= 4 * DoF_) {
      alpha_ = kine_para.segment(0, DoF_);
      a_ = kine_para.segment(DoF_, DoF_);
      theta_ = kine_para.segment(2 * DoF_, DoF_);
      d_ = kine_para.segment(3 * DoF_, DoF_);
      alpha_c_ = alpha_;
      a_c_ = a_;
      d_c_ = d_;
      theta_c_ = theta_;
      initialized_ = true;
    } else {
        strs.str("");
        strs << GetName() << ":" << "input parameters has dimension less than 4 * DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
    }
}

void serialArm::UpdateDH(const Eigen::VectorXd& q, 
                         Eigen::VectorXd& theta, Eigen::VectorXd& d) const {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class (depend on specific kinematic structure), in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
}

void serialArm::UpdateDH(const Eigen::VectorXd& orig_dh,
                         const Eigen::VectorXd& jnt,
                         Eigen::VectorXd& new_dh) const {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class (depend on specific kinematic structure), in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
}



int serialArm::JntToCart(const Eigen::VectorXd& q,
                       Pose& p) {
    std::ostringstream strs;
    Eigen::VectorXd a_tmp, alpha_tmp, d_tmp, theta_tmp;
    if (useCalibrated_) {  // if use calibrated parameters
       a_tmp = a_c_;
       alpha_tmp = alpha_c_;
       d_tmp = d_c_;
       theta_tmp = theta_c_;

       strs.str(""); 
       strs << GetName() << ", in FK, useCalibrated=" << useCalibrated_ << std::endl;
       LOG_INFO(strs);
    } else {
       a_tmp = a_;
       alpha_tmp = alpha_;
       d_tmp = d_;
       theta_tmp = theta_;
    }
    // update dh based upon joint feedback
    UpdateDH(q, theta_tmp, d_tmp);

    Frame tmp = defaultBaseOff_;
    for (size_t i=0; i < DoF_; i++) {
      tmp = tmp * Frame::DH_Craig1989(a_tmp[i],
                                      alpha_tmp[i],
                                      d_tmp[i],
                                      theta_tmp[i]);
    }

    // config flags
    std::vector<int>  branchFlags;
    //turn flags
    std::vector<int>  jointTurns;
    // get flags and turn from final theta and d including their initial values
    // plus joint displacement
    UpdateConfigTurn(theta_tmp, d_tmp, branchFlags, jointTurns);
    p.setFrame(tmp);
    p.setBranchFlags(branchFlags);
    p.setJointTurns(jointTurns);
    return 0;
}

int serialArm::JntToCart(const Eigen::VectorXd& q,
                     const Eigen::VectorXd& qdot,
                     Pose& p, Twist& v) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
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
    // need to compute the Jacobian
    Eigen::VectorXd a_tmp, alpha_tmp, d_tmp, theta_tmp;
    if (useCalibrated_) {  // if use calibrated parameters
       a_tmp = a_c_;
       alpha_tmp = alpha_c_;
       d_tmp = d_c_;
       theta_tmp = theta_c_;
    } else {
       a_tmp = a_;
       alpha_tmp = alpha_;
       d_tmp = d_;
       theta_tmp = theta_;
    }

    // update dh based upon joint feedback
    UpdateDH(q, theta_tmp, d_tmp);

    Eigen::VectorXd kine_para(4 * DoF_);
    kine_para.segment(0, DoF_) = alpha_tmp;
    kine_para.segment(DoF_, DoF_) = a_tmp;
    kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
    kine_para.segment(3 * DoF_, DoF_) = d_tmp;

    // Pose tmp_p;
    Eigen::MatrixXd Jp_t, Jp_r;
    int ret = CalcJacobian(kine_para, p, Jp_t, Jp_r);
    if (ret < 0) {
        return ret;
    }
    // picking submatrix of Jp_t and Jp_r, and then multiplying qdot
    // to obtain twist
    Eigen::MatrixXd Mt(3, DoF_), Mr(3, DoF_);
    if (PickSubJacobian(Jp_t, Jp_r, Mt, Mr)) {
      Eigen::Vector3d gvt = Mt * qdot ;
      Eigen::Vector3d gvr = Mr * qdot;
      v.setLinearVel(Vec(gvt(0), gvt(1), gvt(2)));
      v.setAngularVel(Vec(gvr(0), gvr(1), gvr(2)));
      return 0;
    }
    return -1;
}

int serialArm::JntToCart(const Eigen::VectorXd& q,
                     const Eigen::VectorXd& qdot,
                     const Eigen::VectorXd& qddot,
                     Pose& p, Twist& v, Twist& a) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
    return -1;
    
}

// nominal velocity IK
int serialArm::CartToJnt(const Pose& p, const Twist& v,
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
    // this analytic IK is using canonical robot model, for non-canonical 
    // model (e.g., model after calibration) requires using GI-based iteration
    // method
    if (useCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << "default CartToJnt function must using canonical kinematic"
               " model in" << __FUNCTION__
                << ", at line " << __LINE__  << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_DEFAULT_IK_NOT_CANO;
    }
    // first compute position IK into a std::vector, then convert to Eigen
    int ret = CartToJnt(p, q);
    if (ret < 0) {
      return ret;
    }
    if (qdot.size() != DoF_) {
      qdot.resize(DoF_);
    }
    // need to compute the Jacobian
    Eigen::VectorXd a_tmp, alpha_tmp, beta_tmp, d_tmp, theta_tmp;
    a_tmp = a_;
    alpha_tmp = alpha_;
    d_tmp = d_;
    theta_tmp = theta_;
    UpdateDH(q, theta_tmp, d_tmp);
    Eigen::VectorXd kine_para(4 * DoF_);
    kine_para.segment(0, DoF_) = alpha_tmp;
    kine_para.segment(DoF_, DoF_) = a_tmp;
    kine_para.segment(2 * DoF_, DoF_) = theta_tmp;
    kine_para.segment(3 * DoF_, DoF_) = d_tmp;

    Twist v1 = defaultBaseOff_.Inverse() * v;

    Pose tmp_p;
    Eigen::MatrixXd Jp_t, Jp_r;
    ret = CalcJacobian(kine_para, tmp_p, Jp_t, Jp_r, true);
    if (ret < 0) {
        return ret;
    }
    // picking submatrix of Jp_t and Jp_r, and then multiplying qdot
    // to obtain twist
    Eigen::MatrixXd M;
    Eigen::VectorXd b;
    Eigen::MatrixXd Js_t, Js_r;   
    PickSubJacobian(Jp_t, Jp_r, Js_t, Js_r, true);

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
    double det = M.determinant();
    if (fabs(det) < K_EPSILON) {
        strs.str("");
        strs << GetName() <<  " is singular, can not compute IK "
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_JACOBIAN_IK_SINGULAR;
    }
    Vec t_v = v1.getLinearVel();
    Vec t_w = v1.getAngularVel();
    double spdNorm = PickCartErr(t_v.ToEigenVec(),
                                 t_w.ToEigenVec(), 
                                  b, true);
    qdot = M.inverse() * b;
    return 0;
}

int serialArm::CartToJnt(const Pose& p, Eigen::VectorXd& q) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ": default position IK not implemented for serialArm in " << __FUNCTION__
     << " at line " << __LINE__ << std::endl;
  LOG_ALARM(strs);
  return -ERR_ROB_DEFAULT_IK_NOT_CANO;
}

double serialArm::PickCartErr(const Eigen::Vector3d& errT,
                              const Eigen::Vector3d& errR, 
                              Eigen::VectorXd& b,
                              bool reduction) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
    return -1.0;
}

void  serialArm::UpdateConfigTurn(const Eigen::VectorXd& theta,
                                  const Eigen::VectorXd& d,
                                  std::vector<int>& branchFlags,
                                  std::vector<int>& jointTurns) const {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
}

int serialArm::CartToJnt(const Pose& p, const Twist& v, const Twist& a,
             Eigen::VectorXd& q,
             Eigen::VectorXd& qdot,
             Eigen::VectorXd& qddot) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
  LOG_ALARM(strs);
  return -1;
}

// nominal Jacobian without considering base offset
int serialArm::CalcJacobian(const Eigen::VectorXd& kine_para,
            Pose &p,
            Eigen::MatrixXd& Jp_t,
            Eigen::MatrixXd& Jp_r,
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
    const int tol_size = 4 * DoF_;
    if (kine_para.size() == tol_size) {
       if (Jp_t.rows() !=3 || Jp_t.cols() != tol_size) {
         // if size is not match, we have to resize to the correct size
         Jp_t.resize(3, tol_size);
       }
       if (Jp_r.rows() !=3 || Jp_r.cols() != tol_size) {
         // if size is not match, we have to resize to the correct size
         Jp_r.resize(3, tol_size);
       }
       Frame bt; // default to identity if base jacobian
       if (world_jac) {
           bt = defaultBaseOff_;  // frame that represent transformation from base to tool
       } 
       // Note: base_off_c_ is not used because it is only used in calibration, but not in compensation.
       // in calibration, there might be offset between robot base frame and sensor measuring frame, but in 
       // compensation, the desired traj is w.r.t. robot base frame
       
       Eigen::VectorXd alpha_tmp(DoF_), a_tmp(DoF_),
                       d_tmp(DoF_), theta_tmp(DoF_), beta_tmp(DoF_);
       for (size_t i=0; i < DoF_; i++) {
          alpha_tmp[i] = kine_para[i];
          a_tmp[i] = kine_para[DoF_ + i];
          theta_tmp[i] = kine_para[2 * DoF_ + i];
          d_tmp[i] = kine_para[3 * DoF_ + i];
 
          // computing the linear part of the Jp
          Rotation r = bt.getRotation();
          Vec t = bt.getTranslation();
          Vec t_col2 = r.UnitX();
          Vec t_col1 = t * t_col2;
          Jp_t.col(4 * i) = t_col1.ToEigenVec();      // 1rd column is dalpha
          Jp_t.col(4 * i + 1) = t_col2.ToEigenVec();   // 2rd column is da
          
          Jp_r.col(4 * i) = t_col2.ToEigenVec();
          Eigen::Vector3d v(0,0,0);
          Jp_r.col(4 * i + 1) = v;
           // start computing Jacobian
          Rotation r1 = Rotation::RotX(alpha_tmp[i]);
          // combined trans. of alpha_i and a_i
          Frame f1(r1, Vec(a_tmp[i], 0, 0));
          
          // bt is frame up to end of alpha_i, a_i
          bt = bt * f1;
          r = bt.getRotation();
          t = bt.getTranslation();
          t_col2 = r.UnitZ();
          t_col1 = t * t_col2;
          Jp_t.col(4 * i + 2) = t_col1.ToEigenVec();  // 3rd column is dtheta_i
          Jp_t.col(4 * i + 3) = t_col2.ToEigenVec();  // 4th column is dd_i
          
          Jp_r.col(4 * i + 2) = t_col2.ToEigenVec();
          Jp_r.col(4 * i + 3) = v;

          Rotation r2  = Rotation::RotZ(theta_tmp[i]);
          Frame f2(r2, Vec(0, 0, d_tmp[i]));
          // bt is frame up to end of theta_i, d_i
          bt = bt * f2;
       }

       // config flags
       std::vector<int>  branchFlags;
       //turn flags
       std::vector<int>  jointTurns;
       // get flags and turn from final theta and d including their initial values
       // plus joint displacement
       UpdateConfigTurn(theta_tmp, d_tmp, branchFlags, jointTurns);

      p.setFrame(bt);
      p.setBranchFlags(branchFlags);
      p.setJointTurns(jointTurns);
    } else {
        strs.str("");
        strs << GetName() << ":" << "input kine_parameters has dimension not equal to 4 * DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_PARA_WRONG_DIM;
    }
    return 0;
}



bool serialArm::PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                                const Eigen::MatrixXd& Jp_r,
                                Eigen::MatrixXd& Js_t,
                                Eigen::MatrixXd& Js_r,
                                bool reduction) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
    return false;
}


int serialArm::CalcPassive(const Eigen::VectorXd& q,
                   const Pose& p,
                   Eigen::VectorXd& qpassive) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called for serialArm "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
    return -1;

  }



}


