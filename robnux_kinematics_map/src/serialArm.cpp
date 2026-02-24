#include "serialArm.hpp"
#include <common/pose.hpp>
#include <string>
namespace kinematics_lib {

serialArm::serialArm(const size_t DoF): BaseKinematicMap(DoF, DoF),
    alpha_(Eigen::VectorXd::Zero(DoF)),
    alpha_c_(alpha_),
    a_(Eigen::VectorXd::Zero(DoF)),
    a_c_(a_),
    beta_(Eigen::VectorXd::Zero(DoF)),
    beta_c_(beta_),
    d_(Eigen::VectorXd::Zero(DoF)),
    d_c_(d_),
    theta_(Eigen::VectorXd::Zero(DoF)),
    theta_c_(theta_),
    resetCache_(true) {
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
       for (size_t i=0; i < DoF_; i++) {
          alpha_(i) = kine_para(i);
          a_(i) = kine_para(DoF_ + i);
          theta_(i) = kine_para(2 * DoF_ + i);
          d_(i) = kine_para(3 * DoF_ + i);
       }
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
                         Eigen::VectorXd& theta, Eigen::VectorXd& d) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class (depend on specific kinematic structure), in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
}

void  serialArm::UpdateConfigTurn(const Eigen::VectorXd& theta,
                                  const Eigen::VectorXd &d,
                                  std::vector<int>& branchFlags,
                                  std::vector<int>& jointTurns) const {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
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
    }
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

int serialArm::JntToCart(const std::vector<double> &q,
                     const std::vector<double> &qdot,
                     const std::vector<double> &qddot,
                     Pose& p, Twist& v, Twist& a) {
    return 0;
    
}

// nominal velocity IK
int serialArm::CartToJnt(const Pose &p, const Twist &v,
              Eigen::VectorXd &q,
              Eigen::VectorXd &qdot) {
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
    std::vector<double> q_std;
    int ret = CartToJnt(p, q_std);
    if (ret < 0) {
      return ret;
    }
    // convert to Eigen
    StdVec2EigenVec(q_std, &q);
    if (qdot.size() != DoF_) {
      qdot.resize(DoF_);
    }
    // need to compute the Jacobian
    std::vector<double> a_tmp, alpha_tmp, beta_tmp, d_tmp, theta_tmp;
    a_tmp = a_;
    alpha_tmp = alpha_;
    beta_tmp = beta_;
    d_tmp = d_;
    theta_tmp = theta_;
    Eigen::VectorXd qv;
    StdVec2EigenVec(q_std, &qv);
    UpdateDH(qv, theta_tmp, d_tmp); 
    std::vector<double> kine_para;
    kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
    kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
    kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
    kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
    kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
    
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
    PickSubJacobian(Jp_t, Jp_r, &Js_t, &Js_r, true);

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
        std::ostringstream strs;
        strs << GetName() <<  " is singular, can not compute IK "
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        return -ERR_ROB_JACOBIAN_IK_SINGULAR;
    }
    Vec t_v = v1.getLinearVel();
    Vec t_w = v1.getAngularVel();
    double spdNorm = PickCartErr(t_v.ToEigenVec(),
                                 t_w.ToEigenVec(), 
                                  &b, true);
    Eigen::VectorXd qdot_tmp = M.inverse() * b;
    EigenVec2StdVec(qdot_tmp, qdot);
    return 0;
}

// position-only IK (default not implemented for generic serialArm)
int serialArm::CartToJnt(const Pose &p, std::vector<double> &q) {
  std::ostringstream strs;
  strs.str("");
  strs << GetName() << ": default position IK not implemented for serialArm in " << __FUNCTION__
     << " at line " << __LINE__ << std::endl;
  LOG_ALARM(strs);
  return -ERR_ROB_DEFAULT_IK_NOT_CANO;
}

double serialArm::PickCartErr(const Eigen::Vector3d &errT,
                                           const Eigen::Vector3d &errR, 
                                           Eigen::VectorXd *b,
                                           const bool reduction) {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
    return -1.0;
}

void  serialArm::UpdateConfigTurn(const std::vector<double> & theta,
                                  const std::vector<double> &d,
                                  std::vector<int>  *branchFlags,
                                  std::vector<int>  *jointTurns) const {
    std::ostringstream strs;
    strs.str("");
    strs << GetName() << ":" << "This function shouldn't be called, should be implemented in"
         << " children class, in "
         << __FUNCTION__ << " line " << __LINE__ << std::endl;
    LOG_ALARM(strs);
}

// these will be left to next/next week to do
int serialArm::CartToJnt(const Pose &p, const Twist& v, const Twist& a,
             Eigen::VectorXd &q,
             Eigen::VectorXd &qdot,
             Eigen::VectorXd &qddot) {
  return 0;
}

// nominal Jacobian without considering base offset
int serialArm::CalcJacobian(const std::vector<double> &kine_para,
            Pose &p,
            Eigen::MatrixXd & Jp_t,
            Eigen::MatrixXd & Jp_r,
            const bool reduction) {
  // Jp is 6 * kine_para_.size() matrix
    const int tol_size = 5 * DoF_;
    if (kine_para.size() == tol_size) {
       if (Jp_t.rows() !=3 || Jp_t.cols() != tol_size) {
         // if size is not match, we have to resize to the correct size
         Jp_t.resize(3, tol_size);
       }
       if (Jp_r.rows() !=3 || Jp_r.cols() != tol_size) {
         // if size is not match, we have to resize to the correct size
         Jp_r.resize(3, tol_size);
       }
       Frame bt; // default to identity if reduction true
       if (!reduction) {
           bt = defaultBaseOff_;  // frame that represent transformation from base to tool
       } 
       // Note: base_off_c_ is not used because it is only used in calibration, but not in compensation.
       // in calibration, there might be offset between robot base frame and sensor measuring frame, but in 
       // compensation, the desired traj is w.r.t. robot base frame
       
       std::vector<double> alpha_tmp(DoF_), a_tmp(DoF_),
                           d_tmp(DoF_), theta_tmp(DoF_), beta_tmp(DoF_);
       for (size_t i=0; i < DoF_; i++) {
          alpha_tmp[i] = kine_para[i];
          a_tmp[i] = kine_para[DoF_ + i];
          theta_tmp[i] = kine_para[2 * DoF_ + i];
          d_tmp[i] = kine_para[3 * DoF_ + i];
          beta_tmp[i] = kine_para[4 * DoF_ + i];
 
          // computing the linear part of the Jp
          Rotation r = bt.getRotation();
          Vec t = bt.getTranslation();
          Vec t_col2 = r.UnitX();
          Vec t_col1 = t * t_col2;
          Jp_t.col(5 * i) = t_col1.ToEigenVec();      // 1rd column is dalpha
          Jp_t.col(5 * i + 1) = t_col2.ToEigenVec();   // 2rd column is da
          
          Jp_r.col(5 * i) = t_col2.ToEigenVec();
          Eigen::Vector3d v(0,0,0);
          Jp_r.col(5 * i + 1) = v;
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
          Jp_t.col(5 * i + 2) = t_col1.ToEigenVec();  // 3rd column is dtheta_i
          Jp_t.col(5 * i + 3) = t_col2.ToEigenVec();  // 4th column is dd_i
          
          Jp_r.col(5 * i + 2) = t_col2.ToEigenVec();
          //Eigen::Vector3d v(0,0,0);
          Jp_r.col(5 * i + 3) = v;

          Rotation r2  = Rotation::RotZ(theta_tmp[i]);
          Frame f2(r2, Vec(0, 0, d_tmp[i]));
          // bt is frame up to end of theta_i, d_i
          bt = bt * f2;
          r = bt.getRotation();
          t = bt.getTranslation();
          t_col2 = r.UnitY();
          t_col1 = t * t_col2;
          Jp_t.col(5 * i + 4) = t_col1.ToEigenVec();  // 5rd column is dbeta_i
          Jp_r.col(5 * i + 4) = t_col2.ToEigenVec();
          Rotation r3  = Rotation::RotY(beta_tmp[i]);
          Frame f3(r3, Vec(0, 0, 0));
          bt = bt * f3;
       }

       // config flags
       std::vector<int>  branchFlags;
       //turn flags
       std::vector<int>  jointTurns;
       // get flags and turn from final theta and d including their initial values
       // plus joint displacement
       UpdateConfigTurn(theta_tmp, d_tmp, &branchFlags, &jointTurns);

      p.setFrame(bt);
      p.setBranchFlags(branchFlags);
      p.setJointTurns(jointTurns);
    } else {
        std::ostringstream strs;
        strs << GetName() << ":" << "input kine_parameters has dimension not equal to 5 * DoF in "
                   << __FUNCTION__ << " line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_INPUT_PARA_WRONG_DIM;
    }
    return 0;
}

int serialArm::CalibSanityCheck(const std::vector<double> &cal_DH) {
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
      double da = fabs(ina[k] - a_[k]);
      double limit_a = sanity_loose_coef_ * std::max(MAX_A_DIFF_REG, fabs(a_[k]) * MAX_DH_PERC / 100.0);
      if (da > limit_a) {
        strs << "";
        strs << GetName() << ":" << "Calib. santity check fails: a_c_ " << ina[k] << " diffs from a_ " << a_[k] 
        << " by " << da << " which greater than limit value " << limit_a << std::endl;
        LOG_ERROR(strs);
        return -2002;
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
      double dd = fabs(ind[k] - d_[k]);
      double limit_d = sanity_loose_coef_ * std::max(MAX_D_DIFF_REG, fabs(d_[k]) * MAX_DH_PERC / 100.0);
      if (dd > limit_d) {
        strs << "";
        strs << GetName() << ":" << "Calib. santity check fails: d_c_ " << ind[k] << " diffs from d_ " << d_[k] 
        << " by " << dd << " which greater than limit value " << limit_d << std::endl;
        LOG_ERROR(strs);
        return -2004;
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

double serialArm::CalibrateLaserMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              EigenDRef<Eigen::Matrix3d> * finalLaser2CartMap,
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) {
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!final_tool_offset || !final_base_offset) {
      strs.str("");
      strs << GetName() << ":" << "Input final_tool_offset pointer or final_base_offset is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
    }
    //! temporarily, we use default base offset
    *final_base_offset = init_base_offset;
    strs.str("");
    strs << GetName() << ":" << "before calib, init base = " << init_base_offset 
         << ", init tool =" << init_tool_offset << std::endl;
    strs << ", init LaserMatrix=" << laser2CartMap << std::endl;
    LOG_INFO(strs);
   //! temporarily, we use initial laser2cartmap
    *finalLaser2CartMap = laser2CartMap;
    size_t total_measures = laser_measure.cols();
    size_t total_jnts = qa_array.cols();
    size_t num_measures = 2 * total_measures / 3;
    if (num_measures < 2) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara laser calibration: need at least two samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }
    if (init_base_offset.rows() !=7) {
      strs.str(""); 
      strs << GetName() << ":" << "Input init_base_offset has wrong dimension,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    // either laser measures == jnt measures, or jnt measures = laser measures + 8
    bool sizeOK = (total_measures == total_jnts) || (total_measures + 8 == total_jnts);  
    if (total_measures != cart_measure.cols() || cart_measure.rows() < 3
        || !sizeOK || laser_measure.rows() <3) {
      strs.str("");
      strs << GetName() << ":" << "Laser calibration: number of measure ee coordinates"
              " and number of joint records does not match,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }


    // initialize iter alg parameters
    alg_.setParam( // num_measures - 1, 
                   decay_coef_ *  CALIB_RNN_STEPSIZE,
                   ridgeScale_, cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    bool eightPts = (total_measures + 8 == total_jnts);

    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                        d_tmp = d_c_, theta_tmp = theta_c_;
    std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                        d_old = d_tmp, theta_old = theta_tmp;
    // start with initial tool offset
    Eigen::Vector3d tool_tmp = init_tool_offset.segment(0, 3);
    Eigen::Vector3d tool_old = tool_tmp;
    
    // convert base_offset from trans, quat -> trans, EulerZYX
    Quaternion bq(init_base_offset(3), init_base_offset(4),
                  init_base_offset(5), init_base_offset(6));
    double init_yaw, init_pitch, init_roll;
    bq.GetEulerZYX(&init_yaw, &init_pitch, &init_roll);
    Eigen::VectorXd base_tmp(6);
    base_tmp(0) = init_base_offset(0);
    base_tmp(1) = init_base_offset(1);
    base_tmp(2) = init_base_offset(2);
    base_tmp(3) = init_roll;
    base_tmp(4) = init_pitch;
    base_tmp(5) = init_yaw;
    Eigen::VectorXd base_old  = base_tmp;
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
    Eigen::MatrixXd A((num_measures - 1) * 3 + 5, 6 + 5 * DoF_ + 12);
    Eigen::VectorXd b((num_measures - 1) * 3 + 5);
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    Eigen::MatrixXd first_J; // very first Jacobian
    Eigen::Vector3d first_p; // first cartesian vec
    while ((estimation_err >  MAX_CALIB_STOP_ERR &&
           previous_err - estimation_err > MAX_CALIB_MATCHING_ERR || !resetCache_) 
           && cur_iter < MAX_CALIB_OUTER_ITER
           //&& previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
           ) {
      // assign previous value
      if (resetCache_) {
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        tool_old = tool_tmp; base_old = base_tmp;
        previous_err = estimation_err;
      }
      previous_diff = current_diff;
      cur_iter++;

      // compute the base tranformation
      Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
      Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
      Rotation rb(Eub(2), Eub(1), Eub(0)); 
      Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
      Eigen::Vector3d col1(0, 0, 1);
      EulerDiff.col(2) = col1;
      Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
      EulerDiff.col(1) = col2;
      Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
      EulerDiff.col(0) = col3;
      Vec vb = Vec::FromEigenVec(pb);
      // Frame base(rb, vb);  // get the base transformation
      
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      estimation_err = 0;
      if (!eightPts) {
          A.resize((num_measures - 1) * 3, 6 + 5 * DoF_ + 12);
          b.resize((num_measures - 1) * 3);
      } else {
          strs.str("");
          strs << GetName() << ":" << " start computing 8pt same plane constraint jacobian " << std::endl;
          LOG_INFO(strs);
          A.resize((num_measures - 1) * 3 + 5, 6 + 5 * DoF_ + 12);
          b.resize((num_measures - 1) * 3 + 5);
          std::vector<Eigen::Vector3d> pp(8);
          std::vector<Eigen::MatrixXd> J(8);
          for (size_t i=total_measures; i < total_jnts; i++) {
            J[i-total_measures].resize(3, 6 + 5 * DoF_ + 3);
            UpdateDH(kine_para, qa_array.col(i), &tmp_para);
            Eigen::MatrixXd Jp_t, Jp_r;
            Pose p;
            // compute the expected values from known canonical kinematic parameters
            // i.e., not-calibrated parameter set
            int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
            if (ret < 0) {
              return ret;
            }
            // we get R_04, the rotation matrix between 0 and 4
            Rotation r_04 = p.getRotation();
            Vec  t_04 = p.getTranslation();
            Vec tool_offset(tool_tmp);
            Vec t_e = r_04 * tool_offset + t_04;
            Vec tf = rb * t_e + vb;
            pp[i - total_measures] = tf.ToEigenVec();
            Eigen::Matrix3d pb_tf;
            (vb - tf).ToHat(&pb_tf); 
            Eigen::MatrixXd J_block1 = Eigen::MatrixXd::Identity(3, 3);  // corresponding to delta pb
            Eigen::MatrixXd J_block2 = pb_tf * EulerDiff; // corresponding to delta Eub
        
            // next we compute   J0T, not including base, corresponding to delta p, delta pt
            Eigen::Matrix3d t_e_hat;
            t_e.ToHat(&t_e_hat);
            Eigen::MatrixXd J_block3 = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);  //corresponding to delta p
            Eigen::MatrixXd J_block4 = (rb * r_04).ToEigenMat();  // corresponding to delta pt
            Eigen::MatrixXd current_J(3, 6 + 5 * DoF_ + 3);
            current_J.block(0,0,3,3) = J_block1;
            current_J.block(0,3,3,3) = J_block2;
            current_J.block(0,6,3,5 * DoF_) = J_block3;
            current_J.block(0, 6 + 5 * DoF_, 3, 3) = J_block4;
            J[i - total_measures] = current_J;
          }
          Eigen::Vector3d dv1= pp[1] - pp[0];
          double norm_dv1 =dv1.norm();
          Eigen::Vector3d dv2= pp[2] - pp[1];
          double norm_dv2 =dv2.norm();
          Eigen::MatrixXd dJ1 = J[1] - J[0];
          Eigen::MatrixXd dJ2 = J[2] - J[1];
          // strs.str("");
          // computing the Jacobian
          for (size_t i=0; i<5; i++) {
             Eigen::Vector3d dv3 = pp[i+3] - pp[2];
             double norm_dv3 = dv3.norm();
             Eigen::MatrixXd dJ3 = J[i+3] - J[2];
             Eigen::Matrix3d AA;
             for (size_t j=0; j< 6 + 5 * DoF_ +3; j++) {
                 double sum = 0;
                 AA.col(0) = dJ1.col(j) / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dv3 / norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;;
                 AA.col(1) = dJ2.col(j) / norm_dv2;
                 AA.col(2) = dv3 /  norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dJ3.col(j) / norm_dv3;
                 sum += AA.determinant();
                 A((num_measures - 1) * 3 + i, j) = sum;
                 //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
             }
             A.block((num_measures - 1) * 3 + i, 9 + 5 * DoF_, 1, 9) = Eigen::MatrixXd::Zero(1,9);
             AA.col(0) = dv1 / norm_dv1;
             AA.col(1) = dv2 /  norm_dv2;
             AA.col(2) = dv3 / norm_dv3;
             b((num_measures - 1) * 3 + i) = - AA.determinant();
             //strs << "b(" << (num_measures - 1) * 3 + i <<")=" <<  b((num_measures - 1) * 3 + i) << std::endl;
          }
          LOG_INFO(strs);
      }
      
      for (size_t i=0; i < num_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_tmp);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb; 
        Eigen::Matrix3d pb_tf;
        (vb - tf).ToHat(&pb_tf); 
        Eigen::MatrixXd J_block1 = Eigen::MatrixXd::Identity(3, 3);  // corresponding to delta pb
        Eigen::MatrixXd J_block2 = pb_tf * EulerDiff; // corresponding to delta Eub
        
        // next we compute   J0T, not including base, corresponding to delta p, delta pt
        Eigen::Matrix3d t_e_hat;
        t_e.ToHat(&t_e_hat);
        Eigen::MatrixXd J_block3 = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);  //corresponding to delta p
        Eigen::MatrixXd J_block4 = (rb * r_04).ToEigenMat();  // corresponding to delta pt
        
        // now merge 4 j blocks into one jacobian matrix
        Eigen::MatrixXd current_J(3, 6 + 5 * DoF_ + 3);
        current_J.block(0,0,3,3) = J_block1;
        current_J.block(0,3,3,3) = J_block2;
        current_J.block(0,6,3,5 * DoF_) = J_block3;
        current_J.block(0, 6 + 5 * DoF_, 3, 3) = J_block4;

        if (i==0) {
            first_J = current_J;
            first_p = tf.ToEigenVec();
        } else {
           //Eigen::MatrixXd cur_J = Jp_t - t_e_hat * Jp_r;  // current Jacobian
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           Eigen::Vector3d dls1 = laser_measure.col(i) - laser_measure.col(0);
           Eigen::Vector3d dls = (*finalLaser2CartMap) * dls1;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           estimation_err += av_error;
           A.block((i - 1) * 3, 0, 3, 6 + 5 * DoF_ + 3) = current_J - first_J;
           Eigen::Vector3d zeroVec;
           zeroVec.setZero();
           A.block((i - 1) * 3, 9 + 5 * DoF_, 1, 3) = -dls1.transpose();
           A.block((i - 1) * 3, 9 + 5 * DoF_ + 3, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3, 9 + 5 * DoF_ + 6, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_, 1, 3) =  zeroVec.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_ + 3, 1, 3) =  -dls1.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_ + 6, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_, 1, 3) =  zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_ + 3, 1, 3) =   zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_ + 6, 1, 3) =  -dls1.transpose();
           b.block((i - 1) * 3, 0, 3, 1) = epsi;
        }
      }


      // average estimation error for a single measurement
      estimation_err  /= double(num_measures - 1) ;
      current_diff = std::min(previous_err - estimation_err,
               std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through laser sensor, iteration no. = " << cur_iter << std::endl;
      LOG_INFO(strs);
      Eigen::VectorXd delta_p_old;
      if (resetCache_) {
        size_t numParam = A.cols();
        delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
      }
      if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
         if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
         }
         delta_p_old_cache_ = delta_p_old;
         UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, finalLaser2CartMap);
         // reset estimation_err. to enforce go to next iteration
         //estimation_err = previous_err - 1.02 * MAX_CALIB_MATCHING_ERR;
         resetCache_ = false;
      } else {
         UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, finalLaser2CartMap);
         resetCache_ = true;
      }
    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs<< GetName() << ":" << "In laser calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                 << "With estimation error " << estimation_err
                 << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR) {
        strs.str("");
        strs<< GetName() << ":"  << "In Laser calib: Iteration reaches estimation_err " << estimation_err
                 << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR) {
        strs.str("");
        strs << GetName() << ":" << "In Laser calib: Iteration reaches err diff " << previous_err - estimation_err
                 << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }

    alpha_c_ = alpha_old;
    a_c_ = a_old;
    theta_c_ = theta_old;
    d_c_ = d_old;
    beta_c_ = beta_old;
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "tool = " << tool_old << std::endl;
    strs << "base = " << base_old << std::endl;
    strs << "final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
    (*final_tool_offset)(0) = tool_old(0);
    (*final_tool_offset)(1) = tool_old(1);
    (*final_tool_offset)(2) = tool_old(2);
    // convert base_old from EulerZYX to Quaternion
    Quaternion q0;
    // base_old orientation is in the order of (roll, pitch, yaw)
    // however, yaw is angle about z-axis, pitch is angle about y-axis, and 
    // roll is about x axis
    q0.SetEulerZYX(base_old(5), base_old(4), base_old(3));
    (*final_base_offset)(0) = base_old(0);
    (*final_base_offset)(1) = base_old(1);
    (*final_base_offset)(2) = base_old(2);
    (*final_base_offset)(3) = q0.w();
    (*final_base_offset)(4) = q0.x();
    (*final_base_offset)(5) = q0.y();
    (*final_base_offset)(6) = q0.z();
    // Note: base_off_c_ is not used because it is only used in calibration, but not in compensation
    // in calibration, there might be offset between robot base frame and sensor measuring frame, but in 
    // compensation, the desired traj is w.r.t. robot base frame

    /////// base_off_c_.setTranslation(Vec(base_old(0), base_old(1), base_old(2)));
    /////// base_off_c_.setQuaternion(q0);
    isDHCalibrated_ = true;

    // last step using another half of measurements to verify that calibrated DHs
    // are going to shrink the errors
    // compute the base tranformation
    Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
    Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
    Rotation rb(Eub(2), Eub(1), Eub(0));
    Vec vb = Vec::FromEigenVec(pb);
    // Frame base(rb, vb);  // get the base transformation 

    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    double orig_err = 0, comp_err=0;
    for (size_t i=num_measures; i < total_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_old);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb;

        if (i==num_measures) {
            first_p = tf.ToEigenVec();
            Rotation rf = rb * r_04;
            double yaw1, pitch1, roll1;
            rf.GetEulerZYX(&yaw1, &pitch1, &roll1);
            strs.str("");
            strs << GetName() << ":" << "after compensation, yaw= " << yaw1 << ", pitch=" << pitch1
                 << ", roll=" << roll1 << std::endl;
            LOG_INFO(strs);
        } else {
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::Vector3d dls = (*finalLaser2CartMap) * 
                   (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::Vector3d dls1 = laser2CartMap * (laser_measure.col(i) -
                    laser_measure.col(num_measures));//dls;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           comp_err += av_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(num_measures);
           Eigen::Vector3d dc = dcart.block(0,0,3,1);
           epsi = dls1 - dc;
           av_error = epsi.norm();
           // accu_error = av_error * av_error;
           orig_err += av_error;
        }
    }
    double avg_orig_err= orig_err / (total_measures - num_measures - 1);
    double avg_comp_err = comp_err / (total_measures - num_measures - 1);
    double overall_comp_err = (previous_err * 2.0 + avg_comp_err) / 3.0;  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs<< GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        // return -overall_comp_err;
    }
    return overall_comp_err;
}

double serialArm::CalibrateLaserCombMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap_l,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap_r,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              const int secondset_start_index,
              EigenDRef<Eigen::Matrix3d> *finalLaser2CartMap_l,
              EigenDRef<Eigen::Matrix3d> *finalLaser2CartMap_r,
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) {
   // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!final_tool_offset || !final_base_offset) {
      strs.str("");
      strs << GetName() << ":" << "Input final_tool_offset pointer or final_base_offset is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
    }
    //! temporarily, we use default base offset
    *final_base_offset = init_base_offset;
    strs.str("");
    strs << GetName() << ":" << "before calib, init base = " << init_base_offset 
         << ", init tool =" << init_tool_offset << std::endl;
    strs << ", init LaserMatrix_l=" << laser2CartMap_l << std::endl;
    strs << ", init LaserMatrix_r=" << laser2CartMap_r << std::endl;
    LOG_INFO(strs);
   //! temporarily, we use initial laser2cartmap
    *finalLaser2CartMap_l = laser2CartMap_l;
    *finalLaser2CartMap_r = laser2CartMap_r;
    size_t total_measures = qa_array.cols();
    size_t firstHalfMeasures = secondset_start_index;
    size_t secondHalfMeasures = total_measures - firstHalfMeasures;
    size_t num_measures_l = 2 * firstHalfMeasures / 3;
    size_t num_measures_r = 2 * secondHalfMeasures / 3;
    size_t num_measures = num_measures_l + num_measures_r;
    if (num_measures_l < 2 || num_measures_r < 2) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara laser calibration: need at least two samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }
    if (init_base_offset.rows() !=7) {
      strs << GetName() << ":" << "Input init_base_offset has wrong dimension,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    if (total_measures != cart_measure.cols() || cart_measure.rows() < 3
        || total_measures != laser_measure.cols() || laser_measure.rows() <3) {
      strs.str("");
      strs << GetName() << ":" << "Scara calibration: number of measure ee coordinates"
              " and number of joint records does not match,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }

    // initialize iter alg parameters
    alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                  ridgeScale_,
                  cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                        d_tmp = d_c_, theta_tmp = theta_c_;
    std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                        d_old = d_tmp, theta_old = theta_tmp;
    // start with initial tool offset
    Eigen::Vector3d tool_tmp = init_tool_offset.segment(0, 3);
    Eigen::Vector3d tool_old = tool_tmp;
    
    // convert base_offset from trans, quat -> trans, EulerZYX
    Quaternion bq(init_base_offset(3), init_base_offset(4),
                  init_base_offset(5), init_base_offset(6));
    double init_yaw, init_pitch, init_roll;
    bq.GetEulerZYX(&init_yaw, &init_pitch, &init_roll);
    Eigen::VectorXd base_tmp(6);
    base_tmp(0) = init_base_offset(0);
    base_tmp(1) = init_base_offset(1);
    base_tmp(2) = init_base_offset(2);
    base_tmp(3) = init_roll;
    base_tmp(4) = init_pitch;
    base_tmp(5) = init_yaw;
    Eigen::VectorXd base_old  = base_tmp;
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix_L  transform * dlaser -> dcart
    // and another 9 for affine matrix_R transform
    Eigen::MatrixXd A((num_measures - 2) * 3, 6 + 5 * DoF_ + 21);
    Eigen::VectorXd b((num_measures - 2) * 3);
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    Eigen::MatrixXd first_J; // very first Jacobian
    Eigen::Vector3d first_p; // first cartesian vec
    while ((estimation_err >  MAX_CALIB_STOP_ERR &&
           previous_err - estimation_err > MAX_CALIB_MATCHING_ERR || !resetCache_)
           && cur_iter < MAX_CALIB_OUTER_ITER
           //&& previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
           ) {
      // assign previous value
      if (resetCache_) {
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        tool_old = tool_tmp; base_old = base_tmp;
        previous_err = estimation_err;
      }
      previous_diff = current_diff;
      cur_iter++;

      // compute the base tranformation
      Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
      Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
      Rotation rb(Eub(2), Eub(1), Eub(0)); 
      Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
      Eigen::Vector3d col1(0, 0, 1);
      EulerDiff.col(2) = col1;
      Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
      EulerDiff.col(1) = col2;
      Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
      EulerDiff.col(0) = col3;
      Vec vb = Vec::FromEigenVec(pb);
      // Frame base(rb, vb);  // get the base transformation
      
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      estimation_err = 0;
      A.resize((num_measures - 2) * 3, 6 + 5 * DoF_ + 21);
      for (size_t i=0; i < num_measures_l; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_tmp);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb; 
        Eigen::Matrix3d pb_tf;
        (vb - tf).ToHat(&pb_tf); 
        Eigen::MatrixXd J_block1 = Eigen::MatrixXd::Identity(3, 3);  // corresponding to delta pb
        Eigen::MatrixXd J_block2 = pb_tf * EulerDiff; // corresponding to delta Eub
        
        // next we compute   J0T, not including base, corresponding to delta p, delta pt
        Eigen::Matrix3d t_e_hat;
        t_e.ToHat(&t_e_hat);
        Eigen::MatrixXd J_block3 = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);  //corresponding to delta p
        Eigen::MatrixXd J_block4 = (rb * r_04).ToEigenMat();  // corresponding to delta pt
        
        // now merge 4 j blocks into one jacobian matrix
        Eigen::MatrixXd current_J(3, 6 + 5 * DoF_ + 3);
        current_J.block(0,0,3,3) = J_block1;
        current_J.block(0,3,3,3) = J_block2;
        current_J.block(0,6,3,5 * DoF_) = J_block3;
        current_J.block(0, 6 + 5 * DoF_, 3, 3) = J_block4;

        if (i==0) {
            first_J = current_J;
            first_p = tf.ToEigenVec();
        } else {
           //Eigen::MatrixXd cur_J = Jp_t - t_e_hat * Jp_r;  // current Jacobian
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           Eigen::Vector3d dls1 = laser_measure.col(i) - laser_measure.col(0);
           Eigen::Vector3d dls = (*finalLaser2CartMap_l) * dls1;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           estimation_err += av_error;
           A.block((i - 1) * 3, 0, 3, 6 + 5 * DoF_ + 3) = current_J - first_J;
           Eigen::Vector3d zeroVec;
           zeroVec.setZero();
           A.block((i - 1) * 3, 9 + 5 * DoF_, 1, 3) = -dls1.transpose();
           A.block((i - 1) * 3, 9 + 5 * DoF_ + 3, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3, 9 + 5 * DoF_ + 6, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_, 1, 3) =  zeroVec.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_ + 3, 1, 3) =  -dls1.transpose();
           A.block((i - 1) * 3 + 1, 9 + 5 * DoF_ + 6, 1, 3) = zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_, 1, 3) =  zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_ + 3, 1, 3) =   zeroVec.transpose();
           A.block((i - 1) * 3 + 2, 9 + 5 * DoF_ + 6, 1, 3) =  -dls1.transpose();
           A.block((i - 1) * 3, 9 + 5 * DoF_ + 9, 3, 9) = Eigen::MatrixXd::Zero(3, 9);
           b.block((i - 1) * 3, 0, 3, 1) = epsi;
        }
      }
      
      size_t start_row_block = num_measures_l - 1;
      for (size_t i=firstHalfMeasures; i < firstHalfMeasures + num_measures_r; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_tmp);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb;
        Eigen::Matrix3d pb_tf;
        (vb - tf).ToHat(&pb_tf); 
        Eigen::MatrixXd J_block1 = Eigen::MatrixXd::Identity(3, 3);  // corresponding to delta pb
        Eigen::MatrixXd J_block2 = pb_tf * EulerDiff; // corresponding to delta Eub
        
        // next we compute   J0T, not including base, corresponding to delta p, delta pt
        Eigen::Matrix3d t_e_hat;
        t_e.ToHat(&t_e_hat);
        Eigen::MatrixXd J_block3 = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);  //corresponding to delta p
        Eigen::MatrixXd J_block4 = (rb * r_04).ToEigenMat();  // corresponding to delta pt
        
        // now merge 4 j blocks into one jacobian matrix
        Eigen::MatrixXd current_J(3, 6 + 5 * DoF_ + 3);
        current_J.block(0,0,3,3) = J_block1;
        current_J.block(0,3,3,3) = J_block2;
        current_J.block(0,6,3,5 * DoF_) = J_block3;
        current_J.block(0, 6 + 5 * DoF_, 3, 3) = J_block4;

        if (i==firstHalfMeasures) {
            first_J = current_J;
            first_p = tf.ToEigenVec();
        } else {
           //Eigen::MatrixXd cur_J = Jp_t - t_e_hat * Jp_r;  // current Jacobian
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           Eigen::Vector3d dls1 = laser_measure.col(i) - laser_measure.col(firstHalfMeasures);
           Eigen::Vector3d dls = (*finalLaser2CartMap_r) * dls1;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           estimation_err += av_error;
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3, 0, 3, 6 + 5 * DoF_ + 3) = current_J - first_J;
           Eigen::Vector3d zeroVec;
           zeroVec.setZero();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3,9 + 5 * DoF_, 3, 9 ) = Eigen::MatrixXd::Zero(3, 9);
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3, 9 + 5 * DoF_ + 9, 1, 3) = -dls1.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3, 9 + 5 * DoF_ + 12, 1, 3) = zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3, 9 + 5 * DoF_ + 15, 1, 3) = zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 1, 9 + 5 * DoF_ + 9, 1, 3) =  zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 1, 9 + 5 * DoF_ + 12, 1, 3) =  -dls1.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 1, 9 + 5 * DoF_ + 15, 1, 3) = zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 2, 9 + 5 * DoF_, 9, 3) =  zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 2, 9 + 5 * DoF_ + 12, 1, 3) =   zeroVec.transpose();
           A.block((start_row_block + i - firstHalfMeasures - 1) * 3 + 2, 9 + 5 * DoF_ + 15, 1, 3) =  -dls1.transpose();
           b.block((start_row_block + i - firstHalfMeasures - 1) * 3, 0, 3, 1) = epsi;
        }
      }
      // average estimation error for a single measurement
      estimation_err  /= double(num_measures - 2) ;
      current_diff = std::min(previous_err - estimation_err,
               std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through laser sensor, iteration no. = " << cur_iter << std::endl;
      LOG_INFO(strs);
    
      Eigen::VectorXd delta_p_old;
      if (resetCache_) {
        size_t numParam = A.cols();
        delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
      }
      if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
         if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
         }
         delta_p_old_cache_ = delta_p_old;
         UpdateDH2(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, finalLaser2CartMap_l, finalLaser2CartMap_r);
         // reset estimation_err. to enforce go to next iteration
         //estimation_err = previous_err - 1.02 * MAX_CALIB_MATCHING_ERR;
         resetCache_ = false;
      } else {
         UpdateDH2(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, finalLaser2CartMap_l, finalLaser2CartMap_r);
         resetCache_ = true;
      }
      
    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs << GetName() << ":" << "In laser calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                 << "With estimation error " << estimation_err
                 << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR) {
        strs.str("");
        strs << GetName() << ":" << "In Laser calib: Iteration reaches estimation_err " << estimation_err
                 << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR) {
        strs.str("");
        strs << GetName() << ":" << "In Laser calib: Iteration reaches err diff " << previous_err - estimation_err
                 << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    alpha_c_ = alpha_old;
    a_c_ = a_old;
    theta_c_ = theta_old;
    d_c_ = d_old;
    beta_c_ = beta_old;
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "tool = " << tool_old << std::endl;
    strs << "base = " << base_old << std::endl;
    strs << "final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
    (*final_tool_offset)(0) = tool_old(0);
    (*final_tool_offset)(1) = tool_old(1);
    (*final_tool_offset)(2) = tool_old(2);
    // convert base_old from EulerZYX to Quaternion
    Quaternion q0;
    // base_old orientation is in the order of (roll, pitch, yaw)
    // however, yaw is angle about z-axis, pitch is angle about y-axis, and 
    // roll is about x axis
    q0.SetEulerZYX(base_old(5), base_old(4), base_old(3));
    (*final_base_offset)(0) = base_old(0);
    (*final_base_offset)(1) = base_old(1);
    (*final_base_offset)(2) = base_old(2);
    (*final_base_offset)(3) = q0.w();
    (*final_base_offset)(4) = q0.x();
    (*final_base_offset)(5) = q0.y();
    (*final_base_offset)(6) = q0.z();
    // Note: base_off_c_ is not used because it is only used in calibration, but not in compensation
    // in calibration, there might be offset between robot base frame and sensor measuring frame, but in 
    // compensation, the desired traj is w.r.t. robot base frame

    /////// base_off_c_.setTranslation(Vec(base_old(0), base_old(1), base_old(2)));
    /////// base_off_c_.setQuaternion(q0);
    isDHCalibrated_ = true;

    // last step using another half of measurements to verify that calibrated DHs
    // are going to shrink the errors
    // compute the base tranformation
    Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
    Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
    Rotation rb(Eub(2), Eub(1), Eub(0));
    Vec vb = Vec::FromEigenVec(pb);
    // Frame base(rb, vb);  // get the base transformation 

    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    double orig_err = 0, comp_err=0;
    for (size_t i=num_measures_l; i < firstHalfMeasures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_old);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb;

        if (i==num_measures_l) {
            first_p = tf.ToEigenVec();
            Rotation rf = rb * r_04;
            double yaw1, pitch1, roll1;
            rf.GetEulerZYX(&yaw1, &pitch1, &roll1);
            strs.str("");
            strs << GetName() << ":" << "first set of data: after compensation, yaw= " << yaw1 << ", pitch=" << pitch1
                 << ", roll=" << roll1 << std::endl;
            LOG_INFO(strs);
        } else {
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::Vector3d dls = (*finalLaser2CartMap_l) * 
                   (laser_measure.col(i) - laser_measure.col(num_measures_l));
           Eigen::Vector3d dls1 = laser2CartMap_l * (laser_measure.col(i) -
                    laser_measure.col(num_measures_l));//dls;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           comp_err += av_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(num_measures_l);
           Eigen::Vector3d dc = dcart.block(0,0,3,1);
           epsi = dls1 - dc;
           av_error = epsi.norm();
           // accu_error = av_error * av_error;
           orig_err += av_error;
        }
    }
    for (size_t i=firstHalfMeasures + num_measures_r; i < total_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_old);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb;

        if (i==firstHalfMeasures + num_measures_r) {
            first_p = tf.ToEigenVec();
            Rotation rf = rb * r_04;
            double yaw1, pitch1, roll1;
            rf.GetEulerZYX(&yaw1, &pitch1, &roll1);
            strs.str("");
            strs << GetName() << ":" << "after compensation, yaw= " << yaw1 << ", pitch=" << pitch1
                 << ", roll=" << roll1 << std::endl;
            LOG_INFO(strs);
        } else {
           Eigen::Vector3d dp =  tf.ToEigenVec() - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::Vector3d dls = (*finalLaser2CartMap_r) * 
                   (laser_measure.col(i) - laser_measure.col(firstHalfMeasures + num_measures_r));
           Eigen::Vector3d dls1 = laser2CartMap_r * (laser_measure.col(i) -
                    laser_measure.col(firstHalfMeasures + num_measures_r));//dls;
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           comp_err += av_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(firstHalfMeasures + num_measures_r);
           Eigen::Vector3d dc = dcart.block(0,0,3,1);
           epsi = dls1 - dc;
           av_error = epsi.norm();
           // accu_error = av_error * av_error;
           orig_err += av_error;
        }
    }
    double avg_orig_err= orig_err / (total_measures - num_measures - 2);
    double avg_comp_err = comp_err / (total_measures - num_measures - 2);
    double overall_comp_err = (previous_err * 2.0 + avg_comp_err) / 3.0;  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        //return -overall_comp_err;
    }
    return overall_comp_err;
}


// function to verify if calibrated model is good or not
Eigen::VectorXd serialArm::VerifyCalibrate(
              const EigenDRef<Eigen::VectorXd> &base, // base offset used in measuring
              const EigenDRef<Eigen::VectorXd> &tool,  // tool offset used in measuring
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &measureMents,
              const EigenDRef<Eigen::Matrix3d> &laserMat,
              EigenDRef<Eigen::MatrixXd> *calib_cart,
              EigenDRef<Eigen::MatrixXd> *ucalib_err,
              EigenDRef<Eigen::MatrixXd> *calib_err) {
    std::ostringstream strs;
    Eigen::VectorXd outData(2);
    // initialize the outData
    outData(0) = 0;  // no error
    outData(1) = 0;  // no varify calibration data
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    if (!initialized_ || !calib_cart || !ucalib_err || !calib_err) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);

      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return  outData;
    }
    size_t total_measures = qa_array.cols();
    if (total_measures < 2) {
      strs.str("");
      strs << GetName() << ":" << "No enough measurement points "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);

      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return  outData;
    }
    size_t measDoF = measureMents.rows();
    calib_cart->resize(measDoF, total_measures);
    ucalib_err->resize(measDoF + 1, total_measures-1);  // last row is error norm
    calib_err->resize(measDoF + 1, total_measures-1);  // last row is error norm
    if (total_measures != measureMents.cols() || measDoF > 3) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara calibration: number of measure ee coordinates"
              " and number of joint records does not match, or meas has dimension > 3"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      return outData;
    }
   
    Vec vb(base(0), base(1), base(2));
    Quaternion quat(base(3), base(4), base(5), base(6));
    Rotation rb(quat);


    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    Vec tool_offset(tool.segment(0, 3));
    Eigen::VectorXd first_p; // first cartesian vec
    double orig_err = 0, comp_err=0;
    for (size_t i=0; i < total_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           outData(0) = ret;
           return outData;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec te = r_04 * tool_offset + t_04;
        Vec tf = rb * te + vb;
        calib_cart->col(i) = tf.ToEigenVec().block(0,0,measDoF,1);
        if (i==0) {
            first_p = tf.ToEigenVec().block(0,0,measDoF,1);
        } else {
           Eigen::VectorXd dp =  tf.ToEigenVec().block(0,0,measDoF,1) - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::VectorXd dls = laserMat.block(0, 0, measDoF, measDoF) * (measureMents.col(i) - measureMents.col(0));
           // laser rel. displacement  - robot reported displacement 
           Eigen::VectorXd epsi = dls - dp;
           double av_error = epsi.norm();
           Eigen::VectorXd c_err(measDoF+1,1);
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           calib_err->col(i-1) = c_err;
           // double accu_error = av_error * av_error;
           comp_err += av_error; // accu_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(0);
           Eigen::Vector3d dc = dcart.block(0,0,measDoF,1);
           epsi = dls - dc;
           av_error = epsi.norm();
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           ucalib_err->col(i-1) = c_err;
           // accu_error = av_error * av_error;
           orig_err += av_error; //accu_error;
        }
    }
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << orig_err / (total_measures - 1)
         << ", after comp. the error = " << comp_err / (total_measures - 1) << std::endl;
    LOG_INFO(strs);

    outData(0) = orig_err / (total_measures - 1);
    if (comp_err >= orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        outData(1) = -comp_err / (total_measures - 1);
    } else {
      outData(1) = comp_err / (total_measures - 1);
    }
    return outData;
}

Eigen::VectorXd serialArm::VerifyCalibrateComb(
              const EigenDRef<Eigen::VectorXd> &base, // base offset used in measuring
              const EigenDRef<Eigen::VectorXd> &tool,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &measureMents,
              const int second_start_index,
              const EigenDRef<Eigen::Matrix3d> &laserMat_l,   // 3 by 3 laser measure matrix
              const EigenDRef<Eigen::Matrix3d> &laserMat_r,   // 3 by 3 laser measure matrix
              EigenDRef<Eigen::MatrixXd> *calib_cart,
              EigenDRef<Eigen::MatrixXd> *ucalib_err,
              EigenDRef<Eigen::MatrixXd> *calib_err) {
    std::ostringstream strs;
    Eigen::VectorXd outData(2);
    // initialize the outData
    outData(0) = 0;  // no error
    outData(1) = 0;  // no varify calibration data
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    if (!initialized_ || !calib_cart || !ucalib_err || !calib_err) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);

      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return  outData;
    }
    size_t total_measures = qa_array.cols();
    size_t firstHalfMeasures = second_start_index;
    size_t secondHalfMeasures = total_measures - firstHalfMeasures;
    if (total_measures < 2) {
      strs.str("");
      strs << GetName() << ":" << "No enough measurement points "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);

      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return  outData;
    }
    size_t measDoF = measureMents.rows();
    calib_cart->resize(measDoF, total_measures);
    ucalib_err->resize(measDoF + 1, total_measures-2);  // last row is error norm
    calib_err->resize(measDoF + 1, total_measures-2);  // last row is error norm
    if (total_measures != measureMents.cols() || measDoF > 3) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara calibration: number of measure ee coordinates"
              " and number of joint records does not match, or meas has dimension > 3"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      return outData;
    }
   
    Vec vb(base(0), base(1), base(2));
    Quaternion quat(base(3), base(4), base(5), base(6));
    Rotation rb(quat);


    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    Vec tool_offset(tool.segment(0, 3));
    Eigen::VectorXd first_p; // first cartesian vec
    double orig_err = 0, comp_err=0;
    for (size_t i=0; i < firstHalfMeasures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           outData(0) = ret;
           return outData;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec te = r_04 * tool_offset + t_04;
        Vec tf = rb * te + vb;
        calib_cart->col(i) = tf.ToEigenVec().block(0,0,measDoF,1);
        if (i==0) {
            first_p = tf.ToEigenVec().block(0,0,measDoF,1);
        } else {
           Eigen::VectorXd dp =  tf.ToEigenVec().block(0,0,measDoF,1) - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::VectorXd dls = laserMat_l.block(0, 0, measDoF, measDoF) * (measureMents.col(i) - measureMents.col(0));
           // laser rel. displacement  - robot reported displacement 
           Eigen::VectorXd epsi = dls - dp;
           double av_error = epsi.norm();
           Eigen::VectorXd c_err(measDoF+1,1);
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           calib_err->col(i-1) = c_err;
           // double accu_error = av_error * av_error;
           comp_err += av_error; // accu_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(0);
           Eigen::Vector3d dc = dcart.block(0,0,measDoF,1);
           epsi = dls - dc;
           av_error = epsi.norm();
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           ucalib_err->col(i-1) = c_err;
           // accu_error = av_error * av_error;
           orig_err += av_error; //accu_error;
        }
    }

    for (size_t i=firstHalfMeasures; i < total_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           outData(0) = ret;
           return outData;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec te = r_04 * tool_offset + t_04;
        Vec tf = rb * te + vb;
        calib_cart->col(i) = tf.ToEigenVec().block(0,0,measDoF,1);
        if (i==firstHalfMeasures) {
            first_p = tf.ToEigenVec().block(0,0,measDoF,1);
        } else {
           Eigen::VectorXd dp =  tf.ToEigenVec().block(0,0,measDoF,1) - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::VectorXd dls = laserMat_r.block(0, 0, measDoF, measDoF) * (measureMents.col(i) - measureMents.col(firstHalfMeasures));
           // laser rel. displacement  - robot reported displacement 
           Eigen::VectorXd epsi = dls - dp;
           double av_error = epsi.norm();
           Eigen::VectorXd c_err(measDoF+1,1);
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           calib_err->col(i-2) = c_err;
           // double accu_error = av_error * av_error;
           comp_err += av_error; // accu_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(firstHalfMeasures);
           Eigen::Vector3d dc = dcart.block(0,0,measDoF,1);
           epsi = dls - dc;
           av_error = epsi.norm();
           c_err.block(0,0,measDoF,1) = epsi;
           c_err(measDoF,0) = av_error;
           ucalib_err->col(i-2) = c_err;
           // accu_error = av_error * av_error;
           orig_err += av_error; //accu_error;
        }
    }
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << orig_err / (total_measures - 2)
         << ", after comp. the error = " << comp_err / (total_measures - 2) << std::endl;
    LOG_INFO(strs);

    outData(0) = orig_err / (total_measures - 2);
    if (comp_err >= orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        outData(1) = -comp_err / (total_measures - 2);
    } else {
      outData(1) = comp_err / (total_measures - 2);
    }
    return outData;
}

// Calibration of robot tool
double serialArm::CalibrateTool(
        const EigenDRef<Eigen::MatrixXd> &qa_array,   // joint array measurement
        const EigenDRef<Eigen::VectorXd> &displace_array,  // distance sensor measurement
        const EigenDRef<Eigen::Vector3d> &init_normal,   // initial normal of sensor top surface
        EigenDRef<Eigen::VectorXd> *final_tool_offset      // output of the tool offset
        ) {
   
     std::ostringstream strs;
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Robot geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!final_tool_offset) {
      strs.str("");
      strs << GetName() << ":" << "Input final_tool_offset pointer is null"
                << " so can not do tool calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
    }

    size_t num_jnt_measures = qa_array.cols();
    size_t num_dist_measures = displace_array.size();
    if (num_jnt_measures != num_dist_measures || num_jnt_measures < 3) {
      strs.str("");
      strs << GetName() << ":" << "input data dimension is not matching or too few measures"
                << " so can not do  tool calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }

    strs.str("");
    strs << GetName() << ":" << " init_normal =" << init_normal << std::endl;
    LOG_INFO(strs);
    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp, alpha_tmp, beta_tmp ,
                        d_tmp, theta_tmp ;
    if (useCalibrated_) {
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
    
    // step 1, define meature matrix A.rows(i) = v^T(R_j - R1)
    //  b(i) = displace(j)  - displace(1) - v^T(p_j -p_1)
    Eigen::MatrixXd A(num_jnt_measures - 1, 3);
    Eigen::VectorXd b(num_jnt_measures - 1);
   
    // preparing compute FK to obtain R_j and P_j      
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
    kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
    kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
    kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
    kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());

       
    // pp and J vector are for computing the items in A matrix, and b vector
    Eigen::Vector3d p1;
    Eigen::Matrix3d R1;

    for (size_t i=0; i < num_jnt_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << " calJacobian returns error code " << ret << " in function " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;
        }
        Vec  tf = p.getTranslation();
        Eigen::Vector3d tf1= tf.ToEigenVec();
        Rotation r = p.getRotation();
        Eigen::Matrix3d tr1 = r.ToEigenMat();
        if (i == 0) {
          p1 = tf1;
          R1 = tr1;
        } else {
          // computing the Coefficient matrix
          A.block((i-1), 0, 1, 3) = init_normal.transpose() * (tr1 - R1); // dbase
          b(i - 1) = displace_array(i) - displace_array(0) - init_normal.transpose() * (tf1 - p1);
        }
    }
    Eigen::MatrixXd BB = A;
    if (DoF_ == 4) {  // scara
        BB = A.block(0, 0, num_jnt_measures - 1, 2);  // needs to put inside the each subobject in the future
        //A.resize(num_jnt_measures - 1, 2);
    }
    strs.str("");
    strs << GetName() << ":" << "B=" << BB << std::endl;
    strs<< "b=" << b << std::endl;
    LOG_INFO(strs);
    Eigen::MatrixXd ATA = BB.transpose() * BB;
    const double detV = ATA.determinant();
    strs.str("");
    strs << GetName() << ":" << "determinat is " << detV << std::endl;
    LOG_INFO(strs);
    if (detV < CALIB_SINGULAR_CONST) {
       for (size_t j=0; j < 3; j++) {
         ATA(j, j) += angle_ridge_scale_;
       }
    }
    Eigen::VectorXd outV = ATA.inverse() * BB.transpose() * b;
    final_tool_offset->setZero();  // init to 0
    for (size_t i=0; i < outV.size(); i++) {
      (*final_tool_offset)(i) = outV(i);
    }

    // for this displacement measure sensor based tcp calibration, it only suits with pinacle or needle type tcp,
    // for which the tooloffset is aligned with one of the axis of tcp frame, for scara, this axis is x-axis, for
    //6axis, this axis is z-axis
    Quaternion q;
    if (DoF_ == 4) { // scara
      double yaw = atan2(outV(1), outV(0));
      q.SetEulerZYX(yaw,  0, 0);
    } else {  // 6 axis
      Vec v0(outV);
      Rotation r(v0);
      r.GetQuaternion(&q);
    }
    (*final_tool_offset)(3) = q.w();
    (*final_tool_offset)(4) = q.x();
    (*final_tool_offset)(5) = q.y();
    (*final_tool_offset)(6) = q.z();
    return 0;
}

// major function for DH calibration
double serialArm::CalibrateDirectMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &measureMents,  // direct sensor measurement
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) {
    std::ostringstream strs;
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Robot geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!final_tool_offset) {
      strs.str("");
      strs << GetName() << ":" << "Input final_tool_offset pointer is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;   
    }
    //! temporarily, we use default base offset
    *final_base_offset = init_base_offset;
    strs.str("");
    strs << GetName() << ":" << "before calib, init base = " << init_base_offset 
         << ", init tool =" << init_tool_offset << std::endl;
    size_t total_measures = qa_array.cols();
    size_t num_measures = 2 * total_measures / 3;
 
    if (num_measures < 2) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara laser calibration: need at least two samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }
    if (init_base_offset.rows() !=7) {
      strs.str("");
      strs << GetName() << ":" << "Input init_base_offset has wrong dimension,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }

    size_t measDoF = measureMents.rows();
    if (total_measures != measureMents.cols() || measDoF > 3) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara calibration: number of measure ee coordinates"
              " and number of joint records does not match, or meas has dimension > 3"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    
    alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                  ridgeScale_,
                  cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, a_old = a_c_, alpha_tmp = alpha_c_,
                        beta_tmp = beta_c_, beta_old = beta_c_,
                        alpha_old = alpha_c_, d_tmp = d_c_, d_old = d_c_,
                        theta_tmp = theta_c_, theta_old = theta_c_;
    // start with initial tool offset
    Eigen::Vector3d tool_tmp = init_tool_offset.segment(0, 3);
    Eigen::Vector3d tool_old = tool_tmp;
    
     // convert base_offset from trans, quat -> trans, EulerZYX
    Quaternion bq(init_base_offset(3), init_base_offset(4),
                  init_base_offset(5), init_base_offset(6));
    double init_yaw, init_pitch, init_roll;
    bq.GetEulerZYX(&init_yaw, &init_pitch, &init_roll);
    Eigen::VectorXd base_tmp(6);
    base_tmp(0) = init_base_offset(0);
    base_tmp(1) = init_base_offset(1);
    base_tmp(2) = init_base_offset(2);
    base_tmp(3) = init_roll;
    base_tmp(4) = init_pitch;
    base_tmp(5) = init_yaw;
    Eigen::VectorXd base_old  = base_tmp;
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 3, base_x, base_y, base_z, base_roll, base_pitch, base_yaw, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3], 
    Eigen::MatrixXd A((num_measures-1) * measDoF, 6 + 5 * DoF_ + 3);
    Eigen::VectorXd b((num_measures-1) * measDoF);
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    Eigen::MatrixXd first_J; // very first Jacobian
    Eigen::VectorXd first_p; // first cartesian vec
    while ((estimation_err >  MAX_CALIB_STOP_ERR &&
           previous_err - estimation_err > MAX_CALIB_MATCHING_ERR || !resetCache_)
           && cur_iter < MAX_CALIB_OUTER_ITER
           // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
           ) {
      // assign previous value
      if (resetCache_) {
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        tool_old = tool_tmp; base_old = base_tmp;
        previous_diff = current_diff;
        previous_err = estimation_err;
      }
      cur_iter++;

      // compute the base tranformation
      Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
      Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
      Rotation rb(Eub(2), Eub(1), Eub(0)); 
      Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
      Eigen::Vector3d col1(0, 0, 1);
      EulerDiff.col(2) = col1;
      Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
      EulerDiff.col(1) = col2;
      Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
      EulerDiff.col(0) = col3;
      Vec vb = Vec::FromEigenVec(pb);
      // Frame base(rb, vb);  // get the base transformation
      
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      estimation_err = 0;
      A.resize((num_measures - 1) * measDoF, 6 + 5 * DoF_ + 3);
      for (size_t i=0; i < num_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_tmp);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb; 
        Eigen::Matrix3d pb_tf;
         (vb - tf).ToHat(&pb_tf); 
        Eigen::MatrixXd J_block1 = Eigen::MatrixXd::Identity(3, 3);  // corresponding to delta pb
        Eigen::MatrixXd J_block2 = pb_tf * EulerDiff; // corresponding to delta Eub
        
        // next we compute   J0T, not including base, corresponding to delta p, delta tool
        Eigen::Matrix3d t_e_hat;
        t_e.ToHat(&t_e_hat);
        Eigen::MatrixXd J_block3 = rb.ToEigenMat() * (Jp_t - t_e_hat * Jp_r);  //corresponding to delta p
        Eigen::MatrixXd J_block4 = (rb * r_04).ToEigenMat();  // corresponding to delta tool
        
        // now merge 4 j blocks into one jacobian matrix
        Eigen::MatrixXd current_J(3, 6 + 5 * DoF_ + 3);
        current_J.block(0,0,3,3) = J_block1;
        current_J.block(0,3,3,3) = J_block2;
        current_J.block(0,6,3,5 * DoF_) = J_block3;
        current_J.block(0, 6 + 5 * DoF_, 3, 3) = J_block4;
        if (i==0) {
            first_J = current_J.block(0, 0, measDoF, 6 + 5 * DoF_ + 3);
            first_p = tf.ToEigenVec().block(0,0,measDoF,1);
        } else {
           Eigen::Vector3d dp =  tf.ToEigenVec().block(0,0,measDoF,1) - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           Eigen::Vector3d dls = measureMents.col(i) - measureMents.col(0);
           // laser rel. displacement  - robot reported displacement 
           Eigen::Vector3d epsi = dls - dp;
           double av_error = epsi.norm();
           //double accu_error = av_error * av_error;
           estimation_err += av_error; //accu_error;
           A.block((i - 1) * measDoF, 0, measDoF, 6 + 5 * DoF_ + 3) = current_J.block(0, 0, measDoF, 6 + 5 * DoF_ + 3) - first_J;
           b.block((i - 1) * measDoF, 0, measDoF, 1) = epsi;
        }
      }

      // average estimation error for a single measurement
      estimation_err  /= double(num_measures - 1) ;
      current_diff = std::min(previous_err - estimation_err,
              std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // set up RNN algorithm
    
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through full range sensor, iteration no. = " << cur_iter <<  ", current error=" << estimation_err << std::endl;
      LOG_INFO(strs);
      Eigen::VectorXd delta_p_old;

     
      if (resetCache_) {
        size_t numParam = A.cols();
        delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
      }
      if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
         if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
         }
         delta_p_old_cache_ = delta_p_old;
         UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp);
         // reset estimation_err. to enforce go to next iteration
         //estimation_err = previous_err - 1.02 * MAX_CALIB_MATCHING_ERR;
         resetCache_ = false;
      } else {
         UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp);
         resetCache_ = true;
      }                   
    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs << GetName() << ":" << "In fullrange calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                 << "With estimation error " << estimation_err
                 << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR) {
        strs.str("");
        strs << GetName() << ":" << "In fullrange calib: Iteration reaches estimation_err " << estimation_err
                 << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR) {
        strs.str("");
        strs << GetName() << ":" << "In fullrange calib: Iteration reaches err diff " << previous_err - estimation_err
                 << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    
    alpha_c_ = alpha_old;
    a_c_ = a_old;
    theta_c_ = theta_old;
    d_c_ = d_old;
    beta_c_ = beta_old;
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "tool = " << tool_old << std::endl;
    strs << "base = " << base_old << std::endl;
    strs << "final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
    (*final_tool_offset)(0) = tool_old(0);
    (*final_tool_offset)(1) = tool_old(1);
    (*final_tool_offset)(2) = tool_old(2);
    // convert base_old from EulerZYX to Quaternion
    Quaternion q0;
    // base_old orientation is in the order of (roll, pitch, yaw)
    // however, yaw is angle about z-axis, pitch is angle about y-axis, and 
    // roll is about x axis
    q0.SetEulerZYX(base_old(5), base_old(4), base_old(3));
    (*final_base_offset)(0) = base_old(0);
    (*final_base_offset)(1) = base_old(1);
    (*final_base_offset)(2) = base_old(2);
    (*final_base_offset)(3) = q0.w();
    (*final_base_offset)(4) = q0.x();
    (*final_base_offset)(5) = q0.y();
    (*final_base_offset)(6) = q0.z();

    // Note: base_off_c_ is not used because it is only used in calibration, but not in compensation
    // in calibration, there might be offset between robot base frame and sensor measuring frame, but in 
    // compensation, the desired traj is w.r.t. robot base frame
    ////base_off_c_.setTranslation(Vec(base_old(0), base_old(1), base_old(2)));
    ////base_off_c_.setQuaternion(q0);
    isDHCalibrated_ = true;

    // last step using another half of measurements to verify that calibrated DHs
    // are going to shrink the errors
    // compute the base tranformation
    Eigen::Vector3d pb = base_old.block(0,0,3,1);   // base translation
    Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
    Rotation rb(Eub(2), Eub(1), Eub(0));
    Vec vb = Vec::FromEigenVec(pb);
    // Frame base(rb, vb);  // get the base transformation 

    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());
    double orig_err = 0, comp_err=0;
    for (size_t i=num_measures; i < total_measures; i++) {
        UpdateDH(kine_para, qa_array.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
           return ret;
        }
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_old);
        Vec t_e = r_04 * tool_offset + t_04;
        Vec tf = rb * t_e + vb;

        if (i==num_measures) {
            first_p = tf.ToEigenVec().block(0,0,measDoF,1);
            Rotation rf = rb * r_04;
            double yaw1, pitch1, roll1;
            rf.GetEulerZYX(&yaw1, &pitch1, &roll1);
            strs.str("");
            strs << GetName() << ":" << "after compensation, yaw= " << yaw1 << ", pitch=" << pitch1
                 << ", roll=" << roll1 << std::endl;
            LOG_INFO(strs);
        } else {
           Eigen::VectorXd dp =  tf.ToEigenVec().block(0,0,measDoF,1) - first_p; // (cart_measure.col(i) - cart_measure.col(0)).block(0,0,3,1);
           //Eigen::Vector3d dls1 =  laser2CartMap * 
           //        (laser_measure.col(i) - laser_measure.col(num_measures));
           Eigen::VectorXd dls = measureMents.col(i) - measureMents.col(num_measures);
           // laser rel. displacement  - robot reported displacement 
           Eigen::VectorXd epsi = dls - dp;
           double av_error = epsi.norm();
           // double accu_error = av_error * av_error;
           comp_err += av_error; // accu_error;
           Eigen::VectorXd dcart = cart_measure.col(i) - cart_measure.col(num_measures);
           Eigen::Vector3d dc = dcart.block(0,0,measDoF,1);
           epsi = dls - dc;
           av_error = epsi.norm();
           // accu_error = av_error * av_error;
           orig_err += av_error; //accu_error;
        }
    }
    double avg_orig_err= orig_err / (total_measures - num_measures - 1);
    double avg_comp_err = comp_err / (total_measures - num_measures - 1);
    double overall_comp_err = (previous_err * 2.0 + avg_comp_err) / 3.0;  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " <<  avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        //return -overall_comp_err;
    }
    return overall_comp_err;
}
// pose[6] is the branch flags
bool serialArm::GetJntFromPose(const Eigen::VectorXd &pose,
                       Eigen::VectorXd *jnt) {
    // std::ostringstream strs;
    if (!initialized_ || !jnt){
      return false;
    }
    int DoF = GetActDoF();
    std::vector<double> jnt1(DoF, 0);
    
    Vec trans(pose[0], pose[1], pose[2]);
    Rotation r(pose[5], pose[4], pose[3]);
    
    size_t sz_pose = pose.size();
    int flag =(int) pose[6];
    
    std::vector<int> branchFlags(3, 0);
    int i = 2;
    while (flag >= 1 && i >= 0) {
      branchFlags[i] = flag % 2;
      flag = std::floor(flag / 2.0);
      i = i - 1;
    }
    std::vector<int> ikJointTurns(DoF, 0);
    Frame fr(r, trans);
    Pose ps(fr, branchFlags, ikJointTurns);

    if (CartToJnt(ps, &jnt1) < 0) {
      return false;
    }
    StdVec2EigenVec(jnt1, jnt);
    // strs << "GetJntFromPose(): cart=" << pose << ", jnt=" << *jnt << std::endl;
    // LOG_INFO(strs);
    return true;
}


  bool serialArm::GetPoseFromJnt(const Eigen::VectorXd &jnt,
                   Eigen::VectorXd *pose) {
     // std::ostringstream strs;
    if (!initialized_ || !pose){
      return false;
    }
    size_t DoF = GetActDoF();
    Pose ps;
    std::vector<double> jnt1(DoF, 0);
    for (size_t i=0; i < DoF; i++) {
        jnt1[i] = jnt(i);
    }
    if (JntToCart(jnt1, &ps) <0) {// from default world to default tool, even if robot base is not
                                    // identity, we still can use SetCalibMode to incorporate the base_off
      return false;
    
    }
    
    Vec p = ps.getTranslation();
    Rotation r = ps.getRotation();
    double yaw, pitch, roll;
    r.GetEulerZYX(&yaw, &pitch, &roll);
 
    pose->resize(8);   // pose(6) is config, while pose(7) is joint turns
    (*pose)(0) = p.x();
    (*pose)(1) = p.y();
    (*pose)(2) = p.z();
    (*pose)(3) = roll;
    (*pose)(4) = pitch;
    (*pose)(5) = yaw;
    std::vector<int> branchFlags;
    ps.getBranchFlags(&branchFlags);
    size_t numFlags = branchFlags.size();
    if (numFlags == 0) {
      (*pose)(6) = 0;
    } else {
      (*pose)(6) = branchFlags[0];   // branch[2] flip/nonflip,  branch[1]  up/down, branch[0] left/right
      for (size_t i=1; i< branchFlags.size(); i++) {
        (*pose)(6) = (*pose)(6) * 2.0 + branchFlags[i];
      }
    }

    std::vector<int> turns;
    ps.getJointTurns(&turns);
    int turnValue = 0;
    int multi = 1;
    for (size_t i=0; i < DoF; i++) {
       int turn1 = turns[i];
       if (turn1 < 0) {
           turn1 += 16;   // when turn < 0, e.g. turn=-1, it is actually -1 + 16 = 15=F; turn =-2, it is actually -2 + 16= 14=E   
       }
       turnValue += turn1 * multi;
       multi *= 16;
    }
    (*pose)(7) = turnValue;
    return true;
  }


double  serialArm::CalibrateLaserCoplanarSing(
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::VectorXd> &laser_measure,
       const double laser_scale,
       const int axisChannel) {   // axisChannel \in [0,1,2]
   // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    LOG_INFO(strs);
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
   
    size_t num_l = laser_measure.size();
    size_t num_c = cart_measure.cols();
    bool sizeOK = (num_l == num_c);
    if (cart_measure.rows() < 3 || !sizeOK || axisChannel < 0 || axisChannel >= 3) {
      strs.str("");
      strs << GetName() << ":" << "CalibrateLaserCoplanar: input data dimension is not matching"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }

    size_t num_measure = 3 * num_l / 4 ;
    size_t total_rows = num_measure - 3;

    if (num_measure < 4) {
      strs.str(""); 
      strs << GetName() << ":" << "CalibrateLaserCoplanar: 3/4 calib samples need at least 4 samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }
    // initialize iter alg parameters
    alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                  ridgeScale_, cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    //! before we start iteration, we compute the "supposed" joint arrays
    Eigen::MatrixXd  jnt_shift(DoF_, num_c);
    //! offseted cartesian array (i.e. offset based upon laser measure data)
    Eigen::MatrixXd cart_off_measure = cart_measure;
    Eigen::VectorXd jnt;
    double first_laser;

    //before doing anything, set using uncalibrated model
    SetUsingCalibratedModel(false);
    // plane parallel to  block surface (could be x,y, or z)
    for (size_t i=0; i < num_c; i++) {
        Eigen::VectorXd cart = cart_measure.col(i);
        double laser = laser_measure(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(axisChannel) = cart(axisChannel) - laser_scale * (laser - first_laser);
        }
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_shift.col(i) = jnt;
        cart_off_measure.col(i) = cart;
    }
    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                        d_tmp = d_c_, theta_tmp = theta_c_;
    std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                        d_old = d_tmp, theta_old = theta_tmp;
    
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
    Eigen::MatrixXd A(total_rows, 6 + 5 * DoF_ + 12);
    Eigen::VectorXd b(total_rows);
    Eigen::Vector3d dv1, dv2, dv3;
    double norm_dv1, norm_dv2, norm_dv3;
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
           previous_err - estimation_err > MAX_CALIB_MATCHING_ERR_COPLANAR || !resetCache_)
           && cur_iter < MAX_CALIB_OUTER_ITER
           // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
           ) {
      // assign previous value
      if (resetCache_) {    // if resetCache_ is true, then reset previous_err = estimation_err
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        previous_err = estimation_err;
      }
      previous_diff = current_diff;
      cur_iter++;
      
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      estimation_err = 0;
     
      // before optimization, A and b need to resize, because its dep. columns is removed during opt.
      A.resize(total_rows, 6 + 5 * DoF_ + 12);
      b.resize(total_rows);
      
      // pp and J vector are for computing the items in A matrix, and b vector
      std::vector<Eigen::Vector3d> pp(4);
      std::vector<Eigen::MatrixXd> J(4);
      Eigen::MatrixXd tmpJ(3, 5 * DoF_);
      Eigen::MatrixXd dJ1, dJ2, dJ3;
      size_t start_matrix_rows = 0;

      // first plane constraint equation
      for (size_t i=0; i < num_measure; i++) {
        UpdateDH(kine_para, jnt_shift.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        Eigen::Matrix3d t_e_hat;
        tf.ToHat(&t_e_hat);
        tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
        if (i < 3) {
          J[i] = tmpJ;
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =  dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =  dv2.norm();
             dJ1 = J[1] - J[0];
             dJ2 = J[2] - J[1];
          }
        } else {
          J[3] = tmpJ;
          pp[3] = tf.ToEigenVec();
          dJ3 = J[3] - J[2];
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();

          // computing the Jacobian
          A.block((i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
          Eigen::Matrix3d AA;
          for (size_t j=0; j< 5 * DoF_; j++) {
                 double sum = 0;
                 AA.col(0) = dJ1.col(j) / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dv3 / norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;;
                 AA.col(1) = dJ2.col(j) / norm_dv2;
                 AA.col(2) = dv3 /  norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dJ3.col(j) / norm_dv3;
                 sum += AA.determinant();
                 A(i -3, j + 6) = sum;
                 //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
          }
          A.block(i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 /  norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          b(i - 3) = - AA.determinant();
          estimation_err += fabs(b(i - 3));
        }
      }
      strs.str("");
      // average estimation error for a single measurement
      estimation_err  /= double(total_rows) ;
      strs << GetName() << ":" << "esitmation_err z "<< estimation_err <<  std::endl;
      LOG_INFO(strs);
      current_diff = std::min(previous_err - estimation_err,
               std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        //strs.str("");
        //strs << "removing cols from A: " << d_jacobian_cols_[i] << std::endl;
        //LOG_INFO(strs);
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through laser sensor, iteration no. = " << cur_iter << std::endl;
      LOG_INFO(strs);
      Eigen::VectorXd delta_p_old;
      Eigen::VectorXd base_tmp(6);
      base_tmp.setZero();
      Eigen::Vector3d tool_tmp(3);
      tool_tmp.setZero();

      if (resetCache_) {
        size_t numParam = A.cols();
        delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
      }
      if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
         if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
         }
         delta_p_old_cache_ = delta_p_old;
         UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, NULL);
         // reset estimation_err. to enforce go to next iteration
         //estimation_err = previous_err - 1.02 * MAX_CALIB_MATCHING_ERR;
         resetCache_ = false;
      } else {
         UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, NULL);
         resetCache_ = true;
      }
    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                 << "With estimation error " << estimation_err
                 << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches estimation_err " << estimation_err
                 << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches err diff " << previous_err - estimation_err
                 << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                 << std::endl;
        LOG_ERROR(strs);
    }
    alpha_c_ = alpha_old;
    a_c_ = a_old;
    theta_c_ = theta_old;
    d_c_ = d_old;
    beta_c_ = beta_old;
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
  
    isDHCalibrated_ = true;


    // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    double orig_err = 0, comp_err=0;
    Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
    double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;

    // compare  the planar error before calib and after comp.
    // first plane constraint equation
    size_t numValid = 0;
    for (size_t i=0; i < num_c; i++) {
      UpdateDH(kine_para, jnt_shift.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        return ret;
      }
      Vec  tf = p.getTranslation();
      size_t j = i;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =dv2.norm();
             dv1_orig = cart_off_measure.col(1) - cart_off_measure.col(0);
             norm_dv1_orig = dv1_orig.norm();
             dv2_orig = cart_off_measure.col(2) - cart_off_measure.col(1);
             norm_dv2_orig = dv2_orig.norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_off_measure.col(i) - cart_off_measure.col(2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err += fabs(AA.determinant());
      }
    }
    numValid += num_c - 3;
    double avg_orig_err= orig_err /  numValid;
    double avg_comp_err = comp_err / numValid;
    //double overall_comp_err = (previous_err * total_rows + orig_err) / (total_rows + numValid);  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        // return -agv_comp_err;
    }
    return avg_comp_err;
}


double serialArm::CalibrateLaserOrientation(
       const EigenDRef<Eigen::MatrixXd> &jnt_measure, //  | DoF * 1 | DoF * 1 | DoF * 1 | DoF * 1 | ..... Every surface: 1 column points
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // | 8 * 3| 8 * 3| 8 * 3| 8 * 3|  .... every surface: 3 pts
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,   // not used
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       // const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const double max_laser_dist, // maximal laser distance, under which, laser reading is 0
       const int numPtsInEachPlane,
       const std::vector<int> surfaceArrays) {
  
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t num_row_jnt = jnt_measure.rows();
      size_t num_col_jnt = jnt_measure.cols();

      size_t num_row_cart = cart_measure.rows();
      size_t num_col_cart = cart_measure.cols();
      
      // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
      bool sizeOK = ((num_col_cart == numPtsInEachPlane * num_col_jnt) && (numPtsInEachPlane >= 3));
      if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 4) {
        strs.str("");
        strs << GetName() << ":" << "input data dimension is not matching"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      }
          // unit_normal
      Eigen::Vector3d unit_normal = orient_normal_ /  orient_normal_.norm();
      strs.str("");
      strs << " unit_normal=" << unit_normal << std::endl;
      LOG_INFO(strs);

      
      // initialize iter alg parameters
      alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                    ridgeScale_, cyc_mod_, sam_region_scale_);
      resetCache_ = true; 
      // now compute Jacobian and iterative optimization of parameters
    
      // in any case, we start with uncaliberated model
      std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                          d_tmp = d_c_, theta_tmp = theta_c_;
      std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                          d_old = d_tmp, theta_old = theta_tmp;
      
      size_t num_calib_jnt =  num_col_jnt; // std::floor(num_col_jnt * 3 / 4);
      size_t total_rows = num_calib_jnt; //num_calib_jnt - numMeasSurface;
      // step 1, using canonical FK to compute the corresponding
      // we need to define one matrix A and one vector b for regression
      // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
      // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
      Eigen::MatrixXd A(3 * total_rows, 6 + 5 * DoF_ + 12);
      Eigen::VectorXd b(3 * total_rows);

      // need to optimize base (UR, and XYZ shall have different base)
      Eigen::VectorXd base_tmp(6);
      Vec tBase = defaultBaseOff_.getTranslation();
      //Rotation rBase = defaultBaseOff_.getRotation();
      Quaternion q = defaultBaseOff_.getQuaternion();
      double init_yaw, init_pitch, init_roll;
      q.GetEulerZYX(&init_yaw, &init_pitch, &init_roll);
      base_tmp(0) = tBase.x();
      base_tmp(1) = tBase.y();
      base_tmp(2) = tBase.z();
      base_tmp(3) = init_roll;
      base_tmp(4) = init_pitch;
      base_tmp(5) = init_yaw;
      Eigen::VectorXd base_old  = base_tmp;

      // make a copy of defaultBaseOff_
      Frame defaultBaseOffCopy = defaultBaseOff_;

      // now temporarily set defaultBaseOff_ into 0
      Frame tmp;
      defaultBaseOff_ = tmp;
    
      double previous_err = std::numeric_limits<double>::max();
      double estimation_err = 0.5 * previous_err;
      double estimation_err1; //= 0.5 * previous_err;
      double previous_diff = previous_err - estimation_err;
      double current_diff = previous_diff / 2.0;
      int cur_iter = 0;
      double init_estimation_err = 0;
      while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
            previous_err - estimation_err > MAX_ORIENT_CALIB_MATCHING_ERR || resetCache_)
            && cur_iter < MAX_CALIB_OUTER_ITER
            // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
            ) {
        strs.str("");
        strs << "previous_err=" << previous_err << ", estimation_err=" << estimation_err <<
           ", resetCache=" <<  !resetCache_ << ", diff_err=" << previous_err - estimation_err << std::endl;
        LOG_INFO(strs);
        // assign previous value
        if (resetCache_) {
          a_old = a_tmp; alpha_old = alpha_tmp;
          d_old = d_tmp; theta_old = theta_tmp;
          beta_old  = beta_tmp;
          base_old = base_tmp;
        } else {
          previous_err = estimation_err;
        }
        previous_diff = current_diff;
        cur_iter++;

        // compute the base tranformation
        Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
        
        // Base orientation diff matrix
        Rotation rb(Eub(2), Eub(1), Eub(0)); 
        Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
        Eigen::Vector3d col1(0, 0, 1);
        EulerDiff.col(2) = col1;
        Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
        EulerDiff.col(1) = col2;
        Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
        EulerDiff.col(0) = col3;


        std::vector<double> kine_para, tmp_para; // clearing kine_para
        // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
        kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
        kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
        kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
        kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
        kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
        estimation_err1 = 0;
      
        // before optimization, A and b need to resize, because its dep. columns is removed during opt.
        // A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
        A.resize(3 * total_rows, 6 + 5 * DoF_ + 12);

        // tmp variable to compute the measured normals and predicted normals
        Eigen::Vector3d currentN, currentNd, currentNd1;
        std::vector<Eigen::Vector3d> pp(4);
        //Eigen::MatrixXd firstJ, currentJ;
        Eigen::Matrix3d currentNMat; // firstNMat = \hat{firstN}, currentNMat = \hat{currentN}
        Eigen::Vector3d dv1, dv2;
      
        strs.str("");
        strs << GetName();
        for (size_t i=0; i < num_calib_jnt; i++) {
            // compute the measured normal of the plane formed by 3 vectors
            pp[0] =  cart_measure.col(i * numPtsInEachPlane).segment(0, 3);
            pp[1] =  cart_measure.col(i * numPtsInEachPlane + 1).segment(0, 3);
            pp[2] =  cart_measure.col(i * numPtsInEachPlane + 2).segment(0, 3);
            pp[3] =  cart_measure.col(i * numPtsInEachPlane + 3).segment(0, 3);
            dv1 = pp[1] - pp[0];
            dv2 = pp[2] - pp[1];
            currentNd = dv1.cross(dv2);
            double norm_nd = currentNd.norm();
            currentNd /= norm_nd;   // measured normal

            if (currentNd(laser_channel) < 0) {
              currentNd =-currentNd;
            }

            dv1 = pp[3] - pp[2];
            dv2 = pp[0] - pp[3];
            currentNd1 = dv1.cross(dv2);
            norm_nd = currentNd1.norm();
            currentNd1 /= norm_nd;
            if (currentNd(laser_channel) < 0) {
              currentNd1 =-currentNd1;
            }

            // now doing average of the two normal
            currentNd = (currentNd + currentNd1) / 2.0;
            norm_nd = currentNd.norm();
            currentNd /= norm_nd;   // measured normal
             strs << ", measure " << i << " has normal=" << currentNd << std::endl;
           
            // now compute the normal predicted by uncalibrated model (or current model)
            UpdateDH(kine_para, jnt_measure.col(i), &tmp_para);
            Eigen::MatrixXd Jp_t, Jp_r;
            Pose p;
            // compute the expected values from known canonical kinematic parameters
            // i.e., not-calibrated parameter set
            int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
            if (ret < 0) {
              defaultBaseOff_ = defaultBaseOffCopy;
              return ret;
            }

            // we get R, the rotation matrix between base and 4
            Rotation r = rb * p.getRotation();
            currentN = r.ToEigenMat() * unit_normal; // predicted normal
            if (currentN.dot(currentNd) < -K_EPSILON)  {
               unit_normal *= -1.0;
               currentN *= -1.0;
               strs.str("");
               strs << " modified unit_normal=" << unit_normal << std::endl;
               LOG_INFO(strs);
            }
            Vec vN(currentN);
            vN.ToHat(&currentNMat);  // hat(currentN)
            
            Eigen::Vector3d dN = currentN - currentNd;
            b.block(3 * i, 0, 3, 1) = dN;   // currentN.dot(firstN) - currentNd.dot(firstNd);
            A.block(3 * i, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3); // dbase_linear
            A.block(3 * i, 3, 3, 3) = currentNMat * EulerDiff; // dbase_euler
            A.block(3 * i, 6, 3, 5 * DoF_) = currentNMat *  rb.ToEigenMat() * Jp_r;
            A.block(3 * i, 6 + 5 * DoF_, 3, 12) = Eigen::MatrixXd::Zero(3, 12);
            estimation_err1 += dN.norm() / total_rows; 
        }
            
        strs.str("");
        strs << GetName() << ":" << "estimation_err z "<< estimation_err1  << std::endl;
        LOG_INFO(strs);
        if (cur_iter <= 1) {
          init_estimation_err = estimation_err1;
        }
        current_diff = std::min(previous_err - estimation_err,
                std::numeric_limits<double>::max() / 8.0);
        // this is to remove the impacts from some redundant parameters
        for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
          // luckily, the vector of depend column indices in d_jacobian_cols_
          // are arranged from largest toward smallest, so we can continuously
          // call removeColumn
          removeColumn(&A, d_jacobian_cols_[i]);
        }
        // using RNN to compute the best delta_para given the current para
        // RNN used as an internal loop
        strs.str("");
        strs << GetName() << ":" << "Calib. through laser sensor orient, iteration no. = " << cur_iter << std::endl;
        LOG_INFO(strs);
        Eigen::VectorXd delta_p_old;
        // Eigen::VectorXd base_tmp(6);
        // base_tmp.setZero();
        Eigen::Vector3d tool_tmp(3);
        tool_tmp.setZero();
        
        if (resetCache_) {
          size_t numParam = A.cols();
          delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
        }
        // strs << "base_old=" << base_old << std::endl;
        if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
          if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
          }
          delta_p_old_cache_ = delta_p_old;
          UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = false;
          estimation_err = estimation_err1;
        } else {
          UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = true;
          estimation_err = previous_err - 1.5 * MAX_ORIENT_CALIB_MATCHING_ERR;
        }
        strs.str("");
        strs << "base_tmp=" << base_tmp << std::endl; 
        LOG_INFO(strs);     
      }

      if (cur_iter >= MAX_CALIB_OUTER_ITER) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                  << "With estimation error " << estimation_err
                  << std::endl;
          LOG_ERROR(strs);
          //return -ERR_CALIB_REG_MAX_ITER;
      }
      if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches estimation_err " << estimation_err
                  << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                  << std::endl;
          LOG_ERROR(strs);
      }
      if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches err diff " << previous_err - estimation_err
                  << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                  << std::endl;
          LOG_ERROR(strs);
      }
      strs.str("");
      if (init_estimation_err > 0 && estimation_err < init_estimation_err) {
        alpha_c_ = alpha_old;
        a_c_ = a_old;
        theta_c_ = theta_old;
        d_c_ = d_old;
        beta_c_ = beta_old;
        strs << GetName() << ": original baseoff=" << defaultBaseOffCopy.ToString(true);
        strs << ", base_old =" << base_old << std::endl;
        // reset defaultBaseOff__;
        Vec vb= Vec::FromEigenVec(base_old.segment(0, 3));
        Quaternion qb;
        qb.SetEulerZYX(base_old(5), base_old(4), base_old(3));
        strs << ", qb=" << qb.ToString(false) << std::endl;
        defaultBaseOff_.setTranslation(vb);
        defaultBaseOff_.setQuaternion(qb);
        strs << ", after calib, baseoff=" << defaultBaseOff_.ToString(true) << "in quat=" << defaultBaseOff_.ToString(false) << std::endl;
      } else {
        defaultBaseOff_ = defaultBaseOffCopy;
        strs << "No base update, after calib, baseoff=" << defaultBaseOff_.ToString(true) << "in quat=" << defaultBaseOff_.ToString(false) << std::endl;
      }
      
      strs << "alpha_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << alpha_c_[k] << " ";
      }
      strs << ", a_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << a_c_[k] << " ";
      }
      strs << ", theta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << theta_c_[k] << " ";
      }
      strs << ", d_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << d_c_[k] << " ";
      }
      strs << ", beta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << beta_c_[k] << " ";
      }
      strs << std::endl;
      strs << "init matching error" << init_estimation_err << ", final matching error=" << previous_err << std::endl;
      LOG_INFO(strs);
    
      isDHCalibrated_ = true;
      return previous_err;
  } catch (...) {
    strs.str("");
    strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
    LOG_ERROR(strs);
    return -1111;
  }
}

double serialArm::LaserCalibrateOrientation(
       const Eigen::MatrixXd &jnt_measure, //  | DoF * 1 | DoF * 1 | DoF * 1 | DoF * 1 | ..... Every surface: 1 column points
       const Eigen::MatrixXd &cart_measure,  // | 8 * 3| 8 * 3| 8 * 3| 8 * 3|  .... every surface: 3 pts
       const Eigen::VectorXd &laserMat_z_measure,   // not used
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const double max_laser_dist,
       const int numPtsInEachPlane, // number of points in each plane
       const std::vector<int> surfaceArrays) {
   // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t num_row_jnt = jnt_measure.rows();
      size_t num_col_jnt = jnt_measure.cols();

      size_t num_row_cart = cart_measure.rows();
      size_t num_col_cart = cart_measure.cols();
      
      // either cart measure cols == 3 * jnt measure cols (given one fixed jnt vec, we need at least 3 col cart)
      bool sizeOK = ((num_col_cart == numPtsInEachPlane * num_col_jnt) && (numPtsInEachPlane >= 3));
      if (num_row_cart < 3 || num_row_jnt < DoF_ || !sizeOK || num_col_jnt < 4) {
        strs.str("");
        strs << GetName() << ":" << "input data dimension is not matching"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      }
          // unit_normal
      Eigen::Vector3d unit_normal = orient_normal_ /  orient_normal_.norm();
      strs.str("");
      strs << " unit_normal=" << unit_normal << std::endl;
      LOG_INFO(strs);

      
      // initialize iter alg parameters
      alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                    ridgeScale_, cyc_mod_, sam_region_scale_);
      resetCache_ = true; 
      // now compute Jacobian and iterative optimization of parameters
    
      // in any case, we start with uncaliberated model
      std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                          d_tmp = d_c_, theta_tmp = theta_c_;
      std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                          d_old = d_tmp, theta_old = theta_tmp;
      
      size_t num_calib_jnt =  num_col_jnt; // std::floor(num_col_jnt * 3 / 4);
      size_t total_rows = num_calib_jnt; //num_calib_jnt - numMeasSurface;
      // step 1, using canonical FK to compute the corresponding
      // we need to define one matrix A and one vector b for regression
      // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
      // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
      Eigen::MatrixXd A(3 * total_rows, 6 + 5 * DoF_ + 12);
      Eigen::VectorXd b(3 * total_rows);

      // need to optimize base (UR, and XYZ shall have different base)
      Eigen::VectorXd base_tmp(6);
      Vec tBase = defaultBaseOff_.getTranslation();
      //Rotation rBase = defaultBaseOff_.getRotation();
      Quaternion q = defaultBaseOff_.getQuaternion();
      double init_yaw, init_pitch, init_roll;
      q.GetEulerZYX(&init_yaw, &init_pitch, &init_roll);
      base_tmp(0) = tBase.x();
      base_tmp(1) = tBase.y();
      base_tmp(2) = tBase.z();
      base_tmp(3) = init_roll;
      base_tmp(4) = init_pitch;
      base_tmp(5) = init_yaw;
      Eigen::VectorXd base_old  = base_tmp;

      // make a copy of defaultBaseOff_
      Frame defaultBaseOffCopy = defaultBaseOff_;

      // now temporarily set defaultBaseOff_ into 0
      Frame tmp;
      defaultBaseOff_ = tmp;
    
      double previous_err = std::numeric_limits<double>::max();
      double estimation_err = 0.5 * previous_err;
      double estimation_err1; //= 0.5 * previous_err;
      double previous_diff = previous_err - estimation_err;
      double current_diff = previous_diff / 2.0;
      int cur_iter = 0;
      double init_estimation_err = 0;
      while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
            previous_err - estimation_err > MAX_ORIENT_CALIB_MATCHING_ERR || resetCache_)
            && cur_iter < MAX_CALIB_OUTER_ITER
            // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
            ) {
        strs.str("");
        strs << "previous_err=" << previous_err << ", estimation_err=" << estimation_err <<
           ", resetCache=" <<  !resetCache_ << ", diff_err=" << previous_err - estimation_err << std::endl;
        LOG_INFO(strs);
        // assign previous value
        if (resetCache_) {
          a_old = a_tmp; alpha_old = alpha_tmp;
          d_old = d_tmp; theta_old = theta_tmp;
          beta_old  = beta_tmp;
          base_old = base_tmp;
        } else {
          previous_err = estimation_err;
        }
        previous_diff = current_diff;
        cur_iter++;

        // compute the base tranformation
        Eigen::Vector3d Eub = base_old.block(3,0,3,1); // base orientation, roll, pitch, yaw
        
        // Base orientation diff matrix
        Rotation rb(Eub(2), Eub(1), Eub(0)); 
        Eigen::Matrix3d EulerDiff;   // wb = EulerDiff * [delta roll, delta pitch, delta yaw]'
        Eigen::Vector3d col1(0, 0, 1);
        EulerDiff.col(2) = col1;
        Eigen::Vector3d col2(-sin(Eub(2)), cos(Eub(2)), 0);
        EulerDiff.col(1) = col2;
        Eigen::Vector3d col3(cos(Eub(2))*cos(Eub(1)), sin(Eub(2)) * cos(Eub(1)), -sin(Eub(1)));
        EulerDiff.col(0) = col3;


        std::vector<double> kine_para, tmp_para; // clearing kine_para
        // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
        kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
        kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
        kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
        kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
        kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
        estimation_err1 = 0;
      
        // before optimization, A and b need to resize, because its dep. columns is removed during opt.
        // A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
        A.resize(3 * total_rows, 6 + 5 * DoF_ + 12);

        // tmp variable to compute the measured normals and predicted normals
        Eigen::Vector3d currentN, currentNd, currentNd1;
        std::vector<Eigen::Vector3d> pp(4);
        //Eigen::MatrixXd firstJ, currentJ;
        Eigen::Matrix3d currentNMat; // firstNMat = \hat{firstN}, currentNMat = \hat{currentN}
        Eigen::Vector3d dv1, dv2;
      
        strs.str("");
        strs << GetName();
        for (size_t i=0; i < num_calib_jnt; i++) {
            // compute the measured normal of the plane formed by 3 vectors
            pp[0] =  cart_measure.col(i * numPtsInEachPlane).segment(0, 3);
            pp[1] =  cart_measure.col(i * numPtsInEachPlane + 1).segment(0, 3);
            pp[2] =  cart_measure.col(i * numPtsInEachPlane + 2).segment(0, 3);
            pp[3] =  cart_measure.col(i * numPtsInEachPlane + 3).segment(0, 3);
            dv1 = pp[1] - pp[0];
            dv2 = pp[2] - pp[1];
            currentNd = dv1.cross(dv2);
            double norm_nd = currentNd.norm();
            currentNd /= norm_nd;   // measured normal

            if (currentNd(laser_channel) < 0) {
              currentNd =-currentNd;
            }

            dv1 = pp[3] - pp[2];
            dv2 = pp[0] - pp[3];
            currentNd1 = dv1.cross(dv2);
            norm_nd = currentNd1.norm();
            currentNd1 /= norm_nd;
            if (currentNd(laser_channel) < 0) {
              currentNd1 =-currentNd1;
            }

            // now doing average of the two normal
            currentNd = (currentNd + currentNd1) / 2.0;
            norm_nd = currentNd.norm();
            currentNd /= norm_nd;   // measured normal
             strs << ", measure " << i << " has normal=" << currentNd << std::endl;
           
            // now compute the normal predicted by uncalibrated model (or current model)
            UpdateDH(kine_para, jnt_measure.col(i), &tmp_para);
            Eigen::MatrixXd Jp_t, Jp_r;
            Pose p;
            // compute the expected values from known canonical kinematic parameters
            // i.e., not-calibrated parameter set
            int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
            if (ret < 0) {
              defaultBaseOff_ = defaultBaseOffCopy;
              return ret;
            }

            // we get R, the rotation matrix between base and 4
            Rotation r = rb * p.getRotation();
            currentN = r.ToEigenMat() * unit_normal; // predicted normal
            if (currentN.dot(currentNd) < -K_EPSILON)  {
               unit_normal *= -1.0;
               currentN *= -1.0;
               strs.str("");
               strs << " modified unit_normal=" << unit_normal << std::endl;
               LOG_INFO(strs);
            }
            Vec vN(currentN);
            vN.ToHat(&currentNMat);  // hat(currentN)
            
            Eigen::Vector3d dN = currentN - currentNd;
            b.block(3 * i, 0, 3, 1) = dN;   // currentN.dot(firstN) - currentNd.dot(firstNd);
            A.block(3 * i, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3); // dbase_linear
            A.block(3 * i, 3, 3, 3) = currentNMat * EulerDiff; // dbase_euler
            A.block(3 * i, 6, 3, 5 * DoF_) = currentNMat *  rb.ToEigenMat() * Jp_r;
            A.block(3 * i, 6 + 5 * DoF_, 3, 12) = Eigen::MatrixXd::Zero(3, 12);
            estimation_err1 += dN.norm() / total_rows; 
        }
            
        strs.str("");
        strs << GetName() << ":" << "estimation_err z "<< estimation_err1  << std::endl;
        LOG_INFO(strs);
        if (cur_iter <= 1) {
          init_estimation_err = estimation_err1;
        }
        current_diff = std::min(previous_err - estimation_err,
                std::numeric_limits<double>::max() / 8.0);
        // this is to remove the impacts from some redundant parameters
        for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
          // luckily, the vector of depend column indices in d_jacobian_cols_
          // are arranged from largest toward smallest, so we can continuously
          // call removeColumn
          removeColumn(&A, d_jacobian_cols_[i]);
        }
        // using RNN to compute the best delta_para given the current para
        // RNN used as an internal loop
        strs.str("");
        strs << GetName() << ":" << "Calib. through laser sensor orient, iteration no. = " << cur_iter << std::endl;
        LOG_INFO(strs);
        Eigen::VectorXd delta_p_old;
        // Eigen::VectorXd base_tmp(6);
        // base_tmp.setZero();
        Eigen::Vector3d tool_tmp(3);
        tool_tmp.setZero();
        
        if (resetCache_) {
          size_t numParam = A.cols();
          delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
        }
        // strs << "base_old=" << base_old << std::endl;
        if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
          if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
          }
          delta_p_old_cache_ = delta_p_old;
          UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = false;
          estimation_err = estimation_err1;
        } else {
          UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = true;
          estimation_err = previous_err - 1.5 * MAX_ORIENT_CALIB_MATCHING_ERR;
        }
        strs.str("");
        strs << "base_tmp=" << base_tmp << std::endl; 
        LOG_INFO(strs);     
      }

      if (cur_iter >= MAX_CALIB_OUTER_ITER) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                  << "With estimation error " << estimation_err
                  << std::endl;
          LOG_ERROR(strs);
          //return -ERR_CALIB_REG_MAX_ITER;
      }
      if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches estimation_err " << estimation_err
                  << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                  << std::endl;
          LOG_ERROR(strs);
      }
      if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser orient calib: Iteration reaches err diff " << previous_err - estimation_err
                  << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                  << std::endl;
          LOG_ERROR(strs);
      }

      strs.str("");
      if (init_estimation_err > 0 && estimation_err < init_estimation_err) {
        alpha_c_ = alpha_old;
        a_c_ = a_old;
        theta_c_ = theta_old;
        d_c_ = d_old;
        beta_c_ = beta_old;
    
        strs << GetName() << ": original baseoff=" << defaultBaseOffCopy.ToString(true);
        strs << ", base_old =" << base_old << std::endl;
        // reset defaultBaseOff__;
        Vec vb= Vec::FromEigenVec(base_old.segment(0, 3));
        Quaternion qb;
        qb.SetEulerZYX(base_old(5), base_old(4), base_old(3));
        strs << ", qb=" << qb.ToString(false) << std::endl;
        defaultBaseOff_.setTranslation(vb);
        defaultBaseOff_.setQuaternion(qb);
        strs << ", after calib, baseoff=" << defaultBaseOff_.ToString(true) << "in quat=" << defaultBaseOff_.ToString(false) << std::endl;
      } else {
        defaultBaseOff_ = defaultBaseOffCopy;
        strs << "No base update, after calib, baseoff=" << defaultBaseOff_.ToString(true) << "in quat=" << defaultBaseOff_.ToString(false) << std::endl;
      }
      
      strs << "alpha_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << alpha_c_[k] << " ";
      }
      strs << ", a_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << a_c_[k] << " ";
      }
      strs << ", theta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << theta_c_[k] << " ";
      }
      strs << ", d_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << d_c_[k] << " ";
      }
      strs << ", beta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << beta_c_[k] << " ";
      }
      strs << std::endl;
      strs << "init matching error" << init_estimation_err << ", final matching error=" << previous_err << std::endl;
      LOG_INFO(strs);
    
      isDHCalibrated_ = true;
      return previous_err;
  } catch (...) {
    strs.str("");
    strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
    LOG_ERROR(strs);
    return -1111;
  }
}

double serialArm::CalibrateLaserCoplanar(
       const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
       const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
       const EigenDRef<Eigen::MatrixXd> &laser_measure_x,
       const EigenDRef<Eigen::MatrixXd> &laser_measure_y,
       const EigenDRef<Eigen::MatrixXd> &laser_measure_z,
       const EigenDRef<Eigen::Vector3d> &laser_scale) {
   // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    try {
      size_t num_lx = laser_measure_x.cols();
      size_t num_cx = cart_measure_x.cols();

      size_t num_ly = laser_measure_y.cols();
      size_t num_cy = cart_measure_y.cols();

      size_t num_lz = laser_measure_z.cols();
      size_t num_cz = cart_measure_z.cols();

      
      // either laser measures == jnt measures, or jnt measures = laser measures + 8
      bool sizeOK = ((num_lx == num_cx) && (num_ly == num_cy) && (num_lz == num_cz));
      if (cart_measure_x.rows() < 3 || !sizeOK || laser_measure_x.rows() <3 ||
          cart_measure_y.rows() < 3 || laser_measure_y.rows() < 3 ||
          cart_measure_z.rows() < 3 || laser_measure_z.rows() < 3) {
        strs.str("");
        strs << GetName() << ":" << "CalibrateLaserCoplanar: input data dimension is not matching"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
      }

      size_t num_measure_x = 3 * num_lx / 4 ;
      size_t num_measure_y =  3 * num_ly / 4 ;
      size_t num_measure_z = 3 * num_lz / 4 ;
      size_t total_rows = num_measure_x  + num_measure_y + num_measure_z - 9;
    
      if (num_measure_x < 4 || num_measure_y < 4 || num_measure_z < 4) {
        strs.str(""); 
        strs << GetName() << ":" << "CalibrateLaserCoplanar: 3/4 calib samples need at least 4 samples,"
                  << " so can not do calibration, in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_LASER_LESS_SAMPLES;
      }

      // initialize iter alg parameters
      alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                    ridgeScale_, cyc_mod_, sam_region_scale_);
      resetCache_ = true;
      //! before we start iteration, we compute the "supposed" joint arrays
      Eigen::MatrixXd  jnt_x(DoF_, num_cx), jnt_y(DoF_, num_cy), jnt_z(DoF_, num_cz);
                      //jnt_normal_x(DoF_, 3), jnt_normal_y(DoF_, 3), jnt_normal_z(DoF_, 3);
      //! offseted cartesian array (i.e. offset based upon laser measure data)
      Eigen::MatrixXd cart_off_measure_x, cart_off_measure_y, cart_off_measure_z;
      cart_off_measure_x = cart_measure_x;
      cart_off_measure_y = cart_measure_y;
      cart_off_measure_z = cart_measure_z;
      Eigen::VectorXd first_laser, jnt;

      //before doing anything, set using uncalibrated model
      SetUsingCalibratedModel(false);
      // plane parallel to x block surface
      for (size_t i=0; i < num_cx; i++) {
          Eigen::VectorXd cart = cart_measure_x.col(i);
          Eigen::VectorXd laser = laser_measure_x.col(i);
          if (i==0) {
            first_laser = laser;
          } else {
            cart(0) = cart(0) - laser_scale(0) * (laser(0) - first_laser(0));
          }
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_x.col(i) = jnt;
          cart_off_measure_x.col(i) = cart;
      }
      
      // now using xdata 
      Eigen::MatrixXd  xdata = cart_off_measure_x.block(0, 0, 3, num_cx);
      Eigen::Vector3d meanPt = xdata.rowwise().mean();
      Eigen::MatrixXd tmp = xdata.colwise() - meanPt;

      strs.str("");
      strs << "laser scale=" << laser_scale.transpose() << std::endl;
      LOG_INFO(strs);
      strs.str("");
      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      Eigen::MatrixXd tmpV = svd.matrixV();
      Eigen::Vector3d pnx = tmpV.col(2); 
      strs <<"meanPtx=" << meanPt.transpose() << ", pnx=" << pnx.transpose() << std::endl;
      
      // change the first 3pts to be matching with the svd plane
      for (size_t i=0; i<3; i++) {
          Eigen::VectorXd cart = cart_off_measure_x.col(i);
          Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pnx)) * pnx; 
          cart.segment(0, 3) = correct_cart;
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_x.col(i) = jnt;
          cart_off_measure_x.col(i) = cart;
      }

      // plane parallel to y block surface
      for (size_t i=0; i < num_cy; i++) {
          Eigen::VectorXd cart = cart_measure_y.col(i);
          Eigen::VectorXd laser = laser_measure_y.col(i);
          if (i==0) {
            first_laser = laser;
          } else {
            cart(1) = cart(1) - laser_scale(1) * (laser(1) - first_laser(1));
          }
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_y.col(i) = jnt;
          cart_off_measure_y.col(i) = cart;
      }
      
      // now using ydata 
      Eigen::MatrixXd  ydata = cart_off_measure_y.block(0, 0, 3, num_cy);
      meanPt = ydata.rowwise().mean();
      tmp = ydata.colwise() - meanPt;
    

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd1(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd1.matrixV();
      Eigen::Vector3d pny = tmpV.col(2);
      strs << "meanPty=" << meanPt.transpose() << ", pny=" << pny.transpose() << std::endl; 
      // change the first 3pts to be matching with the svd plane
      for (size_t i=0; i<3; i++) {
          Eigen::VectorXd cart = cart_off_measure_y.col(i);
          Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pny)) * pny; 
          cart.segment(0, 3) = correct_cart;
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_y.col(i) = jnt;
          cart_off_measure_y.col(i) = cart;
      }

      // plane parallel to z block surface
      for (size_t i=0; i < num_cz; i++) {
          Eigen::VectorXd cart = cart_measure_z.col(i);
          Eigen::VectorXd laser = laser_measure_z.col(i);
          if (i==0) {
            first_laser = laser;
          } else {
            cart(2) = cart(2) - laser_scale(2) * (laser(2) - first_laser(2));
          }
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_z.col(i) = jnt;
          cart_off_measure_z.col(i) = cart;
      }

      // now using zdata 
      Eigen::MatrixXd  zdata = cart_off_measure_z.block(0, 0, 3, num_cz);
      meanPt = zdata.rowwise().mean();
      tmp = zdata.colwise() - meanPt;

      // using svd decompsition to find the normal of 8-pt plane
      Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
      tmpV = svd2.matrixV();
      Eigen::Vector3d pnz = tmpV.col(2);
      strs << "meanPtz=" << meanPt.transpose() <<", pnz=" << pnz.transpose() << std::endl;
      LOG_INFO(strs);
      // change the first 3pts to be matching with the svd plane
      for (size_t i=0; i<3; i++) {
          Eigen::VectorXd cart = cart_off_measure_z.col(i);
          Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pnz)) * pnz; 
          cart.segment(0, 3) = correct_cart;
          if (!GetJntFromPose(cart, &jnt)) {
            return -1101;  // IK error
          }
          jnt_z.col(i) = jnt;
          cart_off_measure_z.col(i) = cart;
      }

      // in any case, we start with uncaliberated model
      std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                          d_tmp = d_c_, theta_tmp = theta_c_;
      std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                          d_old = d_tmp, theta_old = theta_tmp;
      
      // step 1, using canonical FK to compute the corresponding
      // we need to define one matrix A and one vector b for regression
      // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
      // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
      Eigen::MatrixXd A; //(total_rows + 3, 6 + 5 * DoF_ + 12);
      Eigen::MatrixXd A1(total_rows, 6 + 5 * DoF_ + 12);
      Eigen::MatrixXd A2(3, 6 + 5 * DoF_ + 12);
      Eigen::VectorXd b; //(total_rows + 3);
      Eigen::VectorXd b1(total_rows);
      Eigen::VectorXd b2(3);
      Eigen::Vector3d dv1, dv2, dv3;
      double norm_dv1, norm_dv2, norm_dv3;
      double previous_err = std::numeric_limits<double>::max();
      double estimation_err = 0.5 * previous_err;
      double init_estimation_err = 0;
      double previous_diff = previous_err - estimation_err;
      double current_diff = previous_diff / 2.0;
      int cur_iter = 0;
      double init_angle_xy =0, init_angle_yz=0, init_angle_zx=0,
            init_angle_xy1 =0, init_angle_yz1=0, init_angle_zx1=0;
      while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
            previous_err - estimation_err > MAX_CALIB_MATCHING_ERR_COPLANAR || resetCache_)
            && cur_iter < MAX_CALIB_OUTER_ITER
            // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
            ) {
        strs.str("");
        strs << "previous_err=" << previous_err << ", estimation_err=" << estimation_err <<
            ", resetCache=" <<  !resetCache_ << ", diff_err=" << previous_err - estimation_err << std::endl;
        LOG_INFO(strs);
        // assign previous value
        if (resetCache_) {
          a_old = a_tmp; alpha_old = alpha_tmp;
          d_old = d_tmp; theta_old = theta_tmp;
          beta_old  = beta_tmp;
          //tool_old = tool_tmp; base_old = base_tmp;
        } else {
          previous_err = estimation_err;
        }
        previous_diff = current_diff;
        cur_iter++;
        std::vector<double> kine_para, tmp_para; // clearing kine_para
        // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
        kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
        kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
        kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
        kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
        kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
        //estimation_err = 0;
      
        double estimation_err3 = 0;
        double estimation_err1 = 0;
        // before optimization, A and b need to resize, because its dep. columns is removed during opt.
        // A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
        A1.resize(total_rows, 6 + 5 * DoF_ + 12);
        A2.resize(3, 6 + 5 * DoF_ + 12);

        // pp and J vector are for computing the items in A matrix, and b vector
        std::vector<Eigen::Vector3d> pp(4);
        std::vector<Eigen::MatrixXd> J(4);
        Eigen::MatrixXd tmpJ(3, 5 * DoF_);
        Eigen::MatrixXd dJ1, dJ2, dJ3;

        // now we add constraints of surface normal angles
        // pp and J vector are for computing the items in A matrix, and b vector
        std::vector<Eigen::Vector3d> nn(3), va(3), vb(3);
        std::vector<Eigen::MatrixXd> dJx(2), dJy(2), dJz(2);
        std::vector<double> norm_nn(3);

        size_t start_matrix_rows = 0;
        // first plane constraint equation
        for (size_t i=0; i < num_measure_x; i++) {
          UpdateDH(kine_para, jnt_x.col(i), &tmp_para);
          Eigen::MatrixXd Jp_t, Jp_r;
          Pose p;
          // compute the expected values from known canonical kinematic parameters
          // i.e., not-calibrated parameter set
          int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
          if (ret < 0) {
            return ret;
          }
          Vec  tf = p.getTranslation();
          Eigen::Matrix3d t_e_hat;
          tf.ToHat(&t_e_hat);
          tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
          if (i < 3) {
            J[i] = tmpJ;
            pp[i] = tf.ToEigenVec();
            if (i == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 =  dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 =  dv2.norm();
              dJ1 = J[1] - J[0];
              dJ2 = J[2] - J[1];


              va[0] = dv1;
              vb[0] = dv2;
              dJx[0] = dJ1;
              dJx[1] = dJ2;
              nn[0] = va[0].cross(vb[0]);
              norm_nn[0] = nn[0].norm();
            }
          } else {
            J[3] = tmpJ;
            pp[3] = tf.ToEigenVec();
            dJ3 = J[3] - J[2];
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();

            // computing the Jacobian
            A1.block((i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
            Eigen::Matrix3d AA;
            for (size_t j=0; j< 5 * DoF_; j++) {
                  double sum = 0;
                  AA.col(0) = dJ1.col(j) / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dv3 / norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;;
                  AA.col(1) = dJ2.col(j) / norm_dv2;
                  AA.col(2) = dv3 /  norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dJ3.col(j) / norm_dv3;
                  sum += AA.determinant();
                  A1(i -3, j + 6) = sum;
                  //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
            }
            A1.block(i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 /  norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            b1(i - 3) = - AA.determinant();
            estimation_err1 += fabs(b1(i - 3));
          }
        }
        start_matrix_rows += num_measure_x - 3;
        
        // second plane (y plane) constraint equation
        for (size_t i=0; i < num_measure_y; i++) {
          UpdateDH(kine_para, jnt_y.col(i), &tmp_para);
          Eigen::MatrixXd Jp_t, Jp_r;
          Pose p;
          // compute the expected values from known canonical kinematic parameters
          // i.e., not-calibrated parameter set
          int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
          if (ret < 0) {
            return ret;
          }
          Vec  tf = p.getTranslation();
          Eigen::Matrix3d t_e_hat;
          tf.ToHat(&t_e_hat);
          tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
          if (i < 3) {
            J[i] = tmpJ;
            pp[i] = tf.ToEigenVec();
            if (i == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 =  dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 = dv2.norm();
              dJ1 = J[1] - J[0];
              dJ2 = J[2] - J[1];

              va[1] = dv1;
              vb[1] = dv2;
              dJy[0] = dJ1;
              dJy[1] = dJ2;
              nn[1] = va[1].cross(vb[1]);
              norm_nn[1] = nn[1].norm();
            }
          } else {
            J[3] = tmpJ;
            pp[3] = tf.ToEigenVec();
            dJ3 = J[3] - J[2];
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();

            // computing the Jacobian
            A1.block(start_matrix_rows + (i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
            Eigen::Matrix3d AA;
            for (size_t j=0; j< 5 * DoF_; j++) {
                  double sum = 0;
                  AA.col(0) = dJ1.col(j) / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dv3 / norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;;
                  AA.col(1) = dJ2.col(j) / norm_dv2;
                  AA.col(2) = dv3 /  norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dJ3.col(j) / norm_dv3;
                  sum += AA.determinant();
                  A1(start_matrix_rows + i -3, j + 6) = sum;
                  //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
            }
            A1.block(start_matrix_rows + i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 /  norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            b1(start_matrix_rows + i - 3) = - AA.determinant();
            estimation_err1 += fabs(b1(start_matrix_rows + i - 3));
          }
        }
        start_matrix_rows += num_measure_y - 3;

        // third plane (z plane) constraint equation
        for (size_t i=0; i < num_measure_z; i++) {
          UpdateDH(kine_para, jnt_z.col(i), &tmp_para);
          Eigen::MatrixXd Jp_t, Jp_r;
          Pose p;
          // compute the expected values from known canonical kinematic parameters
          // i.e., not-calibrated parameter set
          int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
          if (ret < 0) {
            return ret;
          }
          Vec  tf = p.getTranslation();
          Eigen::Matrix3d t_e_hat;
          tf.ToHat(&t_e_hat);
          tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
          if (i < 3) {
            J[i] = tmpJ;
            pp[i] = tf.ToEigenVec();
            if (i == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 = dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 =  dv2.norm();
              dJ1 = J[1] - J[0];
              dJ2 = J[2] - J[1];

              va[2] = dv1;
              vb[2] = dv2;
              dJz[0] = dJ1;
              dJz[1] = dJ2;
              nn[2] = va[2].cross(vb[2]);
              norm_nn[2] = nn[2].norm();
            }
          } else {
            J[3] = tmpJ;
            pp[3] = tf.ToEigenVec();
            dJ3 = J[3] - J[2];
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();

            // computing the Jacobian
            A1.block(start_matrix_rows + (i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
            Eigen::Matrix3d AA;
            for (size_t j=0; j< 5 * DoF_; j++) {
                  double sum = 0;
                  AA.col(0) = dJ1.col(j) / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dv3 / norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;;
                  AA.col(1) = dJ2.col(j) / norm_dv2;
                  AA.col(2) = dv3 /  norm_dv3;
                  sum += AA.determinant();
                  AA.col(0) = dv1 / norm_dv1;
                  AA.col(1) = dv2 / norm_dv2;
                  AA.col(2) = dJ3.col(j) / norm_dv3;
                  sum += AA.determinant();
                  A1(start_matrix_rows + i -3, j + 6) = sum;
                  //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
            }
            A1.block(start_matrix_rows + i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 /  norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            b1(start_matrix_rows + i - 3) = - AA.determinant();
            estimation_err1 += fabs(b1(start_matrix_rows + i - 3));
          }
        }

        start_matrix_rows += num_measure_z - 3;
        
        // estimatation error for normal part
        double  estimation_err2 = 0;
        
        // now computing extended A and b for normal constraints  
        Vec vb2(vb[1]), va2(va[1]), vb1(vb[0]), va1(va[0]), vb3(vb[2]), va3(va[2]);
        Eigen::Matrix3d vb2head, va2head, vb1head, va1head, vb3head, va3head;
        vb2.ToHat(&vb2head);
        va2.ToHat(&va2head);
        va1.ToHat(&va1head);
        vb1.ToHat(&vb1head);
        va3.ToHat(&va3head);
        vb3.ToHat(&vb3head);
        Eigen::VectorXd vj1 = -dJy[0].transpose() * vb2head.transpose() * nn[0];
        Eigen::VectorXd vj2 = dJy[1].transpose() * va2head.transpose() * nn[0];
        Eigen::VectorXd vj3 = -dJx[0].transpose() * vb1head.transpose() * nn[1];
        Eigen::VectorXd vj4 = dJx[1].transpose() * va1head.transpose() * nn[1];

        b2(0) = -nn[0].dot(nn[1]) / (norm_nn[0] * norm_nn[1]);
        estimation_err2 += fabs(b2(0));
        A2.block(0, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
        A2.block(0, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[0] * norm_nn[1]);
        A2.block(0, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);

        vj1 = -dJy[0].transpose() * vb2head.transpose() * nn[2];
        vj2 = dJy[1].transpose() * va2head.transpose() * nn[2];
        vj3 = -dJz[0].transpose() * vb3head.transpose() * nn[1];
        vj4 = dJz[1].transpose() * va3head.transpose() * nn[1];
        b2(1) = -nn[1].dot(nn[2]) / (norm_nn[1] * norm_nn[2]);
        estimation_err2 += fabs(b2(1));
        A2.block(1, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
        A2.block(1, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[1] * norm_nn[2]);
        A2.block(1, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
        
        vj1 = -dJx[0].transpose() * vb1head.transpose() * nn[2];
        vj2 = dJx[1].transpose() * va1head.transpose() * nn[2];
        vj3 = -dJz[0].transpose() * vb3head.transpose() * nn[0];
        vj4 = dJz[1].transpose() * va3head.transpose() * nn[0];
        b2(2) = -nn[0].dot(nn[2]) / (norm_nn[2] * norm_nn[0]);
        estimation_err2 += fabs(b2(2));

        strs.str("");
        strs << "Plane angles: XY=" << acos(fabs(b2(0))) * 180 / M_PI
        << ", YZ=" << acos(fabs(b2(1))) * 180 / M_PI
        << ", ZX="  << acos(fabs(b2(2))) * 180 / M_PI << std::endl;
        LOG_INFO(strs); 
        if (cur_iter <= 1) {
          init_angle_xy = acos(fabs(b2(0)));
          init_angle_yz = acos(fabs(b2(1)));
          init_angle_zx = acos(fabs(b2(2)));
          if (nn[0].dot(pnx) < 0) {
            pnx = -pnx;
          }
          if (nn[1].dot(pny) < 0) {
            pny = -pny;
          }
          if (nn[2].dot(pnz) < 0) {
            pnz=-pnz;
          }
          init_angle_xy1 = acos(fabs(pnx.dot(pny)));
          init_angle_yz1 = acos(fabs(pny.dot(pnz)));
          init_angle_zx1 = acos(fabs(pnz.dot(pnx)));
        }
        A2.block(2, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
        A2.block(2, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[2] * norm_nn[0]);
        A2.block(2, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);

        // compute A, b, and average estimation error for a single measurement
        
        if (coplanar_normal_option_ == 0) {  // using coplanar only
            A = A1;
            b = b1;
            estimation_err3 = estimation_err1 / start_matrix_rows; 
        } else if (coplanar_normal_option_ == 1) {
          A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
          b.resize(total_rows + 3);
          A.block(0, 0, total_rows,  6 + 5 * DoF_ + 12) = A1;
          A.block(total_rows, 0, 3,  6 + 5 * DoF_ + 12) = A2;
          b.block(0, 0, total_rows, 1) = b1;
          b.block(total_rows, 0, 3, 1) = b2;
          estimation_err3 = (estimation_err1 + estimation_err2) / (start_matrix_rows + 3);
        } else if (coplanar_normal_option_ >= 2) {   // using normal only
          A = A2;
          b = b2;
          estimation_err3 = estimation_err2 / 3;
        }
        strs.str("");
        strs << GetName() << ":" << "estimation_err z "<< estimation_err3 <<  std::endl;
        LOG_INFO(strs);
        if (cur_iter <= 1) {
          init_estimation_err = estimation_err3;
        }
        current_diff = std::min(previous_err - estimation_err,
                std::numeric_limits<double>::max() / 8.0);
        // this is to remove the impacts from some redundant parameters
        for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
          // luckily, the vector of depend column indices in d_jacobian_cols_
          // are arranged from largest toward smallest, so we can continuously
          // call removeColumn
          removeColumn(&A, d_jacobian_cols_[i]);
        }
        // using RNN to compute the best delta_para given the current para
        // RNN used as an internal loop
        strs.str("");
        strs << GetName() << ":" << "Calib. through laser sensor, iteration no. = " << cur_iter << std::endl;
        LOG_INFO(strs);
        Eigen::VectorXd delta_p_old;
        Eigen::VectorXd base_tmp(6);
        base_tmp.setZero();
        Eigen::Vector3d tool_tmp(3);
        tool_tmp.setZero();

        
        if (resetCache_) {
          size_t numParam = A.cols();
          delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
        }
        if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
          if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
          }
          delta_p_old_cache_ = delta_p_old;
          UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = false;
          estimation_err = estimation_err3;
        } else {
          UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                          &a_tmp, &theta_tmp, &d_tmp,
                          &beta_tmp, &tool_tmp, NULL);
          resetCache_ = true;
          estimation_err = previous_err - 1.5 * MAX_ORIENT_CALIB_MATCHING_ERR;
        }                   

      }
      if (cur_iter >= MAX_CALIB_OUTER_ITER) {
          strs.str("");
          strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                  << "With estimation error " << estimation_err
                  << std::endl;
          LOG_ERROR(strs);
          //return -ERR_CALIB_REG_MAX_ITER;
      }
      if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches estimation_err " << estimation_err
                  << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                  << std::endl;
          LOG_ERROR(strs);
      }
      if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
          strs.str("");
          strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches err diff " << previous_err - estimation_err
                  << ",prev_err=" << previous_err << ", cur_err=" << estimation_err <<  ", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                  << std::endl;
          LOG_ERROR(strs);
      }
      if (init_estimation_err > 0 && estimation_err < init_estimation_err) {
        alpha_c_ = alpha_old;
        a_c_ = a_old;
        theta_c_ = theta_old;
        d_c_ = d_old;
        beta_c_ = beta_old;
      }
      strs.str("");
      strs << GetName() << ":" << "alpha_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << alpha_c_[k] << " ";
      }
      strs << ", a_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << a_c_[k] << " ";
      }
      strs << ", theta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << theta_c_[k] << " ";
      }
      strs << ", d_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << d_c_[k] << " ";
      }
      strs << ", beta_c: ";
      for (size_t k=0; k < DoF_; k++) {
        strs << beta_c_[k] << " ";
      }
      strs << std::endl;
      strs << "init plane angles:XY=" << init_angle_xy * 180 / M_PI << ", from svd=" << init_angle_xy1 * 180 / M_PI 
        << ", YZ=" << init_angle_yz * 180 / M_PI << ", from svd=" << init_angle_yz1 * 180 / M_PI <<
          ", ZX=" << init_angle_zx *180 / M_PI << ", from svd=" << init_angle_zx1 *180 / M_PI << std::endl;
      strs << " final plane angles: XY=" << acos(fabs(b2(0))) * 180 / M_PI
        << ", YZ=" << acos(fabs(b2(1))) * 180 / M_PI
        << ", ZX="  << acos(fabs(b2(2))) * 180 / M_PI <<
      ",final matching error=" << previous_err << std::endl;
      LOG_INFO(strs);
    
      isDHCalibrated_ = true;

      Eigen::MatrixXd cartx = cart_off_measure_x.block(0, 0, 3, num_cx);
      Eigen::MatrixXd carty = cart_off_measure_y.block(0, 0, 3, num_cy);
      Eigen::MatrixXd cartz = cart_off_measure_z.block(0, 0, 3, num_cz);
      // not goes to testing phase: check if the regressed model also fit to other poits
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
      kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
      kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
      kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
      kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

      // end-effector cartesian position from calibrated model, and original models
      std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
      std::vector<Eigen::Vector3d> nn_orig(3), nn(3);
      std::vector<double> norm_nn_orig(3), norm_nn(3);
      double orig_err1 = 0, comp_err1=0, orig_err2=0, comp_err2=0;
      Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
      double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;

      // compare  the planar error before calib and after comp.
      // first plane constraint equation
      size_t numValid = 0;
      size_t numX = 0; // num_measure_x;
      for (size_t i=numX; i < num_cx; i++) {
        UpdateDH(kine_para, jnt_x.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        size_t j = i - numX;
        if (j < 3) {
            pp[j] = tf.ToEigenVec();
            if (j == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 =dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 =dv2.norm();
              nn[0] = dv1.cross(dv2);
              norm_nn[0] = nn[0].norm();

              dv1_orig = cartx.col(numX + 1) - cartx.col(numX);
              norm_dv1_orig = dv1_orig.norm();
              dv2_orig = cartx.col(numX + 2) - cartx.col(numX + 1);
              norm_dv2_orig = dv2_orig.norm();
              nn_orig[0] = dv1_orig.cross(dv2_orig);
              norm_nn_orig[0] = nn_orig[0].norm();
            }
        } else {
            pp[3] = tf.ToEigenVec();
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();
            dv3_orig = cartx.col(i) - cartx.col(numX + 2);
            norm_dv3_orig = dv3_orig.norm();
          
            Eigen::Matrix3d AA;
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 / norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            comp_err1 += fabs(AA.determinant());
            AA.col(0) = dv1_orig / norm_dv1_orig;
            AA.col(1) = dv2_orig / norm_dv2_orig;
            AA.col(2) = dv3_orig / norm_dv3_orig;
            orig_err1 += fabs(AA.determinant());
        }
      }
      numValid += num_cx - numX - 3;
      size_t numY = 0;  //num_measure_y;
      // second plane (y plane) constraint equation
      for (size_t i=numY; i < num_cy; i++) {
        UpdateDH(kine_para, jnt_y.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        size_t j = i - numY;
        if (j < 3) {
            pp[j] = tf.ToEigenVec();
            if (j == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 =dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 =dv2.norm();
              nn[1] = dv1.cross(dv2);
              norm_nn[1] = nn[1].norm();

              dv1_orig = carty.col(numY + 1) - carty.col(numY);
              norm_dv1_orig = dv1_orig.norm();
              dv2_orig = carty.col(numY + 2) - carty.col(numY + 1);
              norm_dv2_orig = dv2_orig.norm();
              nn_orig[1] = dv1_orig.cross(dv2_orig);
              norm_nn_orig[1] = nn_orig[1].norm();
            }
        } else {
            pp[3] = tf.ToEigenVec();
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();
            dv3_orig = carty.col(i) - carty.col(numY + 2);
            norm_dv3_orig = dv3_orig.norm();
          
            Eigen::Matrix3d AA;
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 / norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            comp_err1 += fabs(AA.determinant());
            AA.col(0) = dv1_orig / norm_dv1_orig;
            AA.col(1) = dv2_orig / norm_dv2_orig;
            AA.col(2) = dv3_orig / norm_dv3_orig;
            orig_err1 += fabs(AA.determinant());
        }
      }
      numValid += num_cy - numY - 3;
      size_t numZ = 0; // num_measure_z;
      // third plane (z plane) constraint equation
      for (size_t i=numZ; i < num_cz; i++) {
        UpdateDH(kine_para, jnt_z.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        size_t j = i - numZ;
        if (j < 3) {
            pp[j] = tf.ToEigenVec();
            if (j == 2) {
              dv1= pp[1] - pp[0];
              norm_dv1 =dv1.norm();
              dv2= pp[2] - pp[1];
              norm_dv2 =dv2.norm();
              nn[2] = dv1.cross(dv2);
              norm_nn[2] = nn[2].norm();

              dv1_orig = cartz.col(numZ + 1) - cartz.col(numZ);
              norm_dv1_orig = dv1_orig.norm();
              dv2_orig = cartz.col(numZ + 2) - cartz.col(numZ + 1);
              norm_dv2_orig = dv2_orig.norm();
              nn_orig[2] = dv1_orig.cross(dv2_orig);
              norm_nn_orig[2] = nn_orig[2].norm();
            }
        } else {
            pp[3] = tf.ToEigenVec();
            dv3 = pp[3] - pp[2];
            norm_dv3 = dv3.norm();
            dv3_orig = cartz.col(i) - cartz.col(numZ + 2);
            norm_dv3_orig = dv3_orig.norm();
          
            Eigen::Matrix3d AA;
            AA.col(0) = dv1 / norm_dv1;
            AA.col(1) = dv2 / norm_dv2;
            AA.col(2) = dv3 / norm_dv3;
            comp_err1 += fabs(AA.determinant());
            AA.col(0) = dv1_orig / norm_dv1_orig;
            AA.col(1) = dv2_orig / norm_dv2_orig;
            AA.col(2) = dv3_orig / norm_dv3_orig;
            orig_err1 += fabs(AA.determinant());
        }
      }
      numValid += num_cz - numZ - 3;
      
      // error comparison for normal methods
      orig_err2 += fabs(nn_orig[0].dot(nn_orig[1]) / (norm_nn_orig[0] * norm_nn_orig[1]));
      orig_err2 += fabs(nn_orig[1].dot(nn_orig[2]) / (norm_nn_orig[1] * norm_nn_orig[2]));
      orig_err2 += fabs(nn_orig[0].dot(nn_orig[2]) / (norm_nn_orig[0] * norm_nn_orig[2]));

        
      comp_err2 += fabs(nn[0].dot(nn[1]) / (norm_nn[0] * norm_nn[1]));
      comp_err2 += fabs(nn[1].dot(nn[2]) / (norm_nn[1] * norm_nn[2]));
      comp_err2 += fabs(nn[0].dot(nn[2]) / (norm_nn[0] * norm_nn[2]));

      strs.str("");
      strs << "orig_err1=" << orig_err1 / numValid << "comp_err1=" << comp_err1 / numValid << std::endl;
      strs << "orig_err2=" << orig_err2 / 3 << "comp_err2=" << comp_err2 / 3 << std::endl;
      LOG_INFO(strs);
    
      double avg_orig_err =0, avg_comp_err = 0;
      if (coplanar_normal_option_ == 0) {  // using coplanar only
        avg_orig_err = orig_err1 / numValid;
        avg_comp_err = comp_err1 / numValid;    
      } else if (coplanar_normal_option_ == 1) {
        avg_orig_err = (orig_err1 + orig_err2) / (numValid +3);
        avg_comp_err = (comp_err1 + comp_err2) / (numValid +3);
      } else if (coplanar_normal_option_ >= 2) {   // using normal only
        avg_orig_err = orig_err2 / 3;
        avg_comp_err = comp_err2 / 3;
      }
    
      //double overall_comp_err = (previous_err * total_rows + orig_err) / (total_rows + numValid);  // balanced overall comp err
      // average estimation error for a single measurement
      strs.str("");
      strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
          << ", after comp. the error = " << avg_comp_err << std::endl;
      LOG_INFO(strs);
      if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        // return -agv_comp_err;
      }
      return avg_comp_err;
  } catch (...) {
    strs.str("");
    strs << GetName() << ": " << __FUNCTION__ << ", got exception" << std::endl;
    LOG_ERROR(strs);
    return -1111;
  }
}

double serialArm::LaserCalibrateCoplanar(
       const Eigen::MatrixXd &cart_measure_x,  // cartesian coordinates reported from robot
       const Eigen::MatrixXd &cart_measure_y,
       const Eigen::MatrixXd &cart_measure_z,
       const Eigen::MatrixXd &laser_measure_x,
       const Eigen::MatrixXd &laser_measure_y,
       const Eigen::MatrixXd &laser_measure_z,
       const Eigen::Vector3d &laser_scale) {
     // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }

    size_t num_lx = laser_measure_x.cols();
    size_t num_cx = cart_measure_x.cols();

    size_t num_ly = laser_measure_y.cols();
    size_t num_cy = cart_measure_y.cols();

    size_t num_lz = laser_measure_z.cols();
    size_t num_cz = cart_measure_z.cols();

    
    // either laser measures == jnt measures, or jnt measures = laser measures + 8
    bool sizeOK = ((num_lx == num_cx) && (num_ly == num_cy) && (num_lz == num_cz));
    if (cart_measure_x.rows() < 3 || !sizeOK || laser_measure_x.rows() <3 ||
        cart_measure_y.rows() < 3 || laser_measure_y.rows() < 3 ||
        cart_measure_z.rows() < 3 || laser_measure_z.rows() < 3) {
      strs.str("");
      strs << GetName() << ":" << "CalibrateLaserCoplanar: input data dimension is not matching"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }

    size_t num_measure_x = 3 * num_lx / 4 ;
    size_t num_measure_y =  3 * num_ly / 4 ;
    size_t num_measure_z = 3 * num_lz / 4 ;
    size_t total_rows = num_measure_x  + num_measure_y + num_measure_z - 9;
  
    if (num_measure_x < 4 || num_measure_y < 4 || num_measure_z < 4) {
      strs.str(""); 
      strs << GetName() << ":" << "CalibrateLaserCoplanar: 3/4 calib samples need at least 4 samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }

    // initialize iter alg parameters
    alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                  ridgeScale_, cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    //! before we start iteration, we compute the "supposed" joint arrays
    Eigen::MatrixXd  jnt_x(DoF_, num_cx), jnt_y(DoF_, num_cy), jnt_z(DoF_, num_cz);
                     //jnt_normal_x(DoF_, 3), jnt_normal_y(DoF_, 3), jnt_normal_z(DoF_, 3);
    //! offseted cartesian array (i.e. offset based upon laser measure data)
    Eigen::MatrixXd cart_off_measure_x, cart_off_measure_y, cart_off_measure_z;
    cart_off_measure_x = cart_measure_x;
    cart_off_measure_y = cart_measure_y;
    cart_off_measure_z = cart_measure_z;
    Eigen::VectorXd first_laser, jnt;

    //before doing anything, set using uncalibrated model
    SetUsingCalibratedModel(false);
    // plane parallel to x block surface
    for (size_t i=0; i < num_cx; i++) {
        Eigen::VectorXd cart = cart_measure_x.col(i);
        Eigen::VectorXd laser = laser_measure_x.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(0) = cart(0) - laser_scale(0) * (laser(0) - first_laser(0));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_x.col(i) = jnt;
        cart_off_measure_x.col(i) = cart;
    }
    
    // now using xdata 
    Eigen::MatrixXd  xdata = cart_off_measure_x.block(0, 0, 3, num_cx);
    Eigen::Vector3d meanPt = xdata.rowwise().mean();
    Eigen::MatrixXd tmp = xdata.colwise() - meanPt;

    strs.str("");
    strs << "laser scale=" << laser_scale.transpose() << std::endl;
    LOG_INFO(strs);
    strs.str("");
    // using svd decompsition to find the normal of 8-pt plane
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::MatrixXd tmpV = svd.matrixV();
    Eigen::Vector3d pnx = tmpV.col(2); 
    strs <<"meanPtx=" << meanPt.transpose() << ", pnx=" << pnx.transpose() << std::endl;

     // change the first 3pts to be matching with the svd plane
    for (size_t i=0; i<3; i++) {
        Eigen::VectorXd cart = cart_off_measure_x.col(i);
        Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pnx)) * pnx; 
        cart.segment(0, 3) = correct_cart;
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_x.col(i) = jnt;
        cart_off_measure_x.col(i) = cart;
    }

    // plane parallel to y block surface
    for (size_t i=0; i < num_cy; i++) {
        Eigen::VectorXd cart = cart_measure_y.col(i);
        Eigen::VectorXd laser = laser_measure_y.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(1) = cart(1) - laser_scale(1) * (laser(1) - first_laser(1));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_y.col(i) = jnt;
        cart_off_measure_y.col(i) = cart;
    }
    
    // now using ydata 
    Eigen::MatrixXd  ydata = cart_off_measure_y.block(0, 0, 3, num_cy);
    meanPt = ydata.rowwise().mean();
    tmp = ydata.colwise() - meanPt;
  

    // using svd decompsition to find the normal of 8-pt plane
    Eigen::JacobiSVD<Eigen::MatrixXd> svd1(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    tmpV = svd1.matrixV();
    Eigen::Vector3d pny = tmpV.col(2);
    strs << "meanPty=" << meanPt.transpose() << ", pny=" << pny.transpose() << std::endl;

    // change the first 3pts to be matching with the svd plane
    for (size_t i=0; i<3; i++) {
        Eigen::VectorXd cart = cart_off_measure_y.col(i);
        Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pny)) * pny; 
        cart.segment(0, 3) = correct_cart;
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_y.col(i) = jnt;
        cart_off_measure_y.col(i) = cart;
    }
   
    // plane parallel to z block surface
    for (size_t i=0; i < num_cz; i++) {
        Eigen::VectorXd cart = cart_measure_z.col(i);
        Eigen::VectorXd laser = laser_measure_z.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(2) = cart(2) - laser_scale(2) * (laser(2) - first_laser(2));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_z.col(i) = jnt;
        cart_off_measure_z.col(i) = cart;
    }

    // now using zdata 
    Eigen::MatrixXd  zdata = cart_off_measure_z.block(0, 0, 3, num_cz);
    meanPt = zdata.rowwise().mean();
    tmp = zdata.colwise() - meanPt;

    // using svd decompsition to find the normal of 8-pt plane
    Eigen::JacobiSVD<Eigen::MatrixXd> svd2(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    tmpV = svd2.matrixV();
    Eigen::Vector3d pnz = tmpV.col(2);
    strs << "meanPtz=" << meanPt.transpose() <<", pnz=" << pnz.transpose() << std::endl;
    LOG_INFO(strs);

    // change the first 3pts to be matching with the svd plane
    for (size_t i=0; i<3; i++) {
        Eigen::VectorXd cart = cart_off_measure_z.col(i);
        Eigen::Vector3d correct_cart = cart.segment(0,3) - ((cart.segment(0,3) - meanPt).dot(pnz)) * pnz; 
        cart.segment(0, 3) = correct_cart;
        if (!GetJntFromPose(cart, &jnt)) {
          return -1101;  // IK error
        }
        jnt_z.col(i) = jnt;
        cart_off_measure_z.col(i) = cart;
    }

    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, alpha_tmp = alpha_c_, beta_tmp = beta_c_,
                        d_tmp = d_c_, theta_tmp = theta_c_;
    std::vector<double> a_old = a_tmp, alpha_old = alpha_tmp, beta_old = beta_tmp,
                        d_old = d_tmp, theta_old = theta_tmp;
    
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
    Eigen::MatrixXd A; //(total_rows + 3, 6 + 5 * DoF_ + 12);
    Eigen::MatrixXd A1(total_rows, 6 + 5 * DoF_ + 12);
    Eigen::MatrixXd A2(3, 6 + 5 * DoF_ + 12);
    Eigen::VectorXd b; //(total_rows + 3);
    Eigen::VectorXd b1(total_rows);
    Eigen::VectorXd b2(3);
    Eigen::Vector3d dv1, dv2, dv3;
    double norm_dv1, norm_dv2, norm_dv3;
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double init_estimation_err = 0.0;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    double init_angle_xy =0, init_angle_yz=0, init_angle_zx=0,
           init_angle_xy1 =0, init_angle_yz1=0, init_angle_zx1=0;
    while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
          previous_err - estimation_err > MAX_CALIB_MATCHING_ERR_COPLANAR || resetCache_)
          && cur_iter < MAX_CALIB_OUTER_ITER
          // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
          ) {
      strs.str("");
      strs << "previous_err=" << previous_err << ", estimation_err=" << estimation_err <<
           ", resetCache=" <<  !resetCache_ << ", diff_err=" << previous_err - estimation_err << std::endl;
      LOG_INFO(strs);
      // assign previous value
      if (resetCache_) {
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        //tool_old = tool_tmp; base_old = base_tmp;
      } else {
        previous_err = estimation_err;
      }
      previous_diff = current_diff;
      cur_iter++;
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      //estimation_err = 0;
    
      double estimation_err1 = 0;
      double estimation_err3 = 0;
      // before optimization, A and b need to resize, because its dep. columns is removed during opt.
      // A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
      A1.resize(total_rows, 6 + 5 * DoF_ + 12);
      A2.resize(3, 6 + 5 * DoF_ + 12);

      // pp and J vector are for computing the items in A matrix, and b vector
      std::vector<Eigen::Vector3d> pp(4);
      std::vector<Eigen::MatrixXd> J(4);
      Eigen::MatrixXd tmpJ(3, 5 * DoF_);
      Eigen::MatrixXd dJ1, dJ2, dJ3;

      // now we add constraints of surface normal angles
      // pp and J vector are for computing the items in A matrix, and b vector
      std::vector<Eigen::Vector3d> nn(3), va(3), vb(3);
      std::vector<Eigen::MatrixXd> dJx(2), dJy(2), dJz(2);
      std::vector<double> norm_nn(3);

      size_t start_matrix_rows = 0;
      // first plane constraint equation
      for (size_t i=0; i < num_measure_x; i++) {
        UpdateDH(kine_para, jnt_x.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        Eigen::Matrix3d t_e_hat;
        tf.ToHat(&t_e_hat);
        tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
        if (i < 3) {
          J[i] = tmpJ;
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =  dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =  dv2.norm();
            dJ1 = J[1] - J[0];
            dJ2 = J[2] - J[1];


            va[0] = dv1;
            vb[0] = dv2;
            dJx[0] = dJ1;
            dJx[1] = dJ2;
            nn[0] = va[0].cross(vb[0]);
            norm_nn[0] = nn[0].norm();
          }
        } else {
          J[3] = tmpJ;
          pp[3] = tf.ToEigenVec();
          dJ3 = J[3] - J[2];
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();

          // computing the Jacobian
          A1.block((i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
          Eigen::Matrix3d AA;
          for (size_t j=0; j< 5 * DoF_; j++) {
                double sum = 0;
                AA.col(0) = dJ1.col(j) / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dv3 / norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;;
                AA.col(1) = dJ2.col(j) / norm_dv2;
                AA.col(2) = dv3 /  norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dJ3.col(j) / norm_dv3;
                sum += AA.determinant();
                A1(i -3, j + 6) = sum;
                //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
          }
          A1.block(i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 /  norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          b1(i - 3) = - AA.determinant();
          estimation_err1 += fabs(b1(i - 3));
        }
      }
      start_matrix_rows += num_measure_x - 3;
      
      // second plane (y plane) constraint equation
      for (size_t i=0; i < num_measure_y; i++) {
        UpdateDH(kine_para, jnt_y.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        Eigen::Matrix3d t_e_hat;
        tf.ToHat(&t_e_hat);
        tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
        if (i < 3) {
          J[i] = tmpJ;
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =  dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 = dv2.norm();
            dJ1 = J[1] - J[0];
            dJ2 = J[2] - J[1];

            va[1] = dv1;
            vb[1] = dv2;
            dJy[0] = dJ1;
            dJy[1] = dJ2;
            nn[1] = va[1].cross(vb[1]);
            norm_nn[1] = nn[1].norm();
          }
        } else {
          J[3] = tmpJ;
          pp[3] = tf.ToEigenVec();
          dJ3 = J[3] - J[2];
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();

          // computing the Jacobian
          A1.block(start_matrix_rows + (i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
          Eigen::Matrix3d AA;
          for (size_t j=0; j< 5 * DoF_; j++) {
                double sum = 0;
                AA.col(0) = dJ1.col(j) / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dv3 / norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;;
                AA.col(1) = dJ2.col(j) / norm_dv2;
                AA.col(2) = dv3 /  norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dJ3.col(j) / norm_dv3;
                sum += AA.determinant();
                A1(start_matrix_rows + i -3, j + 6) = sum;
                //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
          }
          A1.block(start_matrix_rows + i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 /  norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          b1(start_matrix_rows + i - 3) = - AA.determinant();
          estimation_err1 += fabs(b1(start_matrix_rows + i - 3));
        }
      }
      start_matrix_rows += num_measure_y - 3;

      // third plane (z plane) constraint equation
      for (size_t i=0; i < num_measure_z; i++) {
        UpdateDH(kine_para, jnt_z.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        Vec  tf = p.getTranslation();
        Eigen::Matrix3d t_e_hat;
        tf.ToHat(&t_e_hat);
        tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
        if (i < 3) {
          J[i] = tmpJ;
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 = dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =  dv2.norm();
            dJ1 = J[1] - J[0];
            dJ2 = J[2] - J[1];

            va[2] = dv1;
            vb[2] = dv2;
            dJz[0] = dJ1;
            dJz[1] = dJ2;
            nn[2] = va[2].cross(vb[2]);
            norm_nn[2] = nn[2].norm();
          }
        } else {
          J[3] = tmpJ;
          pp[3] = tf.ToEigenVec();
          dJ3 = J[3] - J[2];
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();

          // computing the Jacobian
          A1.block(start_matrix_rows + (i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
          Eigen::Matrix3d AA;
          for (size_t j=0; j< 5 * DoF_; j++) {
                double sum = 0;
                AA.col(0) = dJ1.col(j) / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dv3 / norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;;
                AA.col(1) = dJ2.col(j) / norm_dv2;
                AA.col(2) = dv3 /  norm_dv3;
                sum += AA.determinant();
                AA.col(0) = dv1 / norm_dv1;
                AA.col(1) = dv2 / norm_dv2;
                AA.col(2) = dJ3.col(j) / norm_dv3;
                sum += AA.determinant();
                A1(start_matrix_rows + i -3, j + 6) = sum;
                //strs << "A(" << (num_measures - 1) * 3 + i << ", " << j  << ")=" << sum << std::endl; 
          }
          A1.block(start_matrix_rows + i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 /  norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          b1(start_matrix_rows + i - 3) = - AA.determinant();
          estimation_err1 += fabs(b1(start_matrix_rows + i - 3));
        }
      }

      start_matrix_rows += num_measure_z - 3;
      
      // estimatation error for normal part
      double  estimation_err2 = 0;
      
      // now computing extended A and b for normal constraints  
      Vec vb2(vb[1]), va2(va[1]), vb1(vb[0]), va1(va[0]), vb3(vb[2]), va3(va[2]);
      Eigen::Matrix3d vb2head, va2head, vb1head, va1head, vb3head, va3head;
      vb2.ToHat(&vb2head);
      va2.ToHat(&va2head);
      va1.ToHat(&va1head);
      vb1.ToHat(&vb1head);
      va3.ToHat(&va3head);
      vb3.ToHat(&vb3head);
      Eigen::VectorXd vj1 = -dJy[0].transpose() * vb2head.transpose() * nn[0];
      Eigen::VectorXd vj2 = dJy[1].transpose() * va2head.transpose() * nn[0];
      Eigen::VectorXd vj3 = -dJx[0].transpose() * vb1head.transpose() * nn[1];
      Eigen::VectorXd vj4 = dJx[1].transpose() * va1head.transpose() * nn[1];

      b2(0) = -nn[0].dot(nn[1]) / (norm_nn[0] * norm_nn[1]);
      estimation_err2 += fabs(b2(0));
      A2.block(0, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
      A2.block(0, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[0] * norm_nn[1]);
      A2.block(0, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);

      vj1 = -dJy[0].transpose() * vb2head.transpose() * nn[2];
      vj2 = dJy[1].transpose() * va2head.transpose() * nn[2];
      vj3 = -dJz[0].transpose() * vb3head.transpose() * nn[1];
      vj4 = dJz[1].transpose() * va3head.transpose() * nn[1];
      b2(1) = -nn[1].dot(nn[2]) / (norm_nn[1] * norm_nn[2]);
      estimation_err2 += fabs(b2(1));
      A2.block(1, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
      A2.block(1, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[1] * norm_nn[2]);
      A2.block(1, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
       
      vj1 = -dJx[0].transpose() * vb1head.transpose() * nn[2];
      vj2 = dJx[1].transpose() * va1head.transpose() * nn[2];
      vj3 = -dJz[0].transpose() * vb3head.transpose() * nn[0];
      vj4 = dJz[1].transpose() * va3head.transpose() * nn[0];
      b2(2) = -nn[0].dot(nn[2]) / (norm_nn[2] * norm_nn[0]);
      estimation_err2 += fabs(b2(2));

      strs.str("");
      strs << "Plane angles: XY=" << acos(fabs(b2(0))) * 180 / M_PI
       << ", YZ=" << acos(fabs(b2(1))) * 180 / M_PI
       << ", ZX="  << acos(fabs(b2(2))) * 180 / M_PI << std::endl;
      LOG_INFO(strs); 
      if (cur_iter <= 1) {
        init_angle_xy = acos(fabs(b2(0)));
        init_angle_yz = acos(fabs(b2(1)));
        init_angle_zx = acos(fabs(b2(2)));
        if (nn[0].dot(pnx) < 0) {
          pnx = -pnx;
        }
        if (nn[1].dot(pny) < 0) {
          pny = -pny;
        }
        if (nn[2].dot(pnz) < 0) {
          pnz=-pnz;
        }
        init_angle_xy1 = acos(fabs(pnx.dot(pny)));
        init_angle_yz1 = acos(fabs(pny.dot(pnz)));
        init_angle_zx1 = acos(fabs(pnz.dot(pnx)));
      }
      A2.block(2, 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
      A2.block(2, 6, 1, 5 * DoF_) = (vj1 + vj2 + vj3 + vj4).transpose() / (norm_nn[2] * norm_nn[0]);
      A2.block(2, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);

      // compute A, b, and average estimation error for a single measurement
      
      if (coplanar_normal_option_ == 0) {  // using coplanar only
          A = A1;
          b = b1;
          estimation_err3 = estimation_err1 / start_matrix_rows; 
      } else if (coplanar_normal_option_ == 1) {
         A.resize(total_rows + 3, 6 + 5 * DoF_ + 12);
         b.resize(total_rows + 3);
         A.block(0, 0, total_rows,  6 + 5 * DoF_ + 12) = A1;
         A.block(total_rows, 0, 3,  6 + 5 * DoF_ + 12) = A2;
         b.block(0, 0, total_rows, 1) = b1;
         b.block(total_rows, 0, 3, 1) = b2;
         estimation_err3 = (estimation_err1 + estimation_err2) / (start_matrix_rows + 3);
      } else if (coplanar_normal_option_ >= 2) {   // using normal only
         A = A2;
         b = b2;
         estimation_err3 = estimation_err2 / 3;
      }
      strs.str("");
      strs << GetName() << ":" << "estimation_err z "<< estimation_err3 <<  std::endl;
      LOG_INFO(strs);
      if (cur_iter <= 1) {
        init_estimation_err  = estimation_err3;
      }
      current_diff = std::min(previous_err - estimation_err,
              std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through laser sensor, iteration no. = " << cur_iter << std::endl;
      LOG_INFO(strs);
      Eigen::VectorXd delta_p_old;
      Eigen::VectorXd base_tmp(6);
      base_tmp.setZero();
      Eigen::Vector3d tool_tmp(3);
      tool_tmp.setZero();

      /*
      alg_.RNNOptimize(A, b, &delta_p_old);
      UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                        &beta_tmp, &tool_tmp, NULL);
      */
      if (resetCache_) {
        size_t numParam = A.cols();
        delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
      }
      if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
        if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
        }
        delta_p_old_cache_ = delta_p_old;
        UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                        &beta_tmp, &tool_tmp, NULL);
        resetCache_ = false;
        estimation_err = estimation_err3;
      } else {
        UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                        &beta_tmp, &tool_tmp, NULL);
        resetCache_ = true;
        estimation_err = previous_err - 1.5 * MAX_ORIENT_CALIB_MATCHING_ERR;
      }                   

    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                << "With estimation error " << estimation_err
                << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches estimation_err " << estimation_err
                << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches err diff " << previous_err - estimation_err
                << ",prev_err=" << previous_err << ", cur_err=" << estimation_err  <<", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                << std::endl;
        LOG_ERROR(strs);
    }
    if (init_estimation_err > 0  && estimation_err < init_estimation_err) {
      alpha_c_ = alpha_old;
      a_c_ = a_old;
      theta_c_ = theta_old;
      d_c_ = d_old;
      beta_c_ = beta_old;
    }
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "init plane angles:XY=" << init_angle_xy * 180 / M_PI << ", from svd=" << init_angle_xy1 * 180 / M_PI 
       << ", YZ=" << init_angle_yz * 180 / M_PI << ", from svd=" << init_angle_yz1 * 180 / M_PI <<
        ", ZX=" << init_angle_zx *180 / M_PI << ", from svd=" << init_angle_zx1 *180 / M_PI << std::endl;
    strs << " final plane angles: XY=" << acos(fabs(b2(0))) * 180 / M_PI
       << ", YZ=" << acos(fabs(b2(1))) * 180 / M_PI
       << ", ZX="  << acos(fabs(b2(2))) * 180 / M_PI <<
     ",final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
  
    isDHCalibrated_ = true;


    Eigen::MatrixXd cartx = cart_off_measure_x.block(0, 0, 3, num_cx);
    Eigen::MatrixXd carty = cart_off_measure_y.block(0, 0, 3, num_cy);
    Eigen::MatrixXd cartz = cart_off_measure_z.block(0, 0, 3, num_cz);
    // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    std::vector<Eigen::Vector3d> nn_orig(3), nn(3);
    std::vector<double> norm_nn_orig(3), norm_nn(3);
    double orig_err1 = 0, comp_err1=0, orig_err2=0, comp_err2=0;
    Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
    double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;

    // compare  the planar error before calib and after comp.
    // first plane constraint equation
    size_t numValid = 0;
    size_t numX = 0; // num_measure_x;
    for (size_t i=numX; i < num_cx; i++) {
      UpdateDH(kine_para, jnt_x.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        return ret;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numX;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[0] = dv1.cross(dv2);
            norm_nn[0] = nn[0].norm();

            dv1_orig = cartx.col(numX + 1) - cartx.col(numX);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = cartx.col(numX + 2) - cartx.col(numX + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[0] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[0] = nn_orig[0].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cartx.col(i) - cartx.col(numX + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cx - numX - 3;
    size_t numY = 0;  //num_measure_y;
    // second plane (y plane) constraint equation
    for (size_t i=numY; i < num_cy; i++) {
      UpdateDH(kine_para, jnt_y.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        return ret;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numY;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[1] = dv1.cross(dv2);
            norm_nn[1] = nn[1].norm();

            dv1_orig = carty.col(numY + 1) - carty.col(numY);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = carty.col(numY + 2) - carty.col(numY + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[1] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[1] = nn_orig[1].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = carty.col(i) - carty.col(numY + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cy - numY - 3;
    size_t numZ = 0; // num_measure_z;
    // third plane (z plane) constraint equation
    for (size_t i=numZ; i < num_cz; i++) {
      UpdateDH(kine_para, jnt_z.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        return ret;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numZ;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[2] = dv1.cross(dv2);
            norm_nn[2] = nn[2].norm();

            dv1_orig = cartz.col(numZ + 1) - cartz.col(numZ);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = cartz.col(numZ + 2) - cartz.col(numZ + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[2] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[2] = nn_orig[2].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cartz.col(i) - cartz.col(numZ + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cz - numZ - 3;
    
    // error comparison for normal methods
    orig_err2 += fabs(nn_orig[0].dot(nn_orig[1]) / (norm_nn_orig[0] * norm_nn_orig[1]));
    orig_err2 += fabs(nn_orig[1].dot(nn_orig[2]) / (norm_nn_orig[1] * norm_nn_orig[2]));
    orig_err2 += fabs(nn_orig[0].dot(nn_orig[2]) / (norm_nn_orig[0] * norm_nn_orig[2]));

      
    comp_err2 += fabs(nn[0].dot(nn[1]) / (norm_nn[0] * norm_nn[1]));
    comp_err2 += fabs(nn[1].dot(nn[2]) / (norm_nn[1] * norm_nn[2]));
    comp_err2 += fabs(nn[0].dot(nn[2]) / (norm_nn[0] * norm_nn[2]));

    strs.str("");
    strs << "orig_err1=" << orig_err1 /  numValid << "comp_err1=" << comp_err1 /  numValid << std::endl;
    strs << "orig_err2=" << orig_err2 / 3 << "comp_err2=" << comp_err2 / 3 << std::endl;
    LOG_INFO(strs);
   
    double avg_orig_err =0, avg_comp_err = 0;
    if (coplanar_normal_option_ == 0) {  // using coplanar only
       avg_orig_err = orig_err1 / numValid;
       avg_comp_err = comp_err1 / numValid;    
    } else if (coplanar_normal_option_ == 1) {
       avg_orig_err = (orig_err1 + orig_err2) / (numValid +3);
       avg_comp_err = (comp_err1 + comp_err2) / (numValid +3);
    } else if (coplanar_normal_option_ >= 2) {   // using normal only
      avg_orig_err = orig_err2 / 3;
      avg_comp_err = comp_err2 / 3;
    }
  
    //double overall_comp_err = (previous_err * total_rows + orig_err) / (total_rows + numValid);  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
        << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
      strs.str("");
      strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
      LOG_ALARM(strs);
      // return -agv_comp_err;
    }
    return avg_comp_err;
}

Eigen::VectorXd serialArm::VerifyCoplanarSing(
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::VectorXd> &laser_measure,
       const double laser_scale,
       const int axisChannel) {
  // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    Eigen::VectorXd outData(2);
    // initialize the outData
    outData(0) = 0;  // no error
    outData(1) = 0;  // no varify calibration data
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":"  << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return outData;
    }

    size_t num_l = laser_measure.size();
    size_t num_c = cart_measure.cols();
    
    // either laser measures == jnt measures, or jnt measures = laser measures + 8
    bool sizeOK = (num_l == num_c);
    if (cart_measure.rows() < 3 || !sizeOK || axisChannel < 0 || axisChannel >= 3) {
      strs.str("");
      strs << GetName() << ":" << "VerifyCoplanar: input data dimension is not matching"
                << " so can not do calibration verification, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
      return outData;
    }

    //! before we start iteration, we compute the "supposed" joint arrays
    Eigen::MatrixXd  jnt_shift(DoF_, num_c);
    //! offseted cartesian array (i.e. offset based upon laser measure data)
    Eigen::MatrixXd cart_off_measure = cart_measure;
    Eigen::VectorXd jnt;
    double first_laser;

    //before doing anything, set using uncalibrated model
    SetUsingCalibratedModel(false);
    // plane parallel to x block surface
    for (size_t i=0; i < num_c; i++) {
        Eigen::VectorXd cart = cart_measure.col(i);
        double laser = laser_measure(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(axisChannel) = cart(axisChannel) - laser_scale * (laser - first_laser);
        }
        if (!GetJntFromPose(cart, &jnt)) {
          outData(0) = -1101;  // IK error
          return outData;
        }
        jnt_shift.col(i) = jnt;
        cart_off_measure.col(i) = cart;
    }
    
     // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    // overall error before and after compensation
    double orig_err = 0, comp_err=0;
    Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
    Eigen::Vector3d dv1, dv2, dv3;
    double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;
    double norm_dv1, norm_dv2, norm_dv3;
    size_t numValid = 0;
    for (size_t i=0; i < num_c; i++) {
      UpdateDH(kine_para, jnt_shift.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        outData(0) = ret;
        return outData;
      }
      Vec  tf = p.getTranslation();
      if (i < 3) {
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =dv2.norm();
             dv1_orig = cart_off_measure.col(1) - cart_off_measure.col(0);
             norm_dv1_orig = dv1_orig.norm();
             dv2_orig = cart_off_measure.col(2) - cart_off_measure.col(1);
             norm_dv2_orig = dv2_orig.norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_off_measure.col(i) - cart_off_measure.col(2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err += fabs(AA.determinant());
      }
    }
    numValid += num_c - 3;
    double avg_orig_err= orig_err /  numValid;
    double avg_comp_err = comp_err / numValid;
     // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);

    outData(0) =avg_orig_err;
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        outData(1) = -avg_comp_err;
    } else {
      outData(1) = avg_comp_err;
    }
    return outData;
}

Eigen::VectorXd serialArm::VerifyCoplanar(
               const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
               const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
               const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
               const EigenDRef<Eigen::MatrixXd> &laser_measure_x,
               const EigenDRef<Eigen::MatrixXd> &laser_measure_y,
               const EigenDRef<Eigen::MatrixXd> &laser_measure_z,
               const EigenDRef<Eigen::Vector3d> &laser_scale) {
  // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    Eigen::VectorXd outData(2);
    // initialize the outData
    outData(0) = 0;  // no error
    outData(1) = 0;  // no varify calibration data
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return outData;
    }
   
    size_t num_lx = laser_measure_x.cols();
    size_t num_cx = cart_measure_x.cols();

    size_t num_ly = laser_measure_y.cols();
    size_t num_cy = cart_measure_y.cols();

    size_t num_lz = laser_measure_z.cols();
    size_t num_cz = cart_measure_z.cols();

    
    // either laser measures == jnt measures, or jnt measures = laser measures + 8
    bool sizeOK = ((num_lx == num_cx) && (num_ly == num_cy) && (num_lz == num_cz));
    if (cart_measure_x.rows() < 3 || !sizeOK || laser_measure_x.rows() <3 ||
        cart_measure_y.rows() < 3 || laser_measure_y.rows() < 3 ||
        cart_measure_z.rows() < 3 || laser_measure_z.rows() < 3) {
      strs.str("");
      strs << GetName() << ":" << "VerifyCoplanar: input data dimension is not matching"
                << " so can not do calibration verification, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM;
      return outData;
    }

    //! before we start iteration, we compute the "supposed" joint arrays
    Eigen::MatrixXd  jnt_x(DoF_, num_cx), jnt_y(DoF_, num_cy), jnt_z(DoF_, num_cz);
    //! offseted cartesian array (i.e. offset based upon laser measure data)
    Eigen::MatrixXd cart_off_measure_x, cart_off_measure_y, cart_off_measure_z;
    cart_off_measure_x = cart_measure_x;
    cart_off_measure_y = cart_measure_y;
    cart_off_measure_z = cart_measure_z;
    Eigen::VectorXd first_laser, jnt;

    //before doing anything, set using uncalibrated model
    SetUsingCalibratedModel(false);
    // plane parallel to x block surface
    for (size_t i=0; i < num_cx; i++) {
        Eigen::VectorXd cart = cart_measure_x.col(i);
        Eigen::VectorXd laser = laser_measure_x.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(0) = cart(0) - laser_scale(0) * (laser(0) - first_laser(0));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          outData(0) = -1101;  // IK error
          return outData;
        }
        jnt_x.col(i) = jnt;
        cart_off_measure_x.col(i) = cart;
    }
    
    // plane parallel to y block surface
    for (size_t i=0; i < num_cy; i++) {
        Eigen::VectorXd cart = cart_measure_y.col(i);
        Eigen::VectorXd laser = laser_measure_y.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(1) = cart(1) - laser_scale(1) * (laser(1) - first_laser(1));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          outData(0) = -1101;  // IK error
          return outData;
        }
        jnt_y.col(i) = jnt;
        cart_off_measure_y.col(i) = cart;
    }
    //strs.str("");
    //strs << "debug1 "<< std::endl;
    //LOG_INFO(strs);
    // plane parallel to z block surface
    for (size_t i=0; i < num_cz; i++) {
        Eigen::VectorXd cart = cart_measure_z.col(i);
        Eigen::VectorXd laser = laser_measure_z.col(i);
        if (i==0) {
          first_laser = laser;
        } else {
          cart(2) = cart(2) - laser_scale(2) * (laser(2) - first_laser(2));
        }
        if (!GetJntFromPose(cart, &jnt)) {
          outData(0) = -1101;  // IK error
          return outData;
        }
        jnt_z.col(i) = jnt;
        cart_off_measure_z.col(i) = cart;
    }

    strs.str("");
    strs << GetName() << ":" << "In verifyCoplanar: alpha: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_[k] << " ";
    }
    strs << ", a: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_[k] << " ";
    }
    strs << ", theta: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_[k] << " ";
    }
    strs << ", d: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_[k] << " ";
    }
    strs << ", beta: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_[k] << " ";
    }
    LOG_INFO(strs);

    strs.str("");
    strs << GetName() << ":" << "calibrated set: alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    LOG_INFO(strs);


     // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    std::vector<Eigen::Vector3d> nn_orig(3), nn(3);
    std::vector<double> norm_nn_orig(3), norm_nn(3);
    double orig_err1 = 0, comp_err1=0, orig_err2=0, comp_err2=0;
    Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
    double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;
    Eigen::Vector3d dv1, dv2, dv3;
    double norm_dv1, norm_dv2, norm_dv3;

    // compare  the planar error before calib and after comp.
    // first plane constraint equation
    size_t numValid = 0;
    size_t numX = 0; // num_measure_x;
    for (size_t i=numX; i < num_cx; i++) {
      UpdateDH(kine_para, jnt_x.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        outData(0) = ret;  // IK error
        return outData;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numX;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[0] = dv1.cross(dv2);
            norm_nn[0] = nn[0].norm();

            dv1_orig = cart_off_measure_x.col(numX + 1) - cart_off_measure_x.col(numX);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = cart_off_measure_x.col(numX + 2) - cart_off_measure_x.col(numX + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[0] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[0] = nn_orig[0].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_off_measure_x.col(i) - cart_off_measure_x.col(numX + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cx - numX - 3;
    size_t numY = 0;  //num_measure_y;
    // second plane (y plane) constraint equation
    for (size_t i=numY; i < num_cy; i++) {
      UpdateDH(kine_para, jnt_y.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        outData(0) = ret;  // IK error
        return outData;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numY;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[1] = dv1.cross(dv2);
            norm_nn[1] = nn[1].norm();

            dv1_orig = cart_off_measure_y.col(numY + 1) - cart_off_measure_y.col(numY);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = cart_off_measure_y.col(numY + 2) - cart_off_measure_y.col(numY + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[1] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[1] = nn_orig[1].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_off_measure_y.col(i) - cart_off_measure_y.col(numY + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cy - numY - 3;
    size_t numZ = 0; // num_measure_z;
    // third plane (z plane) constraint equation
    for (size_t i=numZ; i < num_cz; i++) {
      UpdateDH(kine_para, jnt_z.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        outData(0) = ret;  // IK error
        return outData;
      }
      Vec  tf = p.getTranslation();
      size_t j = i - numZ;
      if (j < 3) {
          pp[j] = tf.ToEigenVec();
          if (j == 2) {
            dv1= pp[1] - pp[0];
            norm_dv1 =dv1.norm();
            dv2= pp[2] - pp[1];
            norm_dv2 =dv2.norm();
            nn[2] = dv1.cross(dv2);
            norm_nn[2] = nn[2].norm();

            dv1_orig = cart_off_measure_z.col(numZ + 1) - cart_off_measure_z.col(numZ);
            norm_dv1_orig = dv1_orig.norm();
            dv2_orig = cart_off_measure_z.col(numZ + 2) - cart_off_measure_z.col(numZ + 1);
            norm_dv2_orig = dv2_orig.norm();
            nn_orig[2] = dv1_orig.cross(dv2_orig);
            norm_nn_orig[2] = nn_orig[2].norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_off_measure_z.col(i) - cart_off_measure_z.col(numZ + 2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err1 += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err1 += fabs(AA.determinant());
      }
    }
    numValid += num_cz - numZ - 3;
    
    // error comparison for normal methods
    orig_err2 += fabs(nn_orig[0].dot(nn_orig[1]) / (norm_nn_orig[0] * norm_nn_orig[1]));
    orig_err2 += fabs(nn_orig[1].dot(nn_orig[2]) / (norm_nn_orig[1] * norm_nn_orig[2]));
    orig_err2 += fabs(nn_orig[0].dot(nn_orig[2]) / (norm_nn_orig[0] * norm_nn_orig[2]));

      
    comp_err2 += fabs(nn[0].dot(nn[1]) / (norm_nn[0] * norm_nn[1]));
    comp_err2 += fabs(nn[1].dot(nn[2]) / (norm_nn[1] * norm_nn[2]));
    comp_err2 += fabs(nn[0].dot(nn[2]) / (norm_nn[0] * norm_nn[2]));
   
    double avg_orig_err =0, avg_comp_err = 0;
    if (coplanar_normal_option_ == 0) {  // using coplanar only
       avg_orig_err = orig_err1 / numValid;
       avg_comp_err = comp_err1 / numValid;    
    } else if (coplanar_normal_option_ == 1) {
       avg_orig_err = (orig_err1 + orig_err2) / (numValid +3);
       avg_comp_err = (comp_err1 + comp_err2) / (numValid +3);
    } else if (coplanar_normal_option_ >= 2) {   // using normal only
      avg_orig_err = orig_err2 / 3;
      avg_comp_err = comp_err2 / 3;
    }

     // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);

    outData(0) =avg_orig_err;
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        outData(1) = -avg_comp_err;
    } else {
      outData(1) = avg_comp_err;
    }
    return outData;
}

//! assume we know the rough equation of the probing plane,
// rough initial tool offset, and a list of recorded joint vectors
// for which the tool tip touch the plane, and we want to find the final plane
// and final tool offset
double serialArm::CalibratePlaneMethod(
       //const EigenDRef<Eigen::Vector3d> &init_plane,
       const Eigen::VectorXd &init_base_offset,
       const Eigen::VectorXd &init_tool_offset,
       const EigenDRef<Eigen::MatrixXd> &qa_measure,  // joint coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cart coordinates reported from robot, which shall already include the init_tool_offset
       //EigenDRef<Eigen::Vector3d> *final_plane,
       Eigen::VectorXd *final_base_offset,
       Eigen::VectorXd *final_tool_offset) {
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
    }
    if (!final_base_offset || !final_tool_offset) {
      strs.str("");
      strs << GetName() << ":" << "Input final_plane or final_tool_offset pointer is null"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;  
    }
    // currently ignores base regression, use default base
    *final_base_offset = init_base_offset;   // we don't change the base in probe-based DH calibration (because base doesn't change the plane)
    *final_tool_offset = init_tool_offset;  // we don't change the tool
    strs.str("");
    strs << GetName() << ":" << "before calib, init base = " << init_base_offset 
         << ", init tool =" << init_tool_offset << std::endl;
    LOG_INFO(strs);
    size_t total_measures = qa_measure.cols();
    size_t num_measures = 2 * total_measures / 3;  // half for calib, and the other half for verification
    size_t total_rows = num_measures - 3;  // use the determinant, the first 3 points are forming a plance
    if (total_rows < 2) {
      strs.str(""); 
      strs << GetName() << ":" << "Scara plane calibration: need at least four samples,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_CALIB_LASER_LESS_SAMPLES;
    }
    if (init_base_offset.rows() !=7) {
      strs.str("");
      strs << GetName() << ":" << "Input init_base_offset has wrong dimension,"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
    }
    // initialize iter alg parameters
    alg_.setParam(decay_coef_ *  CALIB_RNN_STEPSIZE,
                   ridgeScale_, cyc_mod_, sam_region_scale_);
    resetCache_ = true;
    // get tool_offset
    Eigen::Vector3d tool_tmp = init_tool_offset.segment(0, 3);
    Eigen::Vector3d tool_old = tool_tmp;
 
    // in any case, we start with uncaliberated model
    std::vector<double> a_tmp = a_c_, a_old = a_c_,
                        alpha_tmp = alpha_c_, alpha_old = alpha_c_,
                        beta_tmp = beta_c_, beta_old = beta_c_,
                        d_tmp = d_c_, d_old = d_c_,
                        theta_tmp = theta_c_, theta_old = theta_c_;
    // step 1, using canonical FK to compute the corresponding
    // we need to define one matrix A and one vector b for regression
    // number of columns 6 + 5 * DoF_ + 12, base_x, base_y, base_z, base_yaw, base_pitch, base_roll, alpha_ [DoF_], a_ [DoF_],
    // theta_ [DoF_], d_ [DoF_],  beta_[DoF_], tool_offset [3],  9 for affine matrix  transform * dlaser -> dcart
    Eigen::MatrixXd A(total_rows, 6 + 5 * DoF_ + 12);
    Eigen::VectorXd b(total_rows);
    Eigen::Vector3d dv1, dv2, dv3;
    double norm_dv1, norm_dv2, norm_dv3;
    double previous_err = std::numeric_limits<double>::max();
    double estimation_err = 0.5 * previous_err;
    double previous_diff = previous_err - estimation_err;
    double current_diff = previous_diff / 2.0;
    int cur_iter = 0;
    while ((estimation_err >  MAX_CALIB_STOP_ERR_COPLANAR &&
           previous_err - estimation_err > MAX_CALIB_MATCHING_ERR_COPLANAR || !resetCache_)
           && cur_iter < MAX_CALIB_OUTER_ITER
           // && previous_diff - current_diff > MAX_CALIB_MATCHING_ERR
           ) {
      // assign previous value
      if (resetCache_) {
        a_old = a_tmp; alpha_old = alpha_tmp;
        d_old = d_tmp; theta_old = theta_tmp;
        beta_old  = beta_tmp;
        tool_old = tool_tmp;
        previous_err = estimation_err;
      }
      previous_diff = current_diff;
      cur_iter++;
      
      std::vector<double> kine_para, tmp_para; // clearing kine_para
      // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
      kine_para.insert(kine_para.end(), alpha_tmp.begin(), alpha_tmp.end());
      kine_para.insert(kine_para.end(), a_tmp.begin(), a_tmp.end());
      kine_para.insert(kine_para.end(), theta_tmp.begin(), theta_tmp.end());
      kine_para.insert(kine_para.end(), d_tmp.begin(), d_tmp.end());
      kine_para.insert(kine_para.end(), beta_tmp.begin(), beta_tmp.end());
      estimation_err = 0;
     
      // before optimization, A and b need to resize, because its dep. columns is removed during opt.
      A.resize(total_rows, 6 + 5 * DoF_ + 12);
      b.resize(total_rows);
      
      // pp and J vector are for computing the items in A matrix, and b vector
      std::vector<Eigen::Vector3d> pp(4);
      std::vector<Eigen::MatrixXd> J(4);
      Eigen::MatrixXd tmpJ(3, 5 * DoF_);
      Eigen::MatrixXd dJ1, dJ2, dJ3;
      size_t start_matrix_rows = 0;
      //strs.str("");
      //strs <<  "start_matrix_rows" << start_matrix_rows << "esitmation_err x "<< estimation_err <<  std::endl;
      //LOG_INFO(strs);
      // first plane constraint equation
      for (size_t i=0; i < num_measures; i++) {
        UpdateDH(kine_para, qa_measure.col(i), &tmp_para);
        Eigen::MatrixXd Jp_t, Jp_r;
        Pose p;
        // compute the expected values from known canonical kinematic parameters
        // i.e., not-calibrated parameter set
        int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
        if (ret < 0) {
          return ret;
        }
        
        // we get R_04, the rotation matrix between 0 and 4
        Rotation r_04 = p.getRotation();
        Vec  t_04 = p.getTranslation();
        Vec tool_offset(tool_tmp);
        Vec tf = r_04 * tool_offset + t_04;
        Eigen::Matrix3d t_e_hat;
        tf.ToHat(&t_e_hat);
        tmpJ = Jp_t - t_e_hat * Jp_r;  //corresponding to delta p
        if (i < 3) {
          J[i] = tmpJ;
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =  dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =  dv2.norm();
             dJ1 = J[1] - J[0];
             dJ2 = J[2] - J[1];
          }
        } else {
          J[3] = tmpJ;
          pp[3] = tf.ToEigenVec();
          dJ3 = J[3] - J[2];
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();

          // computing the Jacobian
          A.block((i-3), 0, 1, 6) = Eigen::MatrixXd::Zero(1, 6); // dbase
          Eigen::Matrix3d AA;
          for (size_t j=0; j< 5 * DoF_; j++) {
                 double sum = 0;
                 AA.col(0) = dJ1.col(j) / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dv3 / norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;;
                 AA.col(1) = dJ2.col(j) / norm_dv2;
                 AA.col(2) = dv3 /  norm_dv3;
                 sum += AA.determinant();
                 AA.col(0) = dv1 / norm_dv1;
                 AA.col(1) = dv2 / norm_dv2;
                 AA.col(2) = dJ3.col(j) / norm_dv3;
                 sum += AA.determinant();
                 A(i -3, j + 6) = sum;
          }
          A.block(i -3, 6 + 5 * DoF_, 1, 12) = Eigen::MatrixXd::Zero(1, 12);
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 /  norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          b(i - 3) = - AA.determinant();
          estimation_err += fabs(b(i - 3));
        }
      }

      strs.str("");
      // average estimation error for a single measurement
      estimation_err  /= double(total_rows) ;
      strs << GetName() << ":" << "esitmation_err z "<< estimation_err <<  std::endl;
      LOG_INFO(strs);
      current_diff = std::min(previous_err - estimation_err,
               std::numeric_limits<double>::max() / 8.0);
      // this is to remove the impacts from some redundant parameters
      for (size_t i=0; i < d_jacobian_cols_.size(); i++) {
        // luckily, the vector of depend column indices in d_jacobian_cols_
        // are arranged from largest toward smallest, so we can continuously
        // call removeColumn
        //strs.str("");
        //strs << "removing cols from A: " << d_jacobian_cols_[i] << std::endl;
        //LOG_INFO(strs);
        removeColumn(&A, d_jacobian_cols_[i]);
      }
      // using RNN to compute the best delta_para given the current para
      // RNN used as an internal loop
      strs.str("");
      strs << GetName() << ":" << "Calib. through probe plane method, iteration no. = " << cur_iter << std::endl;
      LOG_INFO(strs);
      try {
        Eigen::VectorXd delta_p_old;
        Eigen::VectorXd base_tmp(6);
        base_tmp.setZero();
        Eigen::Vector3d tool_tmp(3);
        tool_tmp.setZero();
        /*
         alg_.RNNOptimize(A, b, &delta_p_old);
         UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, NULL);
        */
        if (resetCache_) {
          size_t numParam = A.cols();
          delta_p_old_cache_ = Eigen::VectorXd::Zero(numParam);
        }
        if (!alg_.RNNOptimize(A, b, &delta_p_old)) {
          if (delta_p_old.size()==0) {
            strs.str("");
            strs << "RNNOPtimize fails" << std::endl;
            LOG_ERROR(strs);
            return -ERR_CALIB_REG_WRONG_DIM;
          }
          delta_p_old_cache_ = delta_p_old;
          UpdateDH(delta_p_old, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, NULL);
          // reset estimation_err. to enforce go to next iteration
          //estimation_err = previous_err - 1.02 * MAX_CALIB_MATCHING_ERR;
          resetCache_ = false;
        } else {
           UpdateDH(delta_p_old - delta_p_old_cache_, &base_tmp, &alpha_tmp,
                        &a_tmp, &theta_tmp, &d_tmp,
                         &beta_tmp, &tool_tmp, NULL);
           resetCache_ = true;
        }                   
      } catch (...) {
         strs.str("");
         strs << GetName() << ":" << "Call RNNOptimize and UpdateDH cause exception  in " << __FUNCTION__ << ", line " << __LINE__ << std::endl;
         LOG_ERROR(strs);
         return -10001;
      }
    }
    if (cur_iter >= MAX_CALIB_OUTER_ITER) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches maximal " << MAX_CALIB_OUTER_ITER
                 << "With estimation error " << estimation_err
                 << std::endl;
        LOG_ERROR(strs);
        //return -ERR_CALIB_REG_MAX_ITER;
    }
    if (estimation_err <=  MAX_CALIB_STOP_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches estimation_err " << estimation_err
                 << ", while the set limit is  " << MAX_CALIB_STOP_ERR
                 << std::endl;
        LOG_ERROR(strs);
    }
    if (previous_err - estimation_err <= MAX_CALIB_MATCHING_ERR_COPLANAR) {
        strs.str("");
        strs << GetName() << ":" << "In laser coplanar calib: Iteration reaches err diff " << previous_err - estimation_err
                 << ", while the set limit is  " << MAX_CALIB_MATCHING_ERR_COPLANAR
                 << std::endl;
        LOG_ERROR(strs);
    }
    alpha_c_ = alpha_old;
    a_c_ = a_old;
    theta_c_ = theta_old;
    d_c_ = d_old;
    beta_c_ = beta_old;
    strs.str("");
    strs << GetName() << ":" << "alpha_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << alpha_c_[k] << " ";
    }
    strs << ", a_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << a_c_[k] << " ";
    }
    strs << ", theta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << theta_c_[k] << " ";
    }
    strs << ", d_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << d_c_[k] << " ";
    }
    strs << ", beta_c: ";
    for (size_t k=0; k < DoF_; k++) {
      strs << beta_c_[k] << " ";
    }
    strs << std::endl;
    strs << "final matching error=" << previous_err << std::endl;
    LOG_INFO(strs);
    (*final_tool_offset)(0) = tool_old(0);
    (*final_tool_offset)(1) = tool_old(1);
    (*final_tool_offset)(2) = tool_old(2); 
    strs.str("");
    strs << "final tool offset=" << (*final_tool_offset) << std::endl;
    LOG_INFO(strs);
    isDHCalibrated_ = true;


    // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    double orig_err = 0, comp_err=0;
    Eigen::Vector3d dv1_orig, dv2_orig, dv3_orig;
    double norm_dv1_orig, norm_dv2_orig, norm_dv3_orig;

    // compare  the planar error before calib and after comp.
    // first plane constraint equation
    size_t numValid = 0;
    for (size_t i=0; i < total_measures; i++) {
      UpdateDH(kine_para, qa_measure.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        return ret;
      }
      // we get R_04, the rotation matrix between 0 and 4
      Rotation r_04 = p.getRotation();
      Vec  t_04 = p.getTranslation();
      Vec tool_offset(tool_old);
      Vec tf = r_04 * tool_offset + t_04;
      if (i < 3) {
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =dv2.norm();
             dv1_orig = cart_measure.col(1) - cart_measure.col(0);
             norm_dv1_orig = dv1_orig.norm();
             dv2_orig = cart_measure.col(2) - cart_measure.col(1);
             norm_dv2_orig = dv2_orig.norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_measure.col(i) - cart_measure.col(2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err += fabs(AA.determinant());
      }
    }
    numValid +=  total_measures - 3;
    double avg_orig_err= orig_err /  numValid;
    double avg_comp_err = comp_err / numValid;
    //double overall_comp_err = (previous_err * total_rows + orig_err) / (total_rows + numValid);  // balanced overall comp err
    // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        // return -agv_comp_err;
    }
    return avg_comp_err;
}

 Eigen::VectorXd serialArm::VerifyProbing(
       const EigenDRef<Eigen::VectorXd> & tool_offset,
       const EigenDRef<Eigen::MatrixXd> &qa_measure, 
       const EigenDRef<Eigen::MatrixXd> &cart_measure) {
    // check if robot has been initialized, i.e. we need to know
    // the rough kinematic model
    std::ostringstream strs;
    Eigen::VectorXd outData(2);
    // initialize the outData
    outData(0) = 0;  // no error
    outData(1) = 0;  // no varify calibration data
    if (!initialized_) {
      strs.str("");
      strs << GetName() << ":" << "Scara geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      outData(0) = -ERR_ROB_PARAM_NOT_INITIALIZED;
      return outData;
    }

    strs.str("");
    strs << GetName() << ":" << "VerifyProbing(), tool =" << tool_offset << std::endl;
    LOG_INFO(strs);
    size_t total_measures = qa_measure.cols();
    // not goes to testing phase: check if the regressed model also fit to other poits
    std::vector<double> kine_para, tmp_para; // clearing kine_para
    // fill in the value of alpha_tmp, a_tmp, theta_tmp, d_tmp
    kine_para.insert(kine_para.end(), alpha_c_.begin(), alpha_c_.end());
    kine_para.insert(kine_para.end(), a_c_.begin(), a_c_.end());
    kine_para.insert(kine_para.end(), theta_c_.begin(), theta_c_.end());
    kine_para.insert(kine_para.end(), d_c_.begin(), d_c_.end());
    kine_para.insert(kine_para.end(), beta_c_.begin(), beta_c_.end());

     // get tool_offset
    Vec tool(tool_offset.segment(0, 3));
    // end-effector cartesian position from calibrated model, and original models
    std::vector<Eigen::Vector3d> pp(4), pp_orig(4);
    double orig_err = 0, comp_err=0;
    Eigen::Vector3d dv1, dv1_orig, dv2, dv2_orig, dv3, dv3_orig;
    double norm_dv1, norm_dv1_orig, norm_dv2, norm_dv2_orig, norm_dv3, norm_dv3_orig;

    // compare  the planar error before calib and after comp.
    // first plane constraint equation
    size_t numValid = 0;
    for (size_t i=0; i < total_measures; i++) {
      UpdateDH(kine_para, qa_measure.col(i), &tmp_para);
      Eigen::MatrixXd Jp_t, Jp_r;
      Pose p;
      // compute the expected values from known canonical kinematic parameters
      // i.e., not-calibrated parameter set
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r);
      if (ret < 0) {
        outData(0) = ret;
        return outData;
      }
      // we get R_04, the rotation matrix between 0 and 4
      Rotation r_04 = p.getRotation();
      Vec  t_04 = p.getTranslation();
      Vec tf = r_04 * tool + t_04;
      if (i < 3) {
          pp[i] = tf.ToEigenVec();
          if (i == 2) {
             dv1= pp[1] - pp[0];
             norm_dv1 =dv1.norm();
             dv2= pp[2] - pp[1];
             norm_dv2 =dv2.norm();
             dv1_orig = cart_measure.col(1) - cart_measure.col(0);
             norm_dv1_orig = dv1_orig.norm();
             dv2_orig = cart_measure.col(2) - cart_measure.col(1);
             norm_dv2_orig = dv2_orig.norm();
          }
      } else {
          pp[3] = tf.ToEigenVec();
          dv3 = pp[3] - pp[2];
          norm_dv3 = dv3.norm();
          dv3_orig = cart_measure.col(i) - cart_measure.col(2);
          norm_dv3_orig = dv3_orig.norm();
        
          Eigen::Matrix3d AA;
          AA.col(0) = dv1 / norm_dv1;
          AA.col(1) = dv2 / norm_dv2;
          AA.col(2) = dv3 / norm_dv3;
          comp_err += fabs(AA.determinant());
          AA.col(0) = dv1_orig / norm_dv1_orig;
          AA.col(1) = dv2_orig / norm_dv2_orig;
          AA.col(2) = dv3_orig / norm_dv3_orig;
          orig_err += fabs(AA.determinant());
      }
    }
    numValid +=  total_measures - 3;
    double avg_orig_err= orig_err /  numValid;
    double avg_comp_err = comp_err / numValid;

     // average estimation error for a single measurement
    strs.str("");
    strs << GetName() << ":" << "accumulation error before comp. = " << avg_orig_err
         << ", after comp. the error = " << avg_comp_err << std::endl;
    LOG_INFO(strs);

    outData(0) = avg_orig_err;
    if (avg_comp_err >= avg_orig_err) {
        strs.str("");
        strs << GetName() << ":" << "Calib verification result is not satisfied" << std::endl;
        LOG_ALARM(strs);
        outData(1) = -avg_comp_err;
    } else {
      outData(1) = avg_comp_err;
    }
    return outData;
}

bool serialArm::GetCalibParamSet(EigenDRef<Eigen::VectorXd> *cal_DH) {
  std::ostringstream strs;
  if (!cal_DH) {  // if not calibrated, return false
      strs.str("");
      strs << GetName() << ":" << "DH is not calibrated or the input pointer is null in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
  }
  // additional 7 is for baseOffSet
  if (cal_DH->size() < 5 * DoF_ + 14) {
      strs.str("");
      strs << GetName() << ":" << "The input vector pointer has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
  }
  std::vector<double> tmp;
  tmp.insert(tmp.end(), alpha_c_.begin(), alpha_c_.end());
  tmp.insert(tmp.end(), a_c_.begin(), a_c_.end());
  tmp.insert(tmp.end(), theta_c_.begin(), theta_c_.end());
  tmp.insert(tmp.end(), d_c_.begin(), d_c_.end());
  tmp.insert(tmp.end(), beta_c_.begin(), beta_c_.end());
  for (size_t i=0; i < 5 * DoF_; i++) {
      (*cal_DH)(i) = tmp[i];
  }
  Eigen::VectorXd  eigBaseOff = defaultBaseOff_.ToEigenVecQuat();
  for (size_t i = 0; i < 7; i++ ) {
     (*cal_DH)(5 * DoF_ + i) = eigBaseOff(i);
  }
  eigBaseOff = sub_defaultBaseOff_.ToEigenVecQuat();
  for (size_t i = 0; i < 7; i++ ) {
     (*cal_DH)(5 * DoF_ + 7 + i) = eigBaseOff(i);
  }
  return true;
}

bool serialArm::GetCalibParamSet(Eigen::VectorXd *cal_DH) {
  std::ostringstream strs;
  if (!cal_DH) {  // if not calibrated, return false
      strs.str("");
      strs << GetName() << ":" << "DH is not calibrated or the input pointer is null in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return false;
  }
  if (cal_DH->size() != 5 * DoF_ + 14) {
     cal_DH->resize(5 * DoF_ + 14);
  }
  std::vector<double> tmp;
  tmp.insert(tmp.end(), alpha_c_.begin(), alpha_c_.end());
  tmp.insert(tmp.end(), a_c_.begin(), a_c_.end());
  tmp.insert(tmp.end(), theta_c_.begin(), theta_c_.end());
  tmp.insert(tmp.end(), d_c_.begin(), d_c_.end());
  tmp.insert(tmp.end(), beta_c_.begin(), beta_c_.end());
  for (size_t i=0; i < 5 * DoF_; i++) {
      (*cal_DH)(i) = tmp[i];
  }
  Eigen::VectorXd  eigBaseOff = defaultBaseOff_.ToEigenVecQuat();
  for (size_t i = 0; i < 7; i++ ) {
     (*cal_DH)(5 * DoF_ + i) = eigBaseOff(i);
  }
  eigBaseOff = sub_defaultBaseOff_.ToEigenVecQuat();
  for (size_t i = 0; i < 7; i++ ) {
     (*cal_DH)(5 * DoF_ + 7 + i) = eigBaseOff(i);
  }
  return true;
}

bool serialArm::LoadCalibParamSet(const std::vector<double> &cal_DH) {
  std::ostringstream strs;
  if (cal_DH.size() < 5 *DoF_ + 14) {
      strs.str("");
      strs << GetName() << ":" << "The input vector has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
    return false;
  }
  for (size_t i=0; i < DoF_; i++) {
    alpha_c_[i] = cal_DH[i];
    a_c_[i] = cal_DH[DoF_ + i];
    theta_c_[i] = cal_DH[2 * DoF_ + i];
    d_c_[i] = cal_DH[3 * DoF_ + i];
    beta_c_[i] = cal_DH[4 * DoF_ + i];
  }
  strs.str("");
  strs << GetName() << ":" << "load calib: alpha_c: ";
  for (size_t k=0; k < DoF_; k++) {
    strs << alpha_c_[k] << " ";
  }
  strs << ", a_c: ";
  for (size_t k=0; k < DoF_; k++) {
    strs << a_c_[k] << " ";
  }
  strs << ", theta_c: ";
  for (size_t k=0; k < DoF_; k++) {
    strs << theta_c_[k] << " ";
  }
  strs << ", d_c: ";
  for (size_t k=0; k < DoF_; k++) {
    strs << d_c_[k] << " ";
  }
  strs << ", beta_c: ";
  for (size_t k=0; k < DoF_; k++) {
    strs << beta_c_[k] << " ";
  }
  LOG_INFO(strs);

  std::vector<double> baseoff_vec, subbaseoff_vec;
  baseoff_vec.insert(baseoff_vec.end(), cal_DH.begin() + 5 * DoF_, cal_DH.begin() + 5 * DoF_ + 7);
  subbaseoff_vec.insert(subbaseoff_vec.end(), cal_DH.begin()  + 5 * DoF_ + 7, cal_DH.end());
  Eigen::VectorXd baseoff, subbaseoff;
  StdVec2EigenVec(baseoff_vec, &baseoff);
  StdVec2EigenVec(subbaseoff_vec, &subbaseoff);
  if (baseoff.size() != 7) {
    strs.str("");
    strs << GetName() << ":" << "input of SetDefaultBaseOff = " << baseoff << ", has wrong dimension" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  Vec t(baseoff.segment(0,3));
  Quaternion q(baseoff(3), baseoff(4), baseoff(5), baseoff(6));
  defaultBaseOff_.setTranslation(t);
  defaultBaseOff_.setQuaternion(q);
  strs.str("");
  strs << GetName() << ":" << "SetDefaultBaseOff = " << defaultBaseOff_.ToString(true) << ", quat="  << defaultBaseOff_.ToString(false)
  << ", in edgen=" << baseoff.transpose() << std::endl;
  LOG_INFO(strs);

  // set subBaseoff
  if (subbaseoff.size() != 7) {
    strs.str("");
    strs << GetName() << ":" << "subbaseoff input of SetDefaultBaseOff = " << subbaseoff << ", has wrong dimension" << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  Vec t1(subbaseoff.segment(0,3));
  Quaternion q1(subbaseoff(3), subbaseoff(4), subbaseoff(5), subbaseoff(6));
  sub_defaultBaseOff_.setTranslation(t1);
  sub_defaultBaseOff_.setQuaternion(q1);
  strs.str("");
  strs << GetName() << ":" << "SetSubDefaultBaseOff = " << sub_defaultBaseOff_.ToString(true) << ", quat="  << sub_defaultBaseOff_.ToString(false)
  << ", in edgen=" << subbaseoff.transpose() << std::endl;
  LOG_INFO(strs);
  isDHCalibrated_ = true;
  return true;
}


double serialArm::CalibrateLaserTCP(
      // first 2 sets of 4-pt edge scan to determine the tcp orientation
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_4pt_low,
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_4pt_high,
      // another 3 set of 4-pt edge scan with different orientation, but with same laser readings with jnt_mes_4pt_high
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_12pt,
      EigenDRef<Eigen::VectorXd> *final_tool_offset      // output of the tool offset
      ) {    
    std::ostringstream strs;
    size_t numJnts_1 = jnt_mes_4pt_low.rows();
    size_t numMes_1 = jnt_mes_4pt_low.cols();
    size_t numJnts_2 = jnt_mes_4pt_high.rows();
    size_t numMes_2 = jnt_mes_4pt_high.cols();
    size_t numJnts_3 = jnt_mes_12pt.rows();
    size_t numMes_3 = jnt_mes_12pt.cols();
    
    // here orig_base is the workpiece frame, should be reachable by IK
    if (numJnts_1 < DoF_ || numJnts_2 < DoF_ || numJnts_3 < DoF_ || 
        numMes_1 < 4 || numMes_2 < 4 || numMes_3 < 12) {
      strs.str("");
      strs << GetName() << ":" << "The input vector has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -1;
    }
    if (!final_tool_offset) {
      strs.str("");
      strs << GetName() << ":" << "input final_tool_offset is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
     // set using uncalibrated model
    SetUsingCalibratedModel(false);  
    // declare the tmp variable to recording the pose at each measured cliff point
    std::vector<Pose> ps(numJnts_1); 
    // std::cout << "numJnts_1=" << numJnts_1 <<", numJnts_2=" << numJnts_2 << ", numJnts_3=" << numJnts_3<< std::endl;     
    // compute the first 4pt
    for (size_t i=0; i<numJnts_1; i++) {
      std::vector<double> jnt(DoF_, 0);
      EigenVec2StdVec(jnt_mes_4pt_low.col(i), &jnt);
      int ret = JntToCart(jnt, &ps[i]);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "Scara FK error, code  " << ret
                << "can not do TCP calibration in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
    }

    // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
    Vec p0 = ps[0].getTranslation();
    Vec p1 = ps[1].getTranslation();
    Vec p2 = ps[2].getTranslation();
    Vec p3 = ps[3].getTranslation();
    Vec newX = (p1 - p0).NormalizeVec();
    Vec newCenter1 = p0 + ((p2 - p0).dot(newX)) * newX; // the corner point
     // compute the second 4pt
    for (size_t i=0; i<numJnts_2; i++) {
      std::vector<double> jnt(DoF_, 0);
      EigenVec2StdVec(jnt_mes_4pt_high.col(i), &jnt);
      int ret = JntToCart(jnt, &ps[i]);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "Scara FK error, code  " << ret
                << "can not do TCP calibration in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
    }
    // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
    p0 = ps[0].getTranslation();
    p1 = ps[1].getTranslation();
    p2 = ps[2].getTranslation();
    p3 = ps[3].getTranslation();
    newX = (p1 - p0).NormalizeVec();
    Vec newCenter2 = p0 + ((p2 - p0).dot(newX)) * newX;
    Rotation r = ps[0].getRotation();
    Vec tool_trans = r.Inverse() * (newCenter1 - newCenter2);
    Rotation tool_r(tool_trans);
    strs.str("");
    strs << GetName() << ":" << "tool axis trans=" << tool_trans.ToString() << std::endl;
    strs << "tool axis rot=" << tool_r.ToEigenMat() << std::endl;
    LOG_INFO(strs); 



    // now compute tool translation
    size_t numGroups = numMes_3 / 4;
    std::vector<Vec>  tcp_trans_array(numGroups+1);
    std::vector<Rotation> tcp_rot_array(numGroups+1);
    
    // first point in array
    tcp_trans_array[0] = newCenter2;
    tcp_rot_array[0] = r;

    for (size_t i=0; i< numGroups; i++) {
      // compute the corner point and orientation of each group (each 4 pts)
      size_t start_ind = 4 * i;
      size_t end_ind = 4 * (i+1);
      
      for (size_t j=start_ind; j < end_ind; j++) {
        std::vector<double> jnt(DoF_, 0);
        EigenVec2StdVec(jnt_mes_12pt.col(j), &jnt);
        int ret = JntToCart(jnt, &ps[j - start_ind]);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "Scara FK error, code  " << ret
                << "can not do TCP calibration in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          return ret;  
        }
      }
      // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
      p0 = ps[0].getTranslation();
      p1 = ps[1].getTranslation();
      p2 = ps[2].getTranslation();
      p3 = ps[3].getTranslation();
      newX = (p1 - p0).NormalizeVec();
      tcp_trans_array[i+1] = p0 + ((p2 - p0).dot(newX)) * newX;
      tcp_rot_array[i+1] = ps[0].getRotation();
    }
    // now ready to compute tcp

    // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
    Eigen::MatrixXd A(numGroups * 3, 3);
    Eigen::VectorXd b(numGroups * 3);
    for (size_t i=0; i < numGroups; i++) {
      // computing the Coefficient matrix
      A.block(i * 3, 0, 3, 3) = tcp_rot_array[i+1].ToEigenMat() - tcp_rot_array[0].ToEigenMat(); // dbase
      b.block(i * 3, 0, 3, 1) = - (tcp_trans_array[i+1] - tcp_trans_array[0]).ToEigenVec();
    }
    Eigen::MatrixXd BB = A;
    strs.str("");
    strs << GetName() << ":" << "before reduction: B=" << BB << std::endl;
    LOG_INFO(strs);
    if (DoF_ == 4) {  // scara
        BB = A.block(0, 0, numGroups * 3 , 2);  // needs to put inside the each subobject in the future
        //A.resize(num_jnt_measures - 1, 2);
    }
    strs.str("");
    strs << GetName() << ":" << "after reduction: B=" << BB << std::endl;
    strs<< "b=" << b << std::endl;
    LOG_INFO(strs);
    Eigen::MatrixXd ATA = BB.transpose() * BB;
    const double detV = ATA.determinant();
    strs.str("");
    strs << GetName() << ":" << "determinat is " << detV << std::endl;
    LOG_INFO(strs);
    if (detV < CALIB_SINGULAR_CONST) {
       for (size_t j=0; j < 3; j++) {
         ATA(j, j) += angle_ridge_scale_;
       }
    }
    Eigen::VectorXd outV = ATA.inverse() * BB.transpose() * b;

    final_tool_offset->setZero();  // init to 0
    for (size_t i=0; i < outV.size(); i++) {
      (*final_tool_offset)(i) = outV(i);
    }
    
    Quaternion q;
    tool_r.GetQuaternion(&q);
    (*final_tool_offset)(3) = q.w();
    (*final_tool_offset)(4) = q.x();
    (*final_tool_offset)(5) = q.y();
    (*final_tool_offset)(6) = q.z();
    strs.str("");
    strs<< GetName() << ":"  << "tcp trans=" << outV << std::endl;
    strs << "tcp rot=" << tool_r.ToString() << std::endl;
    LOG_INFO(strs);
    return 0;
}


// at least 3 pt measurements in determining the datum
// if 3pt, that is 3 pt methods like KUKA
// if 8pt, 2 pt on each side, then we set datum reference frame at the corner
int serialArm::ErrCompensateBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 EigenDRef<Eigen::VectorXd> *comp_base_uncal,
                                 EigenDRef<Eigen::VectorXd> *comp_base,
                                 EigenDRef<Eigen::MatrixXd> *origCart,
                                 EigenDRef<Eigen::MatrixXd> *compCart,
                                 EigenDRef<Eigen::VectorXd> *relq_cano,    // need to find out the desired orientation of laser in canonical model w.r.t. workpiece frame
                                 EigenDRef<Eigen::VectorXd> *relq_calib    // need to find out the desired orientaiton of laser in calibrated model w.r.t. ...
                                 ) {
    std::ostringstream strs;
    size_t numJnts = jnt_base_measures.cols();
    // here orig_base is the workpiece frame, should be reachable by IK
    if (jnt_base_measures.rows() < DoF_ || numJnts < 1 ||
         !origCart || !compCart || !relq_cano || !relq_calib) {
      strs.str("");
      strs << GetName() << ":" << "The input vector has wrong dimension in function "
            << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
    }
    if (!comp_base_uncal || !comp_base) {
      strs.str("");
      strs << GetName() << ":"  << "input comp_base is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }

    // initialize relative orientation as 0 w.r.t. workpiece frame
    (*relq_cano).setZero();
    (*relq_calib).setZero();

    Frame userBase; // default base
    Vec t(orig_tool.segment(0, 3)); //here tool only contains the trans. 
    Quaternion q(orig_tool(3), orig_tool(4), orig_tool(5), orig_tool(6));
    Frame userTool;
    userTool.setTranslation(t);  // user Tool
    userTool.setQuaternion(q);
    // three point method
    std::vector<Pose> ps(numJnts);
    std::vector<refPose> rps(numJnts);
    origCart->resize(9, numJnts);
    compCart->resize(9, numJnts);
    // first compute uncalibrated base frame
    SetUsingCalibratedModel(false);
    double sumYaw = 0, sumPitch = 0, sumRoll = 0;
    for (size_t i=0; i<numJnts; i++) {
      std::vector<double> jnt(DoF_, 0);
      EigenVec2StdVec(jnt_base_measures.col(i), &jnt);
      int ret = JntToCart(jnt, &ps[i]);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "Scara FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      refPose default_rps;
      default_rps.setDefaultPose(ps[i]);
      // get refPose under default base and userTool, and calibrated model
      default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
      Rotation r = rps[i].getRotation();
      double yaw,pitch,roll;  // Z,Y,X Euler angles 
      r.GetEulerZYX(&yaw, &pitch, &roll);
      sumYaw +=yaw; sumPitch += pitch; sumRoll += roll;
      strs.str("");
      strs << GetName() << ":" << "Canonical model: Orietation " << i << " = " << r.ToString() <<
           ", translation=" << rps[i].getTranslation().ToString() << std::endl;
      LOG_INFO(strs);
      origCart->col(i) = rps[i].ToEigenVecPose();
    }
    double avgYaw = sumYaw / numJnts;
    double avgPitch = sumPitch / numJnts;
    double avgRoll = sumRoll / numJnts;
    Rotation r_old(avgYaw,  avgPitch, avgRoll);  // average end-effector orientation during 8 pt method
    Rotation newr;
    
    comp_base_uncal->resize(7);
    if (numJnts == 1) {
      Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
      (*comp_base_uncal)(0) = newCenter.x();
      (*comp_base_uncal)(1) = newCenter.y();
      (*comp_base_uncal)(2) = newCenter.z();

      (*comp_base_uncal)(3) = 1;
      (*comp_base_uncal)(4) = 0;
      (*comp_base_uncal)(5) = 0;
      (*comp_base_uncal)(6) = 0;
    } else if (numJnts == 3) {
      Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
      (*comp_base_uncal)(0) = newCenter.x();
      (*comp_base_uncal)(1) = newCenter.y();
      (*comp_base_uncal)(2) = newCenter.z();
    
      Vec newX = (rps[1].getTranslation() - rps[0].getTranslation()).NormalizeVec();
      Vec tmpY = rps[2].getTranslation() - rps[1].getTranslation();
      Vec tmpY1 =  tmpY - (tmpY.dot(newX)) * newX;
      Vec newY = tmpY1.NormalizeVec();
      Vec newZ = newX * newY;
      newr.UnitX(newX);
      newr.UnitY(newY);
      newr.UnitZ(newZ);
      
      //Rotation newr(newX, newY, newZ);
      Quaternion q;
      newr.GetQuaternion(&q);
      (*comp_base_uncal)(3) = q.w();
      (*comp_base_uncal)(4) = q.x();
      (*comp_base_uncal)(5) = q.y();
      (*comp_base_uncal)(6) = q.z();
      strs.str("");
      strs << GetName() << ":" << "3pt base frame computation: uncalibrated frame, trans=" << newCenter.ToString() << ", rot=" << newr.ToString() << std::endl;
      LOG_INFO(strs);
    } else if (numJnts == 4) {   // 4 pt method
       // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
       Vec p0 = rps[0].getTranslation();
       Vec p1 = rps[1].getTranslation();
       Vec p2 = rps[2].getTranslation();
       Vec p3 = rps[3].getTranslation();
       Vec newX = (p1 - p0).NormalizeVec();
       Vec newY = (p3 - p2).NormalizeVec();
       Vec newZ = newX * newY;
       newr.UnitX(newX);
       newr.UnitY(newY);
       newr.UnitZ(newZ);
       //Rotation newr(newX, newY, newZ);
       Quaternion q;
       newr.GetQuaternion(&q);
       (*comp_base_uncal)(3) = q.w();
       (*comp_base_uncal)(4) = q.x();
       (*comp_base_uncal)(5) = q.y();
       (*comp_base_uncal)(6) = q.z();

       Vec newCenter = p0 + ((p2 - p0).dot(newX)) * newX; 
       (*comp_base_uncal)(0) = newCenter.x();
       (*comp_base_uncal)(1) = newCenter.y();
       (*comp_base_uncal)(2) = newCenter.z();
    } else if (numJnts == 8) {   // 8 pt method
         // pt 0, 1 determines x axis on the right, 2,3 is parallel to x axis on the left, 
         // pt 4, 5 on y axis on the top, and 6, 7 is parallel to y axis on the bottom
         //  top_right is the intersection pt between line 01 and line 45,
         // bot_left is the intersection pt between line 23, and line 67
       Eigen::MatrixXd ptIn(3, numJnts);
       for (size_t i=0; i < numJnts; i++) {
         Vec p = rps[i].getTranslation();
         ptIn.col(i) = p.ToEigenVec();
       }
       Eigen::Vector3d meanPt = ptIn.rowwise().mean();
       Eigen::MatrixXd tmp = ptIn.colwise() - meanPt; 
       // take out mean from all pts
       //for (size_t i=0; i < numJnts; i++) {
         //ptIn.col(i) = ptIn.col(i) - meanPt;
       //}

       // using svd decompsition to find the normal of 8-pt plane
       Eigen::JacobiSVD<Eigen::MatrixXd> svd(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
       Eigen::MatrixXd tmpV = svd.matrixV();
       Eigen::Vector3d pn = tmpV.col(2);
       Vec vn(pn); // convert to Vec 

       // now compute x and y
       Eigen::Vector3d newX1 = ptIn.col(1) - ptIn.col(0);
       Vec tx1(newX1);
       Vec vx1 = tx1.NormalizeVec();

       Eigen::Vector3d newX2 = ptIn.col(2) - ptIn.col(3);
       Vec tx2(newX2);
       Vec vx2 = tx2.NormalizeVec();
       Vec vx = (vx1 + vx2).NormalizeVec();


       Eigen::Vector3d newY1 = ptIn.col(5) - ptIn.col(4);
       Vec ty1(newY1);
       Vec vy1 = ty1.NormalizeVec();

       Eigen::Vector3d newY2 = ptIn.col(6) - ptIn.col(7);
       Vec ty2(newY2);
       Vec vy2 = ty2.NormalizeVec();
       Vec vy = (vy1 + vy2).NormalizeVec();

       Vec vz = vx * vy;
       // determine the actual normal of the 8pt plane
       if (vz.dot(vn) > 0) {
         vz = vn;
       } else {
         vz = -vn;
       }

       // obtain the actual vx and vy
       vx = (vx - vx.dot(vz) * vz).NormalizeVec();
       vy = vz * vx;
       // {vx, vy, vz forms a workobj coordinate frame}
       newr.UnitX(vx);
       newr.UnitY(vy);
       newr.UnitZ(vz);
       //Rotation newr(vx, vy, vz);
       Quaternion q;
       newr.GetQuaternion(&q);
       (*comp_base_uncal)(3) = q.w();
       (*comp_base_uncal)(4) = q.x();
       (*comp_base_uncal)(5) = q.y();
       (*comp_base_uncal)(6) = q.z();
       
     
      // now obtain the origin of the workobj coordinate frame
      Eigen::Vector3d ex = vx.ToEigenVec();
      Eigen::Vector3d ey = vy.ToEigenVec();
      // now we can compute projected 8 poitns
      Eigen::VectorXd xproj = ex.transpose() * tmp;
      Eigen::VectorXd yproj = ey.transpose() * tmp;

      // compute the coordinates of 4 corner
      double ytop = (yproj(1) + yproj(0)) / 2.0;
      double ybot = (yproj(2) + yproj(3)) / 2.0;
      double xtop = (xproj(4) + xproj(5)) / 2.0;
      double xbot = (xproj(6) + xproj(7)) / 2.0;
      // center of the figure
      double xcenter = (xtop + xbot) / 2.0;
      double ycenter = (ytop + ybot) / 2.0;
      strs.str("");
      strs << GetName() << ":" << "8 pt method in uncalibrated model, x-length= " << fabs(xtop - xbot) 
           << ", y-length= " << fabs(ytop -ybot) << std::endl;
      LOG_INFO(strs);

      Vec newCenter(meanPt);
      newCenter = newCenter + xcenter * vx + ycenter * vy;
      (*comp_base_uncal)(0) = newCenter.x();
      (*comp_base_uncal)(1) = newCenter.y();
      (*comp_base_uncal)(2) = newCenter.z();
    } else {
       strs.str("");
       strs << GetName() << ":" << "num of input joints in  " << __FUNCTION__ 
           << " at line " << __LINE__ << " is out of scope" << std::endl;
       LOG_ERROR(strs);
       return -1;
    }
    
    Rotation relR = newr.Inverse() * r_old;
    strs.str("");
    strs << GetName() << ":" << " work frame Euler angle = " << newr.ToString() << ", r_old Euler angle=" << r_old.ToString() << std::endl;
    LOG_INFO(strs);
    strs.str("");
    strs << GetName() << ":" << "Canonical model: desired orietation w.r.t. workcoord frame " << relR.ToString() << std::endl;
    LOG_INFO(strs); 
    Quaternion q1;
    relR.GetQuaternion(&q1);
    (*relq_cano)(0) = q1.w();
    (*relq_cano)(1) = q1.x();
    (*relq_cano)(2) = q1.y();
    (*relq_cano)(3) = q1.z();

    if (!isDHCalibrated_) {
      strs.str("");
      strs << GetName() << ":" << "Scara robot is not calibrated, "
         << "can not do error compensation in calibrated base, but use the original base"
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_INFO(strs);
      *comp_base = *comp_base_uncal;
      return 0;
    }
    
    
    // first compute calibrated base frame
    SetUsingCalibratedModel(true);
    sumYaw = 0; sumPitch = 0; sumRoll = 0;
    for (size_t i=0; i<numJnts; i++) {
      std::vector<double> jnt(DoF_, 0);
      EigenVec2StdVec(jnt_base_measures.col(i), &jnt);
      int ret = JntToCart(jnt, &ps[i]);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "Scara FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;  
      }
      refPose default_rps;
      default_rps.setDefaultPose(ps[i]);
      // get refPose under default base and userTool, and calibrated model
      default_rps.getPoseUnderNewRef(userBase, userTool, &rps[i]);
      Rotation r = rps[i].getRotation();
      double yaw,pitch,roll;  // Z,Y,X Euler angles 
      r.GetEulerZYX(&yaw, &pitch, &roll);
      sumYaw +=yaw; sumPitch += pitch; sumRoll += roll;
      strs.str("");
      strs << GetName() << ":" << "Calibrated model: Orientation " << i << " = " << r.ToString() 
           << ", translation=" << rps[i].getTranslation().ToString()
           << std::endl;
      LOG_INFO(strs);
      compCart->col(i) = rps[i].ToEigenVecPose();
    }
    avgYaw = sumYaw / numJnts;
    avgPitch = sumPitch / numJnts;
    avgRoll = sumRoll / numJnts;
    r_old.SetEulerZYX(avgYaw,  avgPitch, avgRoll);  // average end-effector orientation during 8 pt method    

    comp_base->resize(7);
    if (numJnts == 1) {
      Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
      (*comp_base)(0) = newCenter.x();
      (*comp_base)(1) = newCenter.y();
      (*comp_base)(2) = newCenter.z();

      (*comp_base)(3) = 1;
      (*comp_base)(4) = 0;
      (*comp_base)(5) = 0;
      (*comp_base)(6) = 0;
    } else if (numJnts == 3) {
      Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
      Vec newX = (rps[1].getTranslation() - rps[0].getTranslation()).NormalizeVec();
      Vec tmpY = rps[2].getTranslation() - rps[1].getTranslation();
      Vec tmpY1 =  tmpY - (tmpY.dot(newX)) * newX;
      Vec newY = tmpY1.NormalizeVec();
      Vec newZ = newX * newY;
      newr.UnitX(newX);
      newr.UnitY(newY);
      newr.UnitZ(newZ);

      (*comp_base)(0) = newCenter.x();
      (*comp_base)(1) = newCenter.y();
      (*comp_base)(2) = newCenter.z();
      Quaternion q;
      newr.GetQuaternion(&q);
      (*comp_base)(3) = q.w();
      (*comp_base)(4) = q.x();
      (*comp_base)(5) = q.y();
      (*comp_base)(6) = q.z();
    } else if (numJnts == 4) {   // 4 pt method
       // pt 0, 1 determines x axis, 2,3 y axis, origin is the intersection pt between line 01 and line 23
       Vec p0 = rps[0].getTranslation();
       Vec p1 = rps[1].getTranslation();
       Vec p2 = rps[2].getTranslation();
       Vec p3 = rps[3].getTranslation();
       Vec newX = (p1 - p0).NormalizeVec();
       Vec newY = (p3 - p2).NormalizeVec();
       Vec newZ = newX * newY;
       newr.UnitX(newX);
       newr.UnitY(newY);
       newr.UnitZ(newZ);
       // Rotation newr(newX, newY, newZ);
       Quaternion q;
       newr.GetQuaternion(&q);
       (*comp_base)(3) = q.w();
       (*comp_base)(4) = q.x();
       (*comp_base)(5) = q.y();
       (*comp_base)(6) = q.z();

       Vec newCenter = p0 + ((p2 - p0).dot(newX)) * newX; 
       (*comp_base)(0) = newCenter.x();
       (*comp_base)(1) = newCenter.y();
       (*comp_base)(2) = newCenter.z();
    } else if (numJnts == 8) {   // 8 pt method
       // pt 0, 1 determines x axis on the right, 2,3 is parallel to x axis on the left, 
       // pt 4, 5 on y axis on the top, and 6, 7 is parallel to y axis on the bottom
       //  top_right is the intersection pt between line 01 and line 45,
       // bot_left is the intersection pt between line 23, and line 67
       Eigen::MatrixXd ptIn(3, numJnts);
       for (size_t i=0; i < numJnts; i++) {
         Vec p = rps[i].getTranslation();
         ptIn.col(i) = p.ToEigenVec();
       }
       Eigen::Vector3d meanPt = ptIn.rowwise().mean();
       Eigen::MatrixXd tmp = ptIn.colwise() - meanPt; 
       // take out mean from all pts
       //for (size_t i=0; i < numJnts; i++) {
         //ptIn.col(i) = ptIn.col(i) - meanPt;
       //}

       // using svd decompsition to find the normal of 8-pt plane
       Eigen::JacobiSVD<Eigen::MatrixXd> svd(tmp.transpose(), Eigen::ComputeFullV | Eigen::ComputeFullU);
       Eigen::MatrixXd tmpV = svd.matrixV();
       Eigen::Vector3d pn = tmpV.col(2);
       Vec vn(pn); // convert to Vec 

       // now compute x and y
       Eigen::Vector3d newX1 = ptIn.col(1) - ptIn.col(0);
       Vec tx1(newX1);
       Vec vx1 = tx1.NormalizeVec();

       Eigen::Vector3d newX2 = ptIn.col(2) - ptIn.col(3);
       Vec tx2(newX2);
       Vec vx2 = tx2.NormalizeVec();
       Vec vx = (vx1 + vx2).NormalizeVec();


       Eigen::Vector3d newY1 = ptIn.col(5) - ptIn.col(4);
       Vec ty1(newY1);
       Vec vy1 = ty1.NormalizeVec();

       Eigen::Vector3d newY2 = ptIn.col(6) - ptIn.col(7);
       Vec ty2(newY2);
       Vec vy2 = ty2.NormalizeVec();
       Vec vy = (vy1 + vy2).NormalizeVec();

       Vec vz = vx * vy;
       // determine the actual normal of the 8pt plane
       if (vz.dot(vn) > 0) {
         vz = vn;
       } else {
         vz = -vn;
       }

       // obtain the actual vx and vy
       vx = (vx - vx.dot(vz) * vz).NormalizeVec();
       vy = vz * vx;
       // {vx, vy, vz forms a workobj coordinate frame}
       newr.UnitX(vx);
       newr.UnitY(vy);
       newr.UnitZ(vz);
       // Rotation newr(vx, vy, vz);
       Quaternion q;
       newr.GetQuaternion(&q);
       (*comp_base)(3) = q.w();
       (*comp_base)(4) = q.x();
       (*comp_base)(5) = q.y();
       (*comp_base)(6) = q.z();
      
       
     
      // now obtain the origin of the workobj coordinate frame
      Eigen::Vector3d ex = vx.ToEigenVec();
      Eigen::Vector3d ey = vy.ToEigenVec();
      // now we can compute projected 8 poitns
      Eigen::VectorXd xproj = ex.transpose() * tmp;
      Eigen::VectorXd yproj = ey.transpose() * tmp;

      // compute the coordinates of 4 corner
      double ytop = (yproj(1) + yproj(0)) / 2.0;
      double ybot = (yproj(2) + yproj(3)) / 2.0;
      double xtop = (xproj(4) + xproj(5)) / 2.0;
      double xbot = (xproj(6) + xproj(7)) / 2.0;
      strs.str("");
      strs << GetName() << ":" << "8 pt method in calibrated model, x-length= " << fabs(xtop - xbot) 
           << ", y-length= " << fabs(ytop -ybot) << std::endl;
      LOG_INFO(strs);

      // center of the figure
      double xcenter = (xtop + xbot) / 2.0;
      double ycenter = (ytop + ybot) / 2.0;

      Vec newCenter(meanPt);
      newCenter = newCenter + xcenter * vx + ycenter * vy;
      (*comp_base)(0) = newCenter.x();
      (*comp_base)(1) = newCenter.y();
      (*comp_base)(2) = newCenter.z();
    } else {
       strs.str("");
       strs << GetName() << ":" << "num of input joints in  " << __FUNCTION__ 
           << " at line " << __LINE__ << " is out of scope" << std::endl;
       LOG_ERROR(strs);
       return -1;
    }
    strs.str("");
    relR = newr.Inverse() * r_old;
    strs << GetName() << ":" << " work frame Euler angle = " << newr.ToString() << ", r_old Euler angle=" << r_old.ToString() << std::endl;
    LOG_INFO(strs);
    strs.str("");
    strs << GetName() << ":" << "calibrated model: desired orietation w.r.t. calib workcoord frame " << relR.ToString() << std::endl;
    LOG_INFO(strs);
    relR.GetQuaternion(&q1);
    (*relq_calib)(0) = q1.w();
    (*relq_calib)(1) = q1.x();
    (*relq_calib)(2) = q1.y();
    (*relq_calib)(3) = q1.z();
    return 0;
}

int serialArm::ErrCompCart(const refPose &p,
                           const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
                           refPose *cp) {
  std::ostringstream strs;
  if (!cp || origBase.rows() != 7) {
    strs.str("");
    strs << GetName() << ":" << "input cp is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  Pose ps;
  p.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
                                 // this is what scara kinematics does
  // in the following calculation, we all use uncalibrated, or canonical
  // model
  SetUsingCalibratedModel(false);
  // step 1: using canonical IK to compute ideal joint vector
  std::vector<double> init_jnt(DoF_, 0);
  int ret = CartToJnt(ps, &init_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "Scara IK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;  
  }
  // step 2, optimize jnt from init_jnt such that |f(cal_para, jnt) - ps|
  // is minimal
  std::vector<double> opt_jnt(DoF_,0);
  ret = OptimizeJntAfterCalib(init_jnt, p, &opt_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "Scara OptimizeJntAfterCalib error, code  " << ret
              << "can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;  
  }
  // step 3, using canonical FK to find the compensated trajectory
  ret = JntToCart(opt_jnt, &ps);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "Scara canonical FK error, code  " << ret
              << "can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;
  }
  refPose default_rps;
  default_rps.setDefaultPose(ps);
  Frame newTool;
  p.getTool(&newTool);

  Vec bv_old(origBase(0), origBase(1), origBase(2));
  Quaternion bq_old(origBase(3), origBase(4), origBase(5), origBase(6));
  Frame oldBase(bq_old, bv_old);

  default_rps.getPoseUnderNewRef(oldBase, newTool, cp);
  return 0;
}

int serialArm::ErrCompJnt(const refPose &p, std::vector<double> *cq) {
  std::ostringstream strs;  
  if (!cq) {
    strs.str("");
    strs << GetName() << ":" << "input cq is null in function " << __FUNCTION__ 
           << " at line " << __LINE__ << std::endl;
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
  Pose ps;
  p.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
                                 // this is what scara kinematics does
  // in the following calculation, we all use uncalibrated, or canonical
  // model
  SetUsingCalibratedModel(false);
  // step 1: using canonical IK to compute ideal joint vector
  std::vector<double> init_jnt(DoF_, 0);
  int ret = CartToJnt(ps, &init_jnt);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "Scara IK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;  
  }
  // step 2, optimize jnt from init_jnt such that |f(cal_para, *cq) - ps|
  // is minimal
  ret = OptimizeJntAfterCalib(init_jnt, p, cq);
  if (ret < 0) {
    strs.str("");
    strs << GetName() << ":" << "Scara OptimizeJntAfterCalib error, code  " << ret
              << "can not do error compensation in"
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return ret;  
  }
  return 0;
}

void serialArm::ConvertBranchFlag(const size_t inFlag,
                               std::vector<int> *branchFlags) const {
    std::ostringstream strs;
    if (!branchFlags) {
        strs.str("");
        strs << GetName() << ":" << "ConvertBranchFlag fails due to null input flag pointer"
                << " in " << __FUNCTION__
                << " at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return;
    }
   // for scara, there is only 1 branch flag: elbow (up or down)
    branchFlags->resize(3, eBranchLeft);
    // cur_pt[7] is the one which holding the branch info
    // elbow-right=1, elbow-left=0
    // branchFlags->at(0) = inFlag(0) > 0.5 ? 1 : 0;
    size_t flag = inFlag;

    int i = 2;
    while (flag >= 1) {
      branchFlags->at(i) = flag % 2;
      flag = std::floor(flag / 2.0);
      i--;
    }
    
    for (int j=0; j < i+1; j++) {
       branchFlags->at(j) = 0;
    }
}

void serialArm::ConvertMultiTurnFlag(const size_t inTurn,
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

//For scara we only use Vector4d
int  serialArm::ErrCompensationDH(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd &bestTool,
        const EigenDRef<Eigen::MatrixXd> &d_traj,
        EigenDRef<Eigen::MatrixXd> *md_traj) {
    std::ostringstream strs;
    if (!md_traj) {
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
    // setup new base and tool
    Frame newBase, newTool;
    Vec bv(calibBase(0), calibBase(1), calibBase(2));
    Quaternion bq(calibBase(3), calibBase(4), calibBase(5), calibBase(6));
    newBase.setQuaternion(bq);
    newBase.setTranslation(bv);
    Vec t(bestTool.segment(0, 3));
    newTool.setTranslation(t);
    Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
    newTool.setQuaternion(q);

    // in the following calculation, we all use uncalibrated, or canonical
    // model
    SetUsingCalibratedModel(false);
    // resize the output traj size to be same as d_traj
    size_t numPts = d_traj.cols();
    size_t numRows = d_traj.rows();
    md_traj->resize(numRows, numPts);
    

    for (size_t i=0; i < numPts; i++) {
        Eigen::VectorXd cur_pt = d_traj.col(i);
        Vec v(cur_pt(0), cur_pt(1), cur_pt(2));
        Quaternion quat(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
        size_t inFlag = cur_pt(7);
         // for scara, there is only 1 branch flag: elbow (up or down)
        std::vector<int>  branchFlags;
        ConvertBranchFlag(inFlag, &branchFlags);
        
        // convert joint turn data into ikJointTurns
        size_t tFlag = cur_pt(8);
        std::vector<int> ikJointTurns;
        ConvertMultiTurnFlag(tFlag, &ikJointTurns);
        // for the moment we will assume all joint turns are 0
        refPose rps, rps_out;
        rps.setFrame(Frame(quat,v));
        rps.setBranchFlags(branchFlags);
        rps.setJointTurns(ikJointTurns);
        rps.setBase(newBase);
        rps.setTool(newTool);

        int ret = ErrCompCart(rps, origBase, &rps_out) ;
        if (ret < 0) {
           strs.str("");
           strs << GetName() << ":" << "ErrCompCart error, code  " << ret
                << ", stop doing error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
           LOG_ERROR(strs);
           return ret;
        }
        md_traj->col(i) = rps_out.ToEigenVecPose();
    }
    return 0;
}

int  serialArm::ErrCompensationDH(
        const Frame &newBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Frame &oldBase,  // uncalibrated base that filtered path references to
        const Frame &newTool,
        const Pose &d_traj,
        Pose *md_traj,
        Eigen::VectorXd *d_j_traj,
        Eigen::VectorXd *md_j_traj,
        Pose *a_traj) {
    std::ostringstream strs;
    if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
      strs.str("");
      strs << GetName() << ":" << "Input pointers are null"
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

    strs.str("");
    strs << GetName() << ":" << "In traj. compensation: calib base= " << newBase.ToString(true)
         << ", uncalibrated base= " << oldBase.ToString(true)
         << ", desired tool=" << newTool.ToString(true)
         << std::endl;
    LOG_INFO(strs);

    // resize output variables
    d_j_traj->resize(DoF_);
    md_j_traj->resize(DoF_);

    // for the moment we will assume all joint turns are 0
    refPose rps_init, rps_init1;
    Pose ps;
    rps_init.setDefaultPose(d_traj);
    rps_init.setBase(newBase);
    rps_init.setTool(newTool);
    rps_init.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
                                 // this is what scara kinematics does
    // step 1: using canonical IK to compute ideal joint vector
    SetUsingCalibratedModel(false);
    std::vector<double> init_jnt(DoF_, 0), init_jnt1(DoF_, 0);
    strs.str("");
    strs << GetName() << ", IK: default cart is " << ps.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    int ret = CartToJnt(ps, &init_jnt);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " IK error, code  " << ret << ", cart is " << ps.ToString(false)
            << "can not do error compensation in"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;  
    } 

    Eigen::VectorXd vCol;
    StdVec2EigenVec(init_jnt, &vCol);
    strs.str("");
    strs << GetName() << ":" << " init_jnt=" << vCol << std::endl;
    LOG_INFO(strs);

    // for modules, their control often lies in joint space
    rps_init1.setDefaultPose(d_traj);
    rps_init1.setBase(oldBase);
    rps_init1.setTool(newTool);
    rps_init1.getDefaultPose(&ps); //get default pose w.r.t. default base and tool

    strs.str("");
    strs << GetName()  << ", IK: uncalib cart is " << ps.ToString(true)
    << std::endl;
    LOG_INFO(strs);
    ret = CartToJnt(ps, &init_jnt1);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << " IK error, code  " << ret << ", uncalib cart is " << ps.ToString(false)
            << "can not do error compensation in"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return ret;  
    } 

    StdVec2EigenVec(init_jnt1, &vCol);
    strs.str("");
    strs << GetName() << ":" << " uncalib init_jnt=" << vCol << std::endl;
    LOG_INFO(strs);
    *d_j_traj = vCol;

    // step 2, optimize jnt from init_jnt such that |f(calibrate_para, opt_jnt) - ps|
    // is minimal
    std::vector<double> opt_jnt(DoF_,0);
    // here rps_init is the desired traj. w.r.t calib base and  new tool
    ret = OptimizeJntAfterCalib(init_jnt, rps_init, &opt_jnt);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":" << " OptimizeJntAfterCalib error, code  " << ret
            << ", can not do error compension, using the original pose instead"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      *md_traj = d_traj;   // set same as original one
      *a_traj = d_traj;    // set same as original one
      // return ret;
    } else {
      StdVec2EigenVec(opt_jnt, &vCol);
      *md_j_traj = vCol;
      strs.str("");
      strs << GetName() << ":" << " opt_jnt=" << vCol << std::endl;
      LOG_INFO(strs);
      // step 3, using canonical FK to find the compensated trajectory
      ret = JntToCart(opt_jnt, &ps);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "canonical FK error, code  " << ret
            << "can not do error compensation in"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      std::vector<int> branchFlags, turnFlags;
      refPose default_rps, rps;
      default_rps.setDefaultPose(ps);
      default_rps.getPoseUnderNewRef(oldBase, newTool, &rps);  // get  relative pose w.r.t. oldBase, and new Tool
      rps.getBranchFlags(&branchFlags);
      rps.getJointTurns(&turnFlags);

      Frame tmp_rps;
      rps.getFrame(&tmp_rps);
      md_traj->setFrame(tmp_rps);   // in robot app. program, we use oldBase, and canonical kinematics
      md_traj->setBranchFlags(branchFlags);
      md_traj->setJointTurns(turnFlags);
      // step 4, using actual FK to find the actual trajectory
      SetUsingCalibratedModel(true);
      ret = JntToCart(opt_jnt, &ps);
      if (ret < 0) {
        strs.str("");
        strs << GetName() << ":" << "calibrated FK error, code  " << ret
            << "can not do error compensation in"
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return ret;
      }

      default_rps.setDefaultPose(ps);
      default_rps.getPoseUnderNewRef(Frame(), newTool, &rps);
      strs.str("");
      strs << GetName() << ":" << "under default base and new tool, comp rps after comp is " << rps.ToString(false) << std::endl;
      LOG_INFO(strs);
      default_rps.getPoseUnderNewRef(newBase, newTool, &rps);  // get relative to  new Base, which is used for comparing with desired pose
      rps.getBranchFlags(&branchFlags);
      rps.getJointTurns(&turnFlags);
      rps.getFrame(&tmp_rps);
      a_traj->setFrame(tmp_rps);
      a_traj->setBranchFlags(branchFlags);
      a_traj->setJointTurns(turnFlags);
      strs.str("");
      strs << GetName() << ":" << "under new base and tool, desired rps is " << rps_init.ToString(false) 
              <<  "under new Base and tool, comp rps is " << rps.ToString(false) << std::endl;
      LOG_INFO(strs);
    }
    return 0;
}

int  serialArm::ErrCompensationDH(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd & bestTool,  // user tool
        /*d_traj is w.r.t. bestBase and bestTool, note: here bestBase and bestTool
         are defined in robot system, irrespective of measuring frame*/
        const EigenDRef<Eigen::MatrixXd> &d_traj,
        EigenDRef<Eigen::MatrixXd> *md_traj,
        EigenDRef<Eigen::MatrixXd> *d_j_traj,
        EigenDRef<Eigen::MatrixXd> *md_j_traj,
        EigenDRef<Eigen::MatrixXd> *a_traj) {
    std::ostringstream strs;
    if (!md_traj || !d_j_traj || !md_j_traj || !a_traj) {
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
    // generate base and tool frame from the input data
    if (calibBase.rows() !=7  || origBase.rows() != 7) {
      strs.str("");
      strs << GetName() << ":" << "Input Base data is not translation + quaternion"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_PARA_WRONG_DIM;   
    }
    strs.str("");
    strs << GetName() << ":" << "In traj. compensation: calib base= " << calibBase
         << ", uncalibrated base= " << origBase
         << ", desired tool=" << bestTool // << ", base_off= "
            // << base_off_c_.ToString(false)
             << std::endl;
    LOG_INFO(strs);
    Frame newBase, newTool;
    Vec bv(calibBase(0), calibBase(1), calibBase(2));
    Quaternion bq(calibBase(3), calibBase(4), calibBase(5), calibBase(6));
    newBase.setQuaternion(bq);
    newBase.setTranslation(bv);
    Vec t(bestTool.segment(0, 3));
    newTool.setTranslation(t);
    Quaternion q(bestTool(3), bestTool(4), bestTool(5), bestTool(6));
    newTool.setQuaternion(q);

    Vec bv_old(origBase(0), origBase(1), origBase(2));
    Quaternion bq_old(origBase(3), origBase(4), origBase(5), origBase(6));
    Frame oldBase(bq_old, bv_old);
        
    size_t numPts = d_traj.cols();
    size_t numRows = d_traj.rows();
    // resize output variables
    md_traj->resize(numRows, numPts);
    d_j_traj->resize(DoF_, numPts);
    md_j_traj->resize(DoF_, numPts);
    a_traj->resize(numRows, numPts);

    for (size_t i=0; i < numPts; i++) {
        Eigen::VectorXd cur_pt = d_traj.col(i);
        Vec v(cur_pt(0), cur_pt(1), cur_pt(2));
        Quaternion quat(cur_pt(3), cur_pt(4), cur_pt(5), cur_pt(6));
        size_t inFlag = cur_pt(7);
         // for scara, there is only 1 branch flag: elbow (up or down), and for six-axis robot, there are 3 flags
        std::vector<int>  branchFlags;
        ConvertBranchFlag(inFlag, &branchFlags);
        
        // convert joint turn data into ikJointTurns
        size_t tFlag = cur_pt(8);
        std::vector<int> ikJointTurns;
        ConvertMultiTurnFlag(tFlag, &ikJointTurns);
        // for the moment we will assume all joint turns are 0
        refPose rps_init, rps_init1;
        Pose ps;
        rps_init.setFrame(Frame(quat,v));
        rps_init.setBranchFlags(branchFlags);
        rps_init.setJointTurns(ikJointTurns);
        rps_init.setBase(newBase);
        rps_init.setTool(newTool);
        rps_init.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
                                 // this is what scara kinematics does
        // step 1: using canonical IK to compute ideal joint vector
        SetUsingCalibratedModel(false);
        std::vector<double> init_jnt(DoF_, 0), init_jnt1(DoF_, 0);
        strs.str("");
        strs << GetName() << ", pt " << i << ", IK: default cart is " << ps.ToString(true)
        << std::endl;
        LOG_INFO(strs);
        int ret = CartToJnt(ps, &init_jnt);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " IK error, code  " << ret << ", cart is " << ps.ToString(false)
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
           return ret;  
        } 

        Eigen::VectorXd vCol;
        StdVec2EigenVec(init_jnt, &vCol);
        strs.str("");
        strs << GetName() << ":" << " init_jnt=" << vCol << std::endl;
        LOG_INFO(strs);

        // for modules, their control often lies in joint space
        rps_init1.setFrame(Frame(quat,v));
        rps_init1.setBranchFlags(branchFlags);
        rps_init1.setJointTurns(ikJointTurns);
        rps_init1.setBase(oldBase);
        rps_init1.setTool(newTool);
        rps_init1.getDefaultPose(&ps); //get default pose w.r.t. default base and tool

        strs.str("");
        strs << GetName() << ", pt " << i << ", IK: uncalib cart is " << ps.ToString(true)
        << std::endl;
        LOG_INFO(strs);
        ret = CartToJnt(ps, &init_jnt1);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << " IK error, code  " << ret << ", uncalib cart is " << ps.ToString(false)
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
           return ret;  
        } 

        StdVec2EigenVec(init_jnt1, &vCol);
        strs.str("");
        strs << GetName() << ":" << " uncalib init_jnt=" << vCol << std::endl;
        LOG_INFO(strs);
        d_j_traj->col(i) = vCol;

        // step 2, optimize jnt from init_jnt such that |f(calibrate_para, opt_jnt) - ps|
        // is minimal
        std::vector<double> opt_jnt(DoF_,0);
        // here rps_init is the desired traj. w.r.t calib base and  new tool
        ret = OptimizeJntAfterCalib(init_jnt, rps_init, &opt_jnt);
        if (ret < 0) {
          strs.str("");
          strs << GetName() << ":" << "Scara OptimizeJntAfterCalib error, code  " << ret
                << ", for point index " << i
                << ", can not do error compension, using the original pose instead"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
          LOG_ERROR(strs);
          md_traj->col(i) = d_traj.col(i);   // set same as original one
          a_traj->col(i) = d_traj.col(i);    // set same as original one
          // return ret;
        } else {
          StdVec2EigenVec(opt_jnt, &vCol);
          md_j_traj->col(i) = vCol;
          strs.str("");
          strs << GetName() << ":" << " opt_jnt=" << vCol << std::endl;
          LOG_INFO(strs);
          // step 3, using canonical FK to find the compensated trajectory
          ret = JntToCart(opt_jnt, &ps);
          if (ret < 0) {
            strs.str("");
            strs << GetName() << ":" << "Scara canonical FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret;
          }
          // ps.setBranchFlags(branchFlags);  // override branch flags
          //std::vector<int> flgs;
          //ps.getBranchFlags(&flgs);
          //std::cout << "init_jnt[1] = " << init_jnt[1] << "opt_jnt(1) =" << opt_jnt[1] << std::endl;
          // std::cout << "ps.falg = " << flgs[0] << std::endl;

          refPose default_rps, rps;
          default_rps.setDefaultPose(ps);
          default_rps.getPoseUnderNewRef(oldBase, newTool, &rps);  // get  relative pose w.r.t. oldBase, and new Tool
        
          md_traj->col(i) = rps.ToEigenVecPose();   // in robot app. program, we use oldBase, and canonical kinematics
        
          // step 4, using actual FK to find the actual trajectory
          SetUsingCalibratedModel(true);
          ret = JntToCart(opt_jnt, &ps);
          if (ret < 0) {
            strs.str("");

            strs << GetName() << ":" << "Scara canonical FK error, code  " << ret
                << "can not do error compensation in"
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
            LOG_ERROR(strs);
            return ret;
          }

          default_rps.setDefaultPose(ps);
          default_rps.getPoseUnderNewRef(Frame(), newTool, &rps);
          strs.str("");
          strs << GetName() << ":" << "under default base and new tool, comp rps after comp is " << rps.ToString(false) << std::endl;
          LOG_INFO(strs);
          default_rps.getPoseUnderNewRef(newBase, newTool, &rps);  // get relative to  new Base, which is used for comparing with desired pose
          a_traj->col(i) = rps.ToEigenVecPose();
          strs.str("");
          strs << GetName() << ":" << "under new base and tool, desired rps is " << rps_init.ToString(false) 
                 <<  "under new Base and tool, comp rps is " << rps.ToString(false) << std::endl;
          LOG_INFO(strs);
        }
    }
    return 0;
}

int serialArm::HomotopyAlg(const std::vector<double> &init_jnt0,
        const Vec &toolOffset, std::vector<double> *init_jnt) {
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
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
      if (ret < 0) {
           return ret;
      }
      // we get R_04, the rotation matrix between 0 and 4
      Rotation r_04 = p.getRotation();  
      Vec t_04 = p.getTranslation();
      Vec t_e = r_04 * toolOffset + t_04;
      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      // compute the translational Jacobian w.r.t. origin of default T.
      Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;
      
      Eigen::MatrixXd  Js_t, Js_r, Js_t1, Js_r1;
      // pick joint angle related sub jacobians, because we want to 
      // find joint angles correspond to calibrated parameters
      PickSubJacobian(Jt_tmp, Jp_r, &Js_t, &Js_r, true);
      size_t rowTrans = Js_t.rows();
      size_t  rowRot = Js_r.rows();
      size_t  tolCols =  Jt_tmp.cols();
      A.resize(rowTrans + rowRot, rowTrans+rowRot);
      B.resize(rowTrans + rowRot, tolCols);
      if (rowTrans > 0) {
        A.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
      }
      if (rowRot > 0) {
        A.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;
      }

      // pick translation and rotational jacobian for parameter part.
      PickSubJacobianForPara(Jt_tmp, Jp_r, &Js_t1, &Js_r1, true);
      rowTrans = Js_t1.rows();
      rowRot = Js_r1.rows();
      if (rowTrans > 0) {
        B.block(0, 0, rowTrans, tolCols) = Js_t1;
      }
      if (rowRot > 0) {
        B.block(rowTrans, 0, rowRot, tolCols) = Js_r1;
      } 
      
      double detA = A.determinant();
      if (fabs(detA) < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << ":" << "Scara calibration regression matrix A is singular "
              << " even with depend columns removed "
              << " with detA= " << detA << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_JAC_SINGULAR;
      }
      Eigen::VectorXd delta_t = -A.inverse() * B * var_para;
      jnt0 += delta_t;
    }
    EigenVec2StdVec(jnt0, init_jnt);
    return 0;
}
int serialArm::OptimizeJntAfterCalib(const std::vector<double> &init_jnt0,
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
    Vec userToolOffset = userTool.getTranslation();
  
    // convert ps to  cps (under default base frame, but under the same userTool)
    refPose cps;
    // base_off_c_ replaced by default base, because in traj. compensation, the desired traj
    // is always w.r.t. robot base (or default frame). Sensor frame will not be used anymore
    ps.getPoseUnderNewRef(defaultBaseOff_, userTool, &cps);
    strs.str("");
    strs << GetName() << ":" << "before comp, under default base and  new tool, desired rps is " << cps.ToString(false) << std::endl;
    LOG_INFO(strs);
    // using homotopty method to find a rough solution to 
    // f(calib_para, mid_jnt) ~= f(old_para, init_jnt)
    std::vector<double> init_jnt(DoF_, 0);
    if (HomotopyAlg(init_jnt0, userToolOffset, &init_jnt) < 0) {
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
    double yaw_d, pitch_d, roll_d;
    if (!d_r.GetEulerZYX(&yaw_d, &pitch_d, &roll_d)) {
          return -61;
    }
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
      int ret = CalcJacobian(tmp_para, &p, &Jp_t, &Jp_r, true);
      if (ret < 0) {
           return ret;
      }
      // we get R_04, the rotation matrix between 0 and flage
      Rotation r_04 = p.getRotation();
      Rotation r_final = r_04 * userTool.getRotation();
      double yaw, pitch, roll;
      if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
        strs.str("");
        strs << GetName() << ":" << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line " << __LINE__
        << std::endl;
        LOG_ERROR(strs);
        return -ERR_ROT2EULER_SINGULAR;
      }
      
      Vec t_04 = p.getTranslation();
      Vec t_e = r_04 * userToolOffset + t_04;
      Eigen::Matrix3d t_e_hat;
      t_e.ToHat(&t_e_hat);
      // compute the translational Jacobian w.r.t. origin of default T.
      Eigen::MatrixXd Jt_tmp = Jp_t - t_e_hat * Jp_r;
      Eigen::Vector3d errT= (d_t - t_e).ToEigenVec();
      Eigen::Vector3d errR(roll_d - roll, pitch_d - pitch, yaw_d -yaw);

      Eigen::Vector3d Eubd(roll_d, pitch_d, yaw_d); 
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
              << " with detA= " << detEulerDiff << ", Eub=" << Eub << ", Eubd=" << Eubd << ", EulerDiff=" << EulerDiff << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        //  return -ERR_CALIB_JAC_SINGULAR;
        EulerDiff += 0.00001 * Eigen::Matrix3d::Identity(3, 3);
      }
      Eigen::MatrixXd Jr_tmp = EulerDiff.inverse() * Jp_r;  //  Jr_tmp * d para = errR
      Eigen::MatrixXd Js_t, Js_r;
      // get joint vector related jacobian
      PickSubJacobian(Jt_tmp, Jr_tmp, &Js_t, &Js_r, true);
      
      size_t rowTrans = Js_t.rows();
      size_t  rowRot = Js_r.rows();

      A.resize(rowTrans + rowRot, rowTrans+rowRot);
      b.resize(rowTrans + rowRot);
      A.block(0, 0, rowTrans, rowTrans + rowRot) = Js_t;
      A.block(rowTrans, 0, rowRot, rowTrans + rowRot) = Js_r;
      
      double tmp_err = PickCartErr(errT, errR, &b, true); // given full error vector, pick those mathing with robot type
      Eigen::VectorXd delta_t(DoF_);
        
      double detA = A.determinant();
      if (fabs(detA) < CALIB_SINGULAR_CONST) {
        strs.str("");
        strs << GetName() << ":" << "calibration regression matrix A is singular, A= " << A
              << " even with depend columns removed "
              << " with detA= " << detA << " , in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return -ERR_CALIB_JAC_SINGULAR;
      }
      delta_t = A.inverse() * b;
      jnt_best_iter = jnt_tmp;
      // dynamic step size adjustment
      for (size_t i=0; i < numSteps; i++) {
         jnt_iter = jnt_tmp + (i+1) * step_size * delta_t;
         UpdateDH(kine_para, jnt_iter,  &tmp_para); 
         Eigen::MatrixXd Jp_t1, Jp_r1;
         Pose p1;
         // compute the expected values from known canonical kinematic parameters
         // i.e., not-calibrated parameter set
         ret = CalcJacobian(tmp_para, &p1, &Jp_t1, &Jp_r1, true);
         if (ret < 0) {
           return ret;
         }
         
         // we get R_04, the rotation matrix between 0 and flange
         Rotation r = p1.getRotation();
         Rotation r_final = r * userTool.getRotation();
         if (!r_final.GetEulerZYX(&yaw, &pitch, &roll)) {
           strs.str("");
           strs << GetName() << ":" << "GetEulerZYX fails in function " << __FUNCTION__ << ", at line " << __LINE__
           << std::endl;
           LOG_ERROR(strs);
           return -ERR_ROT2EULER_SINGULAR;
         }
         t_e = r * userToolOffset + p1.getTranslation();
         errT= (d_t - t_e).ToEigenVec();
         errR(0) = roll_d - roll; 
         errR(1) = pitch_d - pitch;
         errR(2) = yaw_d - yaw;
         double err = PickCartErr(errT, errR, &b, true); // given full error vector, pick those mathing with robot type
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


