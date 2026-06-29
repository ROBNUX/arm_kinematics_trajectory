#include "robnux_kinematics_map/UJNT.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::UJNT, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

UJNT::UJNT(): serialArm(2)  {
}

int UJNT::CartToJnt(const Pose& pos, Eigen::VectorXd& q) {
    std::ostringstream strs;
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

    if (q.size() != DoF_) {
      q.resize(DoF_);
    }
  
    //get default frame
    Frame tip;
    pos.getFrame(&tip);

    // get relative frame
    Frame relTip = defaultBaseOff_.Inverse() *  tip;

    Rotation r =  relTip.getRotation();
    Vec rx = r.UnitX();   // we solve yaw and pitch from rx
    Vec z1(0,0,1);

    double calpha = rx.dot(z1);
    double yaw, pitch;
    pitch = acos(calpha) - M_PI / 2.0 ;
 
    Vec rx_proj = rx - calpha * z1;
    yaw = atan2(rx_proj.y(), rx_proj.x());
    if (branch[2]) {   // nonflip
       q(0) = yaw;
       q(1) = -pitch;
    } else {   // flip solution
       q(1) = -(M_PI - pitch);
       if (yaw >= 0) {
          q(0) = yaw - M_PI;
       } else {
          q(0) = yaw + M_PI;
       }
    }
   
    std::vector<int>  jointTurns;
    pos.getJointTurns(&jointTurns);
  
    for (size_t i = 0; i < DoF_; i++) {
         // note sometimes, joint 0 could be running more than [-pi, pi]
         double turn = std::floor(q(i) / (2 * M_PI));
         double tmp_q = q(i) - turn * 2 * M_PI;
         // then make sure [-PI, PI]
         // to have 0 turn here
        if (tmp_q > M_PI) {
          turn += 1;
        }
        q(i) += (jointTurns[i] - turn) * 2 * M_PI;  // recall only joint 3 might moving more than 1 turn
        // subtract theta offset
        q(i) = (q(i) - theta_[i]) / pitch_(i);
    }
    return 0;
}

int UJNT::JntToCart(const Eigen::VectorXd& q,
                    Pose& p) {
    std::ostringstream strs;
    Eigen::VectorXd a_tmp, alpha_tmp, beta_tmp,  d_tmp, theta_tmp;
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

    Frame tmp = defaultBaseOff_;
    for (size_t i=0; i < DoF_; i++) {
      tmp = tmp * Frame::DH_Craig1989(a_tmp[i],
                                      alpha_tmp[i],
                                      d_tmp[i],
                                      theta_tmp[i]);
    }

    // multiply this to make sure  the final rotation is R_x(alpha[0])R_z(yaw)R_y(pitch) for easy IK above
    Rotation r = Rotation::RotX(-M_PI / 2.0);
    Frame tmp1;
    tmp1.setRotation(r);
    tmp = tmp * tmp1;
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

int UJNT::CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    bool world_jac) {
  int ret = serialArm::CalcJacobian(kine_para, p, Jp_t, Jp_r, world_jac);
  if (ret < 0) {
    return ret;
  }
  
  Frame tmp;
  p.getFrame(&tmp);
  Rotation r = Rotation::RotX(-M_PI / 2.0);
  Frame tmp1;
  tmp1.setRotation(r);
  tmp = tmp * tmp1;
  p.setFrame(tmp);
  return 0;
}

void  UJNT::UpdateConfigTurn(const Eigen::VectorXd& theta,
                             const Eigen::VectorXd& d,
                             std::vector<int>&  branchFlags,
                             std::vector<int>&  jointTurns) const {
    std::ostringstream strs;
    // for UJNT, there are 0 flags, so all set to 0 (eBranchLeft)
    branchFlags.resize(3, eBranchLeft);
    Rotation r=Rotation::RotY(theta[1]);
    Vec v = r.UnitX();
    if (v.x() >= 0) {   //  non-flip  (right half circle, in zx plane)
      branchFlags.at(2) = eBranchRight;
    } else {  // flip (or left half circle in zx plane)
      branchFlags.at(2) = eBranchLeft;
    }
    // DoF turn flags, shall all be 0, because it is prismatic
    jointTurns.resize(DoF_, 0);
    Eigen::VectorXd q_tmp = theta;
    double tmp_q = 0;
    for (size_t i=0; i < DoF_; i++) {
      jointTurns.at(i) = std::floor(q_tmp[i] / (2 * M_PI));
      tmp_q = q_tmp[i] - jointTurns.at(i) * 2 * M_PI;
      // then make sure [-PI, PI]
      // to have 0 turn here
      if (tmp_q > M_PI) {
        jointTurns.at(i) += 1;
      }
    }            
}

bool UJNT::PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                           const Eigen::MatrixXd& Jp_r,
                                Eigen::MatrixXd& Js_t,
                                Eigen::MatrixXd& Js_r,
                                bool reduction) {
     std::ostringstream strs;
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
    
    Js_t.resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      Js_t.col(i) = Jp_t.col(4 * i + 2) * pitch_(i);
    }
    Js_r.resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      Js_r.col(i) = Jp_r.col(4 * i + 2) * pitch_(i);
    }      
    return true;
}

double  UJNT::PickCartErr(const Eigen::Vector3d& errT,
                          const Eigen::Vector3d& errR, 
                          Eigen::VectorXd& b,
                          bool reduction) {
    std::ostringstream strs;
    double val = errT.norm();
    b.resize(3);
    b.block(0, 0, 3, 1) = errT;
    return val;
}

void UJNT::UpdateDH(const Eigen::VectorXd& orig_dh,
                    const Eigen::VectorXd& jnt,
                    Eigen::VectorXd& new_dh) const {
    std::ostringstream strs;
    if (orig_dh.size() != 5 * DoF_ ||  jnt.size() < DoF_) {
        strs.str("");
        strs << GetName() << " input parameters have wrong size, origin dh size= " << 
             orig_dh.size() << ", jnt size =" << jnt.size()   
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    new_dh = orig_dh;
    for (size_t i=0; i < DoF_; i++) {
       new_dh(2 * DoF_ + i) = orig_dh[2 * DoF_ + i] + jnt(i) * pitch_(i);
    }
}


void UJNT::UpdateDH(const Eigen::VectorXd& jnt,
              Eigen::VectorXd& theta,
              Eigen::VectorXd& d) const {
  std::ostringstream strs;
  if (jnt.size() < DoF_ ||  theta.size() != DoF_ || d.size() != DoF_) {
       strs.str("");
       strs << GetName() << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }

  for (size_t i=0; i < DoF_; i++) {
    theta(i) += jnt(i) * pitch_(i);
  }
}

}