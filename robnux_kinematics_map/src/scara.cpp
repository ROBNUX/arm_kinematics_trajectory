#include "robnux_kinematics_map/scara.hpp"
#include "robnux_kdl_common/pose.hpp"
#include <cmath>

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::Scara, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

Scara::Scara(): serialArm(4) {}

int Scara::CartToJnt(const Pose &pos, Eigen::VectorXd& q) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << "Scara geometric parameters are not initialized"
              << " in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;

    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED; 
  }

  //get default frame
  Frame tip;
  pos.getFrame(&tip);
  // get relative frame
  Frame relTip = defaultBaseOff_.Inverse() *  tip;

  Vec p = relTip.getTranslation();
  Quaternion quat = relTip.getQuaternion();
    
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
  // this analytic IK is using canonical robot model, for non-canonical 
  // model (e.g., model after calibration) requires using GI-based iteration
  // method
  if (useCalibrated_) {
    strs.str("");
    strs << "default CartToJnt function must using canonical kinematic"
              " model in" << __FUNCTION__
              << ", at line " << __LINE__  << std::endl;
    {
      std::string _tmp = strs.str();
      LOG_ERROR(_tmp);
    }
    return -ERR_ROB_DEFAULT_IK_NOT_CANO;
  }

  //computer prismatic joint value
  // recall here d_[2] is the offset value of 0 of this prismatic axis
  q(2) =  p.z() - (d_[0] + d_[1] + d_[2] + d_[3]);
  double xy_length = sqrt(p.x() * p.x() + p.y() * p.y());
  if (xy_length > a_[1] + a_[2]  || xy_length < fabs(a_[1] -a_[2])) { // then out of reach
    strs.str("");
    strs << "exception CartToJnt, scara, IK fails due to desire pose out"
              <<" of reach in " << __FUNCTION__
              <<" at line " << __LINE__ << std::endl;
    {
      std::string _tmp = strs.str();
      LOG_ERROR(_tmp);
    }
    return -ERR_ROB_IK_OUTOF_REACH;
  }
  // the orientation angle from origin to the origin of tool frame
  // in the XY-plane
  double angle_xy = atan2(p.y(), p.x());
  //angle between upper link and the vector (p.(x), p.(y))
  double offset_angle = acos((a_[1] * a_[1] + xy_length * xy_length -
                        a_[2] * a_[2]) / (2 * a_[1] * xy_length));
            
    if (branch[0]) {// if the configuration is righty
      q(0) = angle_xy - offset_angle;
    } else {
      q(0) = angle_xy + offset_angle;
    }
    q(1) = atan2(p.y() - a_[1] * sin(q(0)),
                 p.x() - a_[1] * cos(q(0))) - q(0);
            
    std::vector<int>  jointTurns;
    pos.getJointTurns(&jointTurns);
    double yaw, pitch, roll;
    if (!quat.GetEulerZYX(&yaw, &pitch, &roll)) {
      strs.str("");
      strs << "exception, IK fails due to Quaternion to YPR error"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROT2EULER_SINGULAR;
    }
    q(3) = yaw - q(0) - q(1);

    for (size_t i = 0; i < DoF_; i++) {
      if (i != 2) {
         // note sometimes, joint 0 could be running more than [-pi, pi]
         double turn = std::floor(q(i) / (2 * M_PI));
         double tmp_q = q(i) - turn * 2 * M_PI;
         // then make sure [-PI, PI]
         // to have 0 turn here
        if (tmp_q > M_PI) {
          turn += 1;
        }
        // q->at(3) -= jointTurns3 * 2 * PI;
        q(i) += (jointTurns[i] - turn) * 2 * M_PI;  // recall only joint 3 might moving more than 1 turn
      }
    }
    // last step: q->at(0,1,3) requires to subtract their zero offsets
    q(0) -= theta_[0];
    q(1) -= theta_[1];
    q(3) -= theta_[3];
    return 0;
}

void  Scara::UpdateConfigTurn(const Eigen::VectorXd& theta,
                              const Eigen::VectorXd& d,
                              std::vector<int>& branchFlags,
                              std::vector<int>& jointTurns) const {
    std::ostringstream strs;
    // for scara, there is only 1 branch flag: elbow (up or down)
    branchFlags.resize(3, eBranchLeft);
    // 4 turn flags, actually only 3 turn flags (because joint 3 is prismatic
    jointTurns.resize(4, 0);
    Eigen::VectorXd q_tmp = theta;
    q_tmp(2) = d(2);
    double tmp_q = 0;
    for (size_t i=0; i < DoF_; i++) {
      if (i != 2) {
         jointTurns[i] = std::floor(q_tmp(i) / (2 * M_PI));
         tmp_q = q_tmp(i) - jointTurns[i] * 2 * M_PI;
         // then make sure [-PI, PI]
         // to have 0 turn here
         if (tmp_q > M_PI) {
             jointTurns[i] += 1;
         }
      }
      if (i==1) {
        if (tmp_q >= 0 && tmp_q < M_PI ) {
           branchFlags[0] = eBranchRight;  // righty
        } else { 
           branchFlags[0] = eBranchLeft;  // lefty
        }
      }
    }            
}

bool Scara::PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                            const Eigen::MatrixXd& Jp_r,
                            Eigen::MatrixXd& Js_t,
                            Eigen::MatrixXd& Js_r,
                            const bool reduction) {
     std::ostringstream strs;
     size_t row_t = Jp_t.rows();
     size_t col_t = Jp_t.cols();
     size_t row_r = Jp_r.rows();
     size_t col_r = Jp_r.cols();
     if (row_t < 3 || col_t < 4 || row_r <3 || col_r < 4) {
        strs.str("");
        strs << " input Jacobian matrices have wrong dimension "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
     }
    
    Js_t.resize(3, 4);
    if (reduction) {
       Js_r.resize(1, 4);
    } else {
       Js_r.resize(3, 4);
    }
    for (size_t i=0; i < DoF_; i++) {
      if (i != 2) { // for theta[0], theta[1], theta[3] parameters
         Js_t.col(i) = Jp_t.col(5 * i + 2);
         if (reduction) {
            Js_r(i) = Jp_r(2, 5 * i + 2);
         } else {
            Js_r.col(i) = Jp_r.col(5 * i  + 2);
         }
      } else { // for d[2] parameter
         Js_t.col(i) = Jp_t.col(5 * i + 3);
         if (reduction) {
            Js_r(i) = Jp_r(2, 5 * i + 3); 
         } else {
            Js_r.col(i) = Jp_r.col(5 * i + 3);
         }
      }
    }      
    return true;
}

double  Scara::PickCartErr(const Eigen::Vector3d& errT,
                           const Eigen::Vector3d& errR, 
                           Eigen::VectorXd& b,
                           const bool reduction) {
    double val;
    if (reduction) {
      b.resize(4);
      b.block(0, 0, 3, 1) = errT;
      b(3) = errR(2);
      val = errT.norm() + fabs(errR(2));
    } else {
      b.resize(6);
      b.block(0, 0, 3, 1) = errT;
      b.block(3, 0, 3, 1) = errR;
      val = errT.norm() + errR.norm();
    }
    return val;
}

void Scara::UpdateDH(const Eigen::VectorXd& orig_dh,
                     const Eigen::VectorXd& jnt,
                     Eigen::VectorXd& new_dh) const {
    std::ostringstream strs;
    if (new_dh.size() != orig_dh.size()) {
        strs.str("");
        strs << " input new_dh size is wrong"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
    }
    if (orig_dh.size() != 4 * DoF_ ||  jnt.size() < DoF_) {
        strs.str("");
        strs << " input parameters have wrong size, origin dh size= " << 
             orig_dh.size() << ", jnt size =" << jnt.size()   
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
    }
    new_dh = orig_dh;
    new_dh(8) = orig_dh(8) + jnt(0);
    new_dh(9) = orig_dh(9) + jnt(1);
    new_dh(11) = orig_dh(11) + jnt(3);
    new_dh(14) = orig_dh(14) + jnt(2);
}


void Scara::UpdateDH(const Eigen::VectorXd& jnt,
              Eigen::VectorXd& theta,
              Eigen::VectorXd& d) const {
  std::ostringstream strs;
  if (jnt.size() < DoF_ ||  theta.size() != DoF_ || d.size() != DoF_) {
       strs.str("");
       strs << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return;
  }
  d(2) += jnt(2);
  theta(0) += jnt(0);
  theta(1) += jnt(1);
  theta(3) += jnt(3);
}


}  // namespace kinematics_lib
