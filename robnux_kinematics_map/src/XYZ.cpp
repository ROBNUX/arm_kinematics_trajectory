#include "robnux_kinematics_map/XYZ.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::XYZGantry, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

XYZGantry::XYZGantry(): serialArm(3) {
}

int XYZGantry::CartToJnt(const Pose &pos, Eigen::VectorXd& q) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << "XYZGantry geometric parameters are not initialized"
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
   
    if (q.size() != DoF_) {
      q.resize(DoF_);
    }
    q(0) = (p.z() - d_[0]) / pitch_(0);
    q(1) = (-p.y() - d_[1]) / pitch_(1);
    q(2) = (p.x() - d_[2]) / pitch_(2);
    return 0;
}


void  XYZGantry::UpdateConfigTurn(const Eigen::VectorXd& theta,
                               const Eigen::VectorXd& d,
                              std::vector<int>&  branchFlags,
                              std::vector<int>&  jointTurns) const {
    std::ostringstream strs;
    // for XYZGantry, there are 0 flags, so all set to 0 (eBranchLeft)
    branchFlags.resize(3, eBranchLeft);
    // DoF turn flags, shall all be 0, because it is prismatic
    jointTurns.resize(DoF_, 0);
}

bool XYZGantry::PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                                const Eigen::MatrixXd& Jp_r,
                                Eigen::MatrixXd& Js_t,
                                Eigen::MatrixXd& Js_r,
                                bool reduction) {
    std::ostringstream strs;
    size_t row_t = Jp_t.rows();
    size_t col_t = Jp_t.cols();

    if (row_t < 3 || col_t < DoF_) {    //|| row_r <3 || col_r < DoF_) {
        strs.str("");
        strs << " input Jacobian matrices have wrong dimension "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    Js_t.resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      Js_t.col(i) = Jp_t.col(4 * i + 3) * pitch_(i);  // need pitch here
    }      
    return true;
}

double  XYZGantry::PickCartErr(const Eigen::Vector3d& errT,
                           const Eigen::Vector3d& errR, 
                           Eigen::VectorXd& b,
                           bool reduction) {
    std::ostringstream strs;
    double val = errT.norm();
    b.resize(3);
    b.block(0, 0, 3, 1) = errT;
    return val;
}

void XYZGantry::UpdateDH(const Eigen::VectorXd& orig_dh,
                     const Eigen::VectorXd& jnt,
                     Eigen::VectorXd& new_dh) const {
    std::ostringstream strs;
    if (orig_dh.size() != 4 * DoF_ ||  jnt.size() < DoF_) {
        strs.str("");
        strs << " input parameters have wrong size, origin dh size= " << 
             orig_dh.size() << ", jnt size =" << jnt.size()   
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    new_dh = orig_dh;
    for (size_t i=0; i < DoF_; i++) {
       new_dh(3 * DoF_ + i) = orig_dh(3 * DoF_ + i) + jnt(i) * pitch_(i);
    }
}

void XYZGantry::UpdateDH(const Eigen::VectorXd& jnt,
              Eigen::VectorXd& theta,
              Eigen::VectorXd& d) const {
  std::ostringstream strs;

  if (jnt.size() < DoF_ ||  theta.size() != DoF_ || d.size() != DoF_) {
       strs.str("");
       strs << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }

  for (size_t i=0; i < DoF_; i++) {
    d(i) += jnt(i) * pitch_(i);
  }
}

}