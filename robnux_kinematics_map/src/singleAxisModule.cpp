#include "robnux_kinematics_map/singleAxisModule.hpp"
namespace kinematics_lib {

singleAxisModule::singleAxisModule(): serialArm(1)  {
}


int singleAxisModule::CartToJnt(const Pose& pos, Eigen::VectorXd& q) {
    std::ostringstream strs;
    if (!initialized_) {
      strs.str("");
      strs << "singleAxisModule geometric parameters are not initialized"
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
    return 0;
}


void  singleAxisModule::UpdateConfigTurn(const std::vector<double> & theta,
                              const std::vector<double> &d,
                              std::vector<int>  *branchFlags,
                              std::vector<int>  *jointTurns) const {
    std::ostringstream strs;
    if (!branchFlags || !jointTurns) {
        strs.str("");
        strs << "Input branchFlags and jointTurns are null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return;
    }
    // for singleAxisModule, there are 0 flags, so all set to 0 (eBranchLeft)
    branchFlags->resize(3, eBranchLeft);
    // DoF turn flags, shall all be 0, because it is prismatic
    jointTurns->resize(DoF_, 0);
}

bool singleAxisModule::PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r,
                                const bool reduction) {
     std::ostringstream strs;
     if (!Js_t || !Js_r) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
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
    
    Js_t->resize(3, DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*Js_t).col(i) = Jp_t.col(5 * i + 3)  * pitch_(i);
    }      
    return true;
}
   

bool singleAxisModule::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                   const Eigen::MatrixXd &Jp_r, 
                                   Eigen::MatrixXd *Js_t1, 
                                   Eigen::MatrixXd *Js_r1,
                                   const bool reduction) {
     std::ostringstream strs;
     if (!Js_t1 || !Js_r1) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    size_t row_t = Jp_t.rows();
    size_t col_t = Jp_t.cols();
    //size_t row_r = Jp_r.rows();
    //size_t col_r = Jp_r.cols();
    
    Js_t1->resize(row_t, col_t);
    *Js_t1 = Jp_t;
    for (size_t i=0; i < DoF_; i++) {
      (*Js_t1).col(5 * i + 3) = Jp_t.col(5 * i + 3) * pitch_(i);  // need pitch here
    }      
    //Js_r1->resize(1, col_r);
    //(*Js_r1).row(0) = Jp_r.row(2);
    return true;
} 
   
// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm
double  singleAxisModule::PickCartErr(const Eigen::Vector3d &errT,
                           const Eigen::Vector3d &errR, 
                           Eigen::VectorXd *b,
                           const bool reduction) {
    std::ostringstream strs;
    double val = errT.norm();
    if (!b) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    b->resize(3);
    *b = errT;
    return val;
}
   
   //! virtual function for updating actual DH parameters based upon joint feedback
   // orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k>
   // jnt angles def. depends on specific robot type
void singleAxisModule::UpdateDH(const std::vector<double> &orig_dh,
                     const Eigen::VectorXd &jnt,
                     std::vector<double> *new_dh) const {
    std::ostringstream strs;
    if (!new_dh) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    if (orig_dh.size() != 5 * DoF_ ||  jnt.size() < DoF_) {
        strs.str("");
        strs << " input parameters have wrong size, origin dh size= " << 
             orig_dh.size() << ", jnt size =" << jnt.size()   
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    *new_dh = orig_dh;
    for (size_t i=0; i < DoF_; i++) {
       new_dh->at(3 * DoF_ + i) = orig_dh[3 * DoF_ + i] + jnt(i) * pitch_(i);
    }
}

   //! virtual function for updating actual DH parameters based upon joint feedback
   // Note: alpha, a, theta, d, beta has their initial values, which will be updated
   // based upon jnt input
void singleAxisModule::UpdateDH(const Eigen::VectorXd &jnt,
              std::vector<double> *theta,
              std::vector<double> *d) const {
  std::ostringstream strs;
  if (!theta || !d) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }
  if (jnt.size() < DoF_ ||  theta->size() != DoF_ || d->size() != DoF_) {
       strs.str("");
       strs << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
  }

  for (size_t i=0; i < DoF_; i++) {
    d->at(i) += jnt(i) * pitch_(i);
  }
}


//! calibrate single module
double singleAxisModule::CalibrateSingleModule(const EigenDRef<Eigen::MatrixXd> &jnt_in,
                                      const EigenDRef<Eigen::MatrixXd> &cart) {
   std::ostringstream strs;
   size_t dimJnt = jnt_in.cols();
   size_t dimCart= cart.cols();

   size_t  rowsJnt = jnt_in.rows();
   size_t rowsCart = cart.rows();
   if (dimJnt != dimCart ||  dimJnt < 3 || dimCart < 3 || rowsJnt <= 0 || rowsCart < 3) {
      strs.str("");
      strs << " input parameters have wrong size "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_CALIB_MEASURE_DATA_WRONG_DIM; 
   }

   if (!initialized_) {
      strs.str("");
      strs << "Single Axis Module geometric parameters are not initialized"
                << " so can not do calibration, in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_ROB_PARAM_NOT_INITIALIZED; 
   }

   // now find out the indices of joint vector for which this measurement is performed
   size_t ind_jnt_meas = 0;
   if (rowsJnt == 1) {
        ind_jnt_meas = 0;
   } else {
      Eigen::VectorXd  dJnt = jnt_in.col(1) - jnt_in.col(0);
      double maxDist = 0;
      for (size_t i =0; i< dJnt.size(); i++) {
         if (fabs(dJnt(i)) > maxDist) {
            ind_jnt_meas = i;
            maxDist = fabs(dJnt(i));
         }
      }
   }


   Eigen::VectorXd  diffJnt(dimJnt - 1);
   Eigen::VectorXd  diffCart(dimCart - 1);

   std::vector<size_t> reverse_indices;
   std::vector<double> ratio_vec;
   for (size_t i=0; i< dimJnt-1; i++) {
      Eigen::VectorXd dJnt = jnt_in.col(i+1) - jnt_in.col(i);
      diffJnt(i) = dJnt(ind_jnt_meas);
      Eigen::VectorXd dcart =  cart.col(i+1) - cart.col(i);
      diffCart(i) = dcart.norm(); 
      if (i > 0) {
          if (diffJnt(i) * diffJnt(i-1) < -MAX_CALIB_BACKLASH_ERR) {
             reverse_indices.push_back(i);
          } else {
            ratio_vec.push_back(diffCart(i) / fabs(diffJnt(i)));
          }
      }
   }

   // first compute the pitch: the average ratio between jnt  and  module linear motion (m)
   double sumRatio = 0;
   for (size_t j=0; j < ratio_vec.size(); j++) {
      sumRatio += ratio_vec[j];
   }
   if (ratio_vec.size() > 0) {
     pitch_(0) = sumRatio / ratio_vec.size();
   }

   strs.str("");
   strs << "diffJnt = " <<  diffJnt.transpose() << std::endl;
   strs << "diffCart = " << diffCart.transpose() << std::endl;
   strs << "pitch_=" << pitch_  << ", ratio_vec size=" << ratio_vec.size() << std::endl;
   LOG_INFO(strs);

   // second compute the backlash
   strs.str("");
   double sumBacklash = 0;
   for (size_t j=0; j < reverse_indices.size(); j++) {
     size_t ind = reverse_indices[j];
     sumBacklash += fabs(fabs(diffCart(ind) / pitch_(0)) - fabs(diffJnt(ind)));
     strs << "ind =" << ind << ", diffCart(ind) =" << diffCart(ind) << ", diffJnt(ind)" << diffJnt(ind) << std::endl;
   }
   LOG_INFO(strs);
   if (reverse_indices.size() > 0) {
      backlash_(0) = sumBacklash / reverse_indices.size();
   } else {
      backlash_(0) = 0;
   }
   strs.str("");
   strs << "backlash_=" << backlash_  << ", reverse_indices size=" << reverse_indices.size() << std::endl;
   LOG_INFO(strs);
   
   isDHCalibrated_ = true;
   return 0;
}

}