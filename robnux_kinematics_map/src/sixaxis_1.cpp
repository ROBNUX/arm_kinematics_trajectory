#include "sixaxis_1.hpp"
#include <common/pose.hpp>
namespace kinematics_lib {

SixAxis_1::SixAxis_1(): serialArm(6)  {
}
  
SixAxis_1::SixAxis_1(const std::vector<double> &kine_para): serialArm(kine_para) {
}

// nominal IK
int SixAxis_1::CartToJnt(const Pose &pos, std::vector<double> *q) {
    std::ostringstream strs;
    if (!q) {
      strs.str("");
      strs << "input joint angle pointer is null in " << __FUNCTION__
                 << ", at line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return -ERR_INPUT_POINTER_NULL;
    }
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
    if (branch.size() < 3) {
       strs.str("");
       strs << "input pose has no enough branch information"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return -ERR_ROB_NO_BRANCH_INFO;
    }
    if (q->size() != DoF_) {
      q->resize(DoF_);
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

    //recall the first bit is about overhead , second bit is about elbow, last bit is about wrist flip
    // note: for Mitsubishi, branch[0] is denoted as  [right: 1, left: 0]
    //                       branch[1]  is denoted as [above: 1, below: 0]
    //                       branch[2] is denoted as  [Non-flip: 1, flip: 0]
    bool not_overhead = branch[0];
    bool righty = branch[1];
    bool wristState = branch[2];


    //Vector p = temp_f.p; //retrieve position data
    Rotation R(quat); // = temp_f.M; //retrieve orientation data
    Vec pl(0, 0, d_[5]);
    p = p - R * pl;

    // to add the effect of a3  we define a sudo a2 link with length of: 
	  double sudo_s4 =  hypot(a_[3], d_[3]);
    double angle_xyz;
    double wristToJoint2_length;
    double xyRadius2 = sqr(p.x()) + sqr(p.y()) - sqr(d_[1] + d_[2]);
            
    if (xyRadius2 < 0) {
        return -ERR_ID_IKPOS_UNREACHABLE;
    }

    double xyNorm = hypot(p.x(), p.y());
    if (not_overhead) {
		q->at(0) = atan2(p.y(), p.x()) - asin((d_[1] + d_[2]) / xyNorm);  
		double xyna1 = sqrt(xyRadius2) - a_[1];
		wristToJoint2_length = sqrt(sqr(xyna1) + sqr(p.z() - d_[0]));
		angle_xyz = atan2(xyna1, p.z() - d_[0]);

		if ((wristToJoint2_length > a_[2] + sudo_s4) || (wristToJoint2_length < sqrt(sqr(a_[2] - sudo_s4)))) {
            return -ERR_ID_IKPOS_UNREACHABLE;
        }
    } else {
		q->at(0) = atan2(p.y(), p.x()) + asin((d_[1] + d_[2]) / xyNorm);
        if (q->at(0)<0) {
            q->at(0) += M_PI;
        } else{
            q->at(0) -= M_PI;
        }
		double xya1 = sqrt(xyRadius2) + a_[1];
		wristToJoint2_length = hypot(xya1, p.z() - d_[0]);
		angle_xyz = atan2(xya1, p.z() - d_[0]);
		if ((wristToJoint2_length > a_[2] + sudo_s4) 
            || (wristToJoint2_length < sqrt(sqr(a_[2] - sudo_s4)))) {
            return -ERR_ID_IKPOS_UNREACHABLE; //-1;
        }
    }

	double a2s4_angle = acos((sqr(a_[2]) + sqr(sudo_s4) - sqr(wristToJoint2_length))
                        / (2 * a_[2] * sudo_s4)); // the angle between a2 and s4 (upper and lower arms \)

	double offset_angle = acos((sqr(a_[2]) + sqr(wristToJoint2_length) - sqr(sudo_s4))
                        / (2 * a_[2] * wristToJoint2_length)); 

    if (not_overhead) {
        if (!righty) {// if the configuration is blow
            q->at(1) = angle_xyz + offset_angle;
            q->at(2) = -M_PI + a2s4_angle;
        } else {// above
            q->at(1) = angle_xyz - offset_angle;
            q->at(2) = M_PI - a2s4_angle;
        } 
    } else { // overhead
        if (!righty) {// if the configuration is below
            q->at(1) = -angle_xyz + offset_angle;
            q->at(2) = -M_PI + a2s4_angle;
        } else { // above
            q->at(1) = -angle_xyz - offset_angle;
            q->at(2) = M_PI - a2s4_angle;
        }
    }
	double s4_offset = atan(a_[3]/d_[3]); //atan2(a3, s4); , a bug here, we need to make sure s4_offset is <90 degree  
    q->at(2) -= s4_offset;
    

    std::vector<int>  jointTurns;
    pos.getJointTurns(&jointTurns);
    for (size_t i=0; i < 3; i++) {
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
        if (i!=1 && i!=2) {  // because these two joints, we calculated already takein into account the joint initial offset
          q->at(i) -= theta_[i];
        }
    }

    q->at(1) -= (theta_[1] + M_PI / 2.0);
    q->at(2) -= (theta_[2] - M_PI / 2.0);


    q->at(3) = 0 - theta_[3];
    q->at(4) = 0 - theta_[4];
    q->at(5) = 0 - theta_[5];

    // FK with current *q
    Pose tmp;
    JntToCart(*q, &tmp);
    Rotation r=tmp.getRotation();
    Rotation wristRotation = r.Inverse() * R;
    double alpha, beta, gamma;
    wristRotation.GetEulerZYZ(&alpha, &beta, &gamma);

    if (wristState) {  // non-flip
       if (beta > 0) {
          q->at(3) = alpha;
          q->at(4) = beta;
          q->at(5) = gamma;
       } else {
          q->at(4) = -beta;
          // both can are right solutions , we use the one that our in math range             
          if (alpha > 0){ 
            q->at(3) = alpha - M_PI;
          } else {
            q->at(3) = alpha + M_PI;
          }    
          if (gamma > 0){       
            q->at(5) = gamma - M_PI;
          } else {   
            q->at(5) = gamma + M_PI;
          }       
       }
    } else {  // flip
       if (beta < 0) {
          q->at(3) = alpha;
          q->at(4) = beta;
          q->at(5) = gamma;
       } else {
          q->at(4) = -beta;
          // both can are right solutions , we use the one that our in math range             
          if (alpha > 0){ 
            q->at(3) = alpha - M_PI;
          } else {
            q->at(3) = alpha + M_PI;
          }    
          if (gamma > 0){       
            q->at(5) = gamma - M_PI;
          } else {   
            q->at(5) = gamma + M_PI;
          }       
       } 
    }
           

    // at this moment alpha - gamma = constant , the KDL  equalet to beta = 0 or pi
    if (wristRotation.UnitZ().z()> (1 - SIXDOF_WRISTSINGULAR)) {
            //std::cout << "beta" << beta << std::endl;
            return -ERR_ID_IKPOS_INFINITE_WRIST_SOLUTION;
    }

	double headDist = hypot(p.x(), p.y());
    if (headDist < SIXDOF_HEADDIST) {
            return -ERR_ID_IKPOS_INFINITE_SOLUTION; //-100; // infinite solutions 
    }

    for (size_t i=3; i < DoF_; i++) {
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
        if (i!=1 && i!=2) {  // because these two joints, we calculated already takein into account the joint initial offset
          q->at(i) -= theta_[i];
        }
    }
    return 0;
}


void  SixAxis_1::UpdateConfigTurn(const std::vector<double> & theta,
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
    // for scara, there is only 1 branch flag: elbow (up or down)
    branchFlags->resize(3, eBranchLeft);
    // 4 turn flags, actually only 3 turn flags (because joint 3 is prismatic
    jointTurns->resize(6, 0);
    std::vector<double> q_tmp = theta;
    // jnt 2, 3 converted to angles about vertical "home position"
    q_tmp[1] += M_PI / 2.0;
    q_tmp[2] -= M_PI / 2.0;

    //strs.str("");
    for (size_t i=0; i < DoF_; i++) {
        //strs << " joint " << i << " = " << q_tmp[i] << ", theta=" << theta_[i];
        jointTurns->at(i) = std::floor(q_tmp[i] / (2 * M_PI));
        double tmp_q = q_tmp[i] - jointTurns->at(i) * 2 * M_PI;
        // then make sure [-PI, PI]
        // to have 0 turn here
        if (tmp_q > M_PI) {
            jointTurns->at(i) += 1;
            tmp_q -= 2 * M_PI;
        }
        q_tmp[i] = tmp_q;

        if (i==2) { // for above and below (or elbow up and down)
            double phi_s4_a3 = atan(a_[3] / d_[3]);
            double qelbow = tmp_q + phi_s4_a3;  // -M_PI/2   (remove -M_PI/2 as qelbow is already about vertical home pose)
            if ((qelbow >= 0 && qelbow < M_PI) || (qelbow >= -2 * M_PI && qelbow< -M_PI)) {
                branchFlags->at(1) = 1; // above
            }
            if ((qelbow < 0 && qelbow>-M_PI) || (qelbow >= M_PI && qelbow < 2 * M_PI)) {
                branchFlags->at(1) = 0; // below
            }
        }

        // wrist state
        if (i==4) {
          if (tmp_q <= M_PI && tmp_q >= 0) {   // || (tmp_q >= -2 * M_PI && tmp_q < -M_PI)) {
              branchFlags->at(2) = 1;  // no flip
          } else {
              branchFlags->at(2) = 0;  // flip
          }
        }
    }
    //strs << std::endl;
    //LOG_INFO(strs);
    // q_tmp[1] -= theta_[1];
    // q_tmp[2] -= theta_[2];
    // we followed kuka manual for overhead calculation
    double xAtFrame1 = a_[1] + a_[2] * sin(q_tmp[1]) + 
                    d_[3] * sin(q_tmp[1] + q_tmp[2]) + a_[3] * cos(q_tmp[1] + q_tmp[2]);
    if (xAtFrame1 >= 0) {
       branchFlags->at(0) = 1; // no overhead , basic, or right(for mitsubishi)
    } else {
       branchFlags->at(0) = 0; //  overhead, or left
    }
}

bool SixAxis_1::PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
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
     size_t row_r = Jp_r.rows();
     size_t col_r = Jp_r.cols();
     if (row_t < 3 || col_t < 6 || row_r <3 || col_r < 6) {
        strs.str("");
        strs << " input Jacobian matrices have wrong dimension "
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
       return false;
     }
    
    Js_t->resize(3, 6);
    Js_r->resize(3, 6);
    for (size_t i=0; i < DoF_; i++) {
        // for theta[i] parameters
        (*Js_t).col(i) = Jp_t.col(5 * i + 2) * pitch_(i);
        (*Js_r).col(i) = Jp_r.col(5 * i + 2) * pitch_(i);
    }      
    return true;
}
   

bool SixAxis_1::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
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
    size_t row_r = Jp_r.rows();
    size_t col_r = Jp_r.cols();
    
    Js_t1->resize(row_t, col_t);
    *Js_t1 = Jp_t;
    Js_r1->resize(row_r, col_r);
    *Js_r1 = Jp_r;
    for (size_t i=0; i < DoF_; i++) {
        // for theta[i] parameters
        (*Js_t1).col(5 * i + 2) = Jp_t.col(5 * i + 2) * pitch_(i);
        (*Js_r1).col(5 * i + 2) = Jp_r.col(5 * i + 2) * pitch_(i);
    }      
    return true;
} 
   
// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm
double  SixAxis_1::PickCartErr(const Eigen::Vector3d &errT,
                           const Eigen::Vector3d &errR, 
                           Eigen::VectorXd *b,
                           const bool reduction) {
    std::ostringstream strs;
    double val = errT.norm() + errR.norm();
    if (!b) {
        strs.str("");
        strs << " input pointer is null"
                <<" in " << __FUNCTION__
                <<" at line " << __LINE__ << std::endl;
       LOG_ERROR(strs);
    }
    b->resize(6);
    b->block(0, 0, 3, 1) = errT;
    b->block(3, 0, 3, 1) = errR;
    return val;
}
   
   //! virtual function for updating actual DH parameters based upon joint feedback
   // orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k>
   // jnt angles def. depends on specific robot type
void SixAxis_1::UpdateDH(const std::vector<double> &orig_dh,
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
        new_dh->at(2 * DoF_ + i) = orig_dh[2 * DoF_ + i] + jnt(i) * pitch_(i);
    }
}

   //! virtual function for updating actual DH parameters based upon joint feedback
   // Note: alpha, a, theta, d, beta has their initial values, which will be updated
   // based upon jnt input
void SixAxis_1::UpdateDH(const Eigen::VectorXd &jnt,
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
    theta->at(i) += jnt(i) * pitch_(i);
  }
}

}

