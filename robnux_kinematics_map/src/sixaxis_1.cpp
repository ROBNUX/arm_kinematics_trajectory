#include "robnux_kinematics_map/sixaxis_1.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::SixAxis_1, kinematics_lib::BaseKinematicMap)
namespace kinematics_lib {

SixAxis_1::SixAxis_1(): serialArm(6)  {
}

int SixAxis_1::CartToJnt(const Pose& pos, Eigen::VectorXd& q) {
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
    if (branch.size() < 3) {
       strs.str("");
       strs << "input pose has no enough branch information"
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
		q(0) = atan2(p.y(), p.x()) - asin((d_[1] + d_[2]) / xyNorm);  
		double xyna1 = sqrt(xyRadius2) - a_[1];
		wristToJoint2_length = sqrt(sqr(xyna1) + sqr(p.z() - d_[0]));
		angle_xyz = atan2(xyna1, p.z() - d_[0]);

		if ((wristToJoint2_length > a_[2] + sudo_s4) || (wristToJoint2_length < sqrt(sqr(a_[2] - sudo_s4)))) {
            return -ERR_ID_IKPOS_UNREACHABLE;
        }
    } else {
		q(0) = atan2(p.y(), p.x()) + asin((d_[1] + d_[2]) / xyNorm);
        if (q(0)<0) {
            q(0) += M_PI;
        } else{
            q(0) -= M_PI;
        }
		double xya1 = sqrt(xyRadius2) + a_[1];
		wristToJoint2_length = hypot(xya1, p.z() - d_[0]);
		angle_xyz = atan2(xya1, p.z() - d_[0]);
		if ((wristToJoint2_length > a_[2] + sudo_s4) 
            || (wristToJoint2_length < sqrt(sqr(a_[2] - sudo_s4)))) {
            return -ERR_ID_IKPOS_UNREACHABLE; //-1;
        }
    }

	double offset_angle = acos((sqr(a_[2]) + sqr(wristToJoint2_length) - sqr(sudo_s4))
                        / (2 * a_[2] * wristToJoint2_length));

    double q1_sign = not_overhead ? 1.0 : -1.0;
    if (!righty) {// if the configuration is below
        q(1) = q1_sign * angle_xyz + offset_angle;
    } else {// above
        q(1) = q1_sign * angle_xyz - offset_angle;
    }

    // q(2) (elbow): from-scratch, chirality-general solve, replacing the old
    // "2*s4_offset +- a2s4_angle" law-of-cosines formula. That formula (and
    // the s4_offset=atan(a_[3]/d_[3]) it was built from) implicitly assumed
    // alpha_[3]<0 -- it produced a *plausible-looking* q(2) with a genuine
    // FK/IK round trip for that sign (see project_sixaxis_ik_bugs memory),
    // but silently returned wrong answers (not errors) for the alpha_[3]>0
    // convention real robots use too (e.g. the Mitsubishi RV-2FR-D:
    // ~0.5m position error on ~90% of poses, not a crash, which is what
    // made it easy to miss until tested against a second real DH set).
    //
    // Frame1 (waist * shoulder, using the already-solved q(0)/q(1) "raw"
    // values -- q(1) hasn't had its -(theta_[1]+PI/2) offset subtracted
    // yet at this point) locates the elbow: elbow = T1 * (a_[2],0,0).
    // Expressing the elbow->wrist vector in T1's own axes and comparing its
    // direction against the (a_[3],d_[3]) offset's own local direction
    // (-sin(alpha_[3])*d_[3], a_[3]) gives q(2)'s DH theta directly via
    // atan2 -- exact for *any* alpha_[3] (not just +-PI/2), no separate
    // sign case needed. Derived from the Craig-DH position equations and
    // verified (100% FK/IK round trip, both alpha_[3] signs, all 8 branch
    // codes) in verify_sixaxis_ik_math.py before porting here.
    Frame T1 = Frame::DH_Craig1989(0, 0, d_[0], q(0))
             * Frame::DH_Craig1989(a_[1], alpha_[1], 0, q(1) - M_PI / 2.0);
    Vec elbow = T1.getRotation() * Vec(a_[2], 0, 0) + T1.getTranslation();
    Vec elbowToWrist = T1.getRotation().Inverse() * (p - elbow);
    double refAngle = atan2(-sin(alpha_[3]) * d_[3], a_[3]);
    double q2_dh = atan2(elbowToWrist.y(), elbowToWrist.x()) - refAngle - M_PI;
    q(2) = q2_dh + M_PI / 2.0;


    std::vector<int>  jointTurns;
    pos.getJointTurns(&jointTurns);
    for (size_t i=0; i < 3; i++) {
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
        if (i!=1 && i!=2) {  // because these two joints, we calculated already takein into account the joint initial offset
          q(i) -= theta_[i];
        }
    }

    q(1) -= (theta_[1] + M_PI / 2.0);
    q(2) -= (theta_[2] - M_PI / 2.0);


    q(3) = 0 - theta_[3];
    q(4) = 0 - theta_[4];
    q(5) = 0 - theta_[5];

    // FK with current *q
    Pose tmp;
    JntToCart(q, tmp);
    Rotation r=tmp.getRotation();
    Rotation wristRotation = r.Inverse() * R;

    // A spherical wrist built with alpha_[3..5] = [s,-s,s] is the mirror
    // image (through the plane perpendicular to the local Z axis) of one
    // built with alpha_[3..5] = [-s,s,-s], for the *same* joint angles --
    // verified: R_mirror(q3,q4,q5) = M * R_default(q3,q4,q5) * M with
    // M = diag(1,1,-1). The GetEulerZYZ-based extraction below was derived
    // for alpha_[4]>0 (this file's original convention); for alpha_[4]<0
    // geometries (e.g. Mitsubishi RV-2FR-D) reflect wristRotation by M
    // first -- since M*M=I, this maps wristRotation = R_mirror(q3,q4,q5)
    // onto M*wristRotation*M = R_default(q3,q4,q5), the exact equation the
    // unchanged formula below already solves correctly.
    if (alpha_[4] < 0) {
      wristRotation = Rotation(
           wristRotation(0, 0),  wristRotation(0, 1), -wristRotation(0, 2),
           wristRotation(1, 0),  wristRotation(1, 1), -wristRotation(1, 2),
          -wristRotation(2, 0), -wristRotation(2, 1),  wristRotation(2, 2));
    }
    double alpha, beta, gamma;
    wristRotation.GetEulerZYZ(&alpha, &beta, &gamma);

    // GetEulerZYZ always returns beta in [0, pi] -- it cannot itself tell us
    // which of the two valid (q3,q4,q5)/(q3+pi,-q4,q5+pi) wrist solutions the
    // caller wants, so branching on the sign of beta here (as the old code
    // did) is dead logic that made wristState select the wrong solution (or
    // no solution at all). wristState alone must pick the branch: non-flip
    // takes q4=+beta (and renormalizes alpha/gamma by +-pi to compensate),
    // flip takes q4=-beta (alpha/gamma used as returned).
    if (wristState) {  // non-flip: q4 = +beta
       q(4) = beta;
       if (alpha > 0){
         q(3) = alpha - M_PI;
       } else {
         q(3) = alpha + M_PI;
       }
       if (gamma > 0){
         q(5) = gamma - M_PI;
       } else {
         q(5) = gamma + M_PI;
       }
    } else {  // flip: q4 = -beta
       q(3) = alpha;
       q(4) = -beta;
       q(5) = gamma;
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
         double turn = std::floor(q(i) / (2 * M_PI));
         double tmp_q = q(i) - turn * 2 * M_PI;
         // then make sure [-PI, PI]
         // to have 0 turn here
        if (tmp_q > M_PI) {
          turn += 1;
        }
        // q->at(3) -= jointTurns3 * 2 * PI;
        q(i) += (jointTurns[i] - turn) * 2 * M_PI;  // recall only joint 3 might moving more than 1 turn
        if (i!=1 && i!=2) {  // because these two joints, we calculated already takein into account the joint initial offset
          q(i) -= theta_[i];
        }
    }
    return 0;
}


void  SixAxis_1::UpdateConfigTurn(const Eigen::VectorXd& theta,
                              const Eigen::VectorXd& d,
                              std::vector<int>& branchFlags,
                              std::vector<int>& jointTurns) const {
    std::ostringstream strs;
    // for scara, there is only 1 branch flag: elbow (up or down)
    branchFlags.resize(3, eBranchLeft);
    // 4 turn flags, actually only 3 turn flags (because joint 3 is prismatic
    jointTurns.resize(6, 0);
    Eigen::VectorXd q_tmp = theta;
    // jnt 2, 3 converted to angles about vertical "home position"
    q_tmp(1) += M_PI / 2.0;
    q_tmp(2) -= M_PI / 2.0;

    //strs.str("");
    for (size_t i=0; i < DoF_; i++) {
        //strs << " joint " << i << " = " << q_tmp[i] << ", theta=" << theta_[i];
        jointTurns[i] = std::floor(q_tmp(i) / (2 * M_PI));
        double tmp_q = q_tmp(i) - jointTurns[i] * 2 * M_PI;
        // then make sure [-PI, PI]
        // to have 0 turn here
        if (tmp_q > M_PI) {
            jointTurns[i] += 1;
            tmp_q -= 2 * M_PI;
        }
        q_tmp(i) = tmp_q;

        if (i==2) { // for above and below (or elbow up and down)
            double phi_s4_a3 = atan(a_[3] / d_[3]);
            double qelbow = tmp_q + phi_s4_a3;  // -M_PI/2   (remove -M_PI/2 as qelbow is already about vertical home pose)
            // CartToJnt's law-of-cosines elbow solution is symmetric about
            // 2*phi_s4_a3 (q(2) = 2*s4_offset +- a2s4_angle), not about 0/PI,
            // so the above/below split must be taken relative to that same
            // reference angle -- splitting at 0/PI (as before) disagreed with
            // CartToJnt for roughly half of all poses.
            double elbowShifted = qelbow - 2 * phi_s4_a3;
            while (elbowShifted > M_PI) {
                elbowShifted -= 2 * M_PI;
            }
            while (elbowShifted <= -M_PI) {
                elbowShifted += 2 * M_PI;
            }
            if (elbowShifted <= 0) {
                branchFlags[1] = 1; // above
            } else {
                branchFlags[1] = 0; // below
            }
        }

        // wrist state
        if (i==4) {
          if (tmp_q <= M_PI && tmp_q >= 0) {   // || (tmp_q >= -2 * M_PI && tmp_q < -M_PI)) {
              branchFlags[2] = 1;  // no flip
          } else {
              branchFlags[2] = 0;  // flip
          }
        }
    }
    //strs << std::endl;
    //LOG_INFO(strs);
    // q_tmp[1] -= theta_[1];
    // q_tmp[2] -= theta_[2];
    // we followed kuka manual for overhead calculation
    // NOTE: this must match the radial-reach sign convention used by CartToJnt
    // (q(0) = atan2(y,x) directly when this is >=0, i.e. not_overhead), which
    // is R = a1 + a2*cos(t1) + a3*cos(t1+t2) - d3*sin(t1+t2). The d_[3] term
    // was previously added instead of subtracted, which flipped the
    // overhead/not_overhead classification for any pose where the d3 term
    // dominates (e.g. a wrist-up pose can be misclassified as not_overhead).
    double xAtFrame1 = a_[1] + a_[2] * sin(q_tmp(1)) -
                    d_[3] * sin(q_tmp(1) + q_tmp(2)) + a_[3] * cos(q_tmp(1) + q_tmp(2));
    if (xAtFrame1 >= 0) {
       branchFlags[0] = 1; // no overhead , basic, or right(for mitsubishi)
    } else {
       branchFlags[0] = 0; //  overhead, or left
    }
}

bool SixAxis_1::PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                                const Eigen::MatrixXd& Jp_r,
                                Eigen::MatrixXd& Js_t,
                                Eigen::MatrixXd& Js_r,
                                bool world_jac) {
     std::ostringstream strs;
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
    
    Js_t.resize(3, 6);
    Js_r.resize(3, 6);
    for (size_t i=0; i < DoF_; i++) {
        // for theta[i] parameters
        Js_t.col(i) = Jp_t.col(4 * i + 2) * pitch_(i);
        Js_r.col(i) = Jp_r.col(4 * i + 2) * pitch_(i);
    }      
    return true;
}

double  SixAxis_1::PickCartErr(const Eigen::Vector3d& errT,
                           const Eigen::Vector3d& errR, 
                           Eigen::VectorXd& b,
                           bool reduction) {
    double val = errT.norm() + errR.norm();
    b.resize(6);
    b.block(0, 0, 3, 1) = errT;
    b.block(3, 0, 3, 1) = errR;
    return val;
}

void SixAxis_1::UpdateDH(const Eigen::VectorXd& orig_dh,
                     const Eigen::VectorXd &jnt,
                     Eigen::VectorXd&new_dh) const {
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
        new_dh(2 * DoF_ + i) = orig_dh(2 * DoF_ + i) + jnt(i) * pitch_(i);
    }
}

void SixAxis_1::UpdateDH(const Eigen::VectorXd& jnt,
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
    theta(i) += jnt(i) * pitch_(i);
  }
}

}

