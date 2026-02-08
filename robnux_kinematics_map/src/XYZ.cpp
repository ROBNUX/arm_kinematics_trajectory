#include "XYZ.hpp"
#include <common/pose.hpp>
namespace kinematics_lib {

XYZGantry::XYZGantry(): serialArm(3)  {
}
  
XYZGantry::XYZGantry(const std::vector<double> &kine_para): serialArm(kine_para) {
}

// nominal IK
int XYZGantry::CartToJnt(const Pose &pos, std::vector<double> *q) {
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
   
    if (q->size() != DoF_) {
      q->resize(DoF_);
    }
    q->at(0) = (p.z() - d_[0]) / pitch_(0);
    q->at(1) = (-p.y() - d_[1]) / pitch_(1);
    q->at(2) = (p.x() - d_[2]) / pitch_(2);
    return 0;
}


void  XYZGantry::UpdateConfigTurn(const std::vector<double> & theta,
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
    // for XYZGantry, there are 0 flags, so all set to 0 (eBranchLeft)
    branchFlags->resize(3, eBranchLeft);
    // DoF turn flags, shall all be 0, because it is prismatic
    jointTurns->resize(DoF_, 0);
}

bool XYZGantry::PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
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
      (*Js_t).col(i) = Jp_t.col(5 * i + 3) * pitch_(i);  // need pitch here
    }      
    return true;
}
   

bool XYZGantry::PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
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
    
    Js_t1->resize(row_t, col_t);
    *Js_t1 = Jp_t;
    for (size_t i=0; i < DoF_; i++) {
      (*Js_t1).col(5 * i + 3) = Jp_t.col(5 * i + 3) * pitch_(i);  // need pitch here
    }
    return true;
} 
   
// given trans and euler angle error, pick a sub error vector matching
// robot model, and return the aboslute error norm
double  XYZGantry::PickCartErr(const Eigen::Vector3d &errT,
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
    b->block(0, 0, 3, 1) = errT;
    return val;
}
   
   //! virtual function for updating actual DH parameters based upon joint feedback
   // orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k>
   // jnt angles def. depends on specific robot type
void XYZGantry::UpdateDH(const std::vector<double> &orig_dh,
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
void XYZGantry::UpdateDH(const Eigen::VectorXd &jnt,
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

int XYZGantry::ErrCompensateBase(const Eigen::MatrixXd &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 Eigen::VectorXd *tcp_ur_uncal,
                                 Eigen::VectorXd *tcp_ur_cal
                                 ) {
  std::ostringstream strs;
  size_t numJnts = jnt_base_measures.cols();
  // here orig_base is the workpiece frame, should be reachable by IK
  if (jnt_base_measures.rows() < DoF_ || numJnts < 1) {
    strs.str("");
    strs << GetName() << ":" << "The input vector has wrong dimension in function "
          << __FUNCTION__ << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
  }
  if (!tcp_ur_uncal || !tcp_ur_cal) {
    strs.str("");
    strs << GetName() << ":"  << "input tcp_ur_cal is null in function " << __FUNCTION__ 
          << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }


  Frame userBase; // default base
  Vec t(orig_tool.segment(0, 3)); //here tool only contains the trans. 
  Quaternion q(orig_tool(3), orig_tool(4), orig_tool(5), orig_tool(6));
  Frame userTool;
  userTool.setTranslation(t);  // user Tool
  userTool.setQuaternion(q);
  // 1, 3, 8 point method
  std::vector<Pose> ps(numJnts);
  std::vector<refPose> rps(numJnts);

  // first compute uncalibrated base frame
  SetUsingCalibratedModel(false);
  for (size_t i=0; i<numJnts; i++) {
    std::vector<double> jnt(DoF_, 0);
    EigenVec2StdVec(jnt_base_measures.col(i), &jnt);
    int ret = JntToCart(jnt, &ps[i]);
    if (ret < 0) {
      strs.str("");
      strs << GetName() << ":" << "FK error, code  " << ret
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
    strs << GetName() << ":" << "Canonical model: pt" << i  <<
          ", translation=" << rps[i].getTranslation().ToString() << std::endl;
    LOG_INFO(strs);
  }
  Rotation newr;
  tcp_ur_uncal->resize(7);
  if (numJnts == 1) {
    Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
    (*tcp_ur_uncal)(0) = newCenter.x();
    (*tcp_ur_uncal)(1) = newCenter.y();
    (*tcp_ur_uncal)(2) = newCenter.z();

    (*tcp_ur_uncal)(3) = 1;
    (*tcp_ur_uncal)(4) = 0;
    (*tcp_ur_uncal)(5) = 0;
    (*tcp_ur_uncal)(6) = 0;
  } else if (numJnts == 3) {
    Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
    (*tcp_ur_uncal)(0) = newCenter.x();
    (*tcp_ur_uncal)(1) = newCenter.y();
    (*tcp_ur_uncal)(2) = newCenter.z();
  
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
    (*tcp_ur_uncal)(3) = q.w();
    (*tcp_ur_uncal)(4) = q.x();
    (*tcp_ur_uncal)(5) = q.y();
    (*tcp_ur_uncal)(6) = q.z();
    strs.str("");
    strs << GetName() << ":" << "3pt base frame computation: uncalibrated frame, trans=" << newCenter.ToString() << ", rot=" << newr.ToString() << std::endl;
    LOG_INFO(strs);
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
    (*tcp_ur_uncal)(3) = q.w();
    (*tcp_ur_uncal)(4) = q.x();
    (*tcp_ur_uncal)(5) = q.y();
    (*tcp_ur_uncal)(6) = q.z();
      
    
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
    (*tcp_ur_uncal)(0) = newCenter.x();
    (*tcp_ur_uncal)(1) = newCenter.y();
    (*tcp_ur_uncal)(2) = newCenter.z();
  } else {
      strs.str("");
      strs << GetName() << ":" << "num of input joints in  " << __FUNCTION__ 
          << " at line " << __LINE__ << " is out of scope" << std::endl;
      LOG_ERROR(strs);
      return -1;
  }
    

  strs.str("");
  strs << GetName() << ":" << " work frame Euler angle = " << newr.ToString() << std::endl;
  LOG_INFO(strs);

  if (!isDHCalibrated_) {
    strs.str("");
    strs << GetName() << ":" << " robot is not calibrated, "
        << "can not do error compensation in calibrated base, but use the original base"
        << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_INFO(strs);
    *tcp_ur_cal = *tcp_ur_uncal;
    return 0;
  }
    
    
  // first compute calibrated base frame
  SetUsingCalibratedModel(true);
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
    strs.str("");
    strs << GetName() << ":" << "Calibrated model: pt " << i << 
          ", translation=" << rps[i].getTranslation().ToString() << std::endl;
    LOG_INFO(strs);
  }

  tcp_ur_cal->resize(7);
  if (numJnts == 1) {
    Vec newCenter = rps[0].getTranslation(); // rps[0] is the origin of the datus reference frame of workpiece
    (*tcp_ur_cal)(0) = newCenter.x();
    (*tcp_ur_cal)(1) = newCenter.y();
    (*tcp_ur_cal)(2) = newCenter.z();

    (*tcp_ur_cal)(3) = 1;
    (*tcp_ur_cal)(4) = 0;
    (*tcp_ur_cal)(5) = 0;
    (*tcp_ur_cal)(6) = 0;
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

    (*tcp_ur_cal)(0) = newCenter.x();
    (*tcp_ur_cal)(1) = newCenter.y();
    (*tcp_ur_cal)(2) = newCenter.z();
    Quaternion q;
    newr.GetQuaternion(&q);
    (*tcp_ur_cal)(3) = q.w();
    (*tcp_ur_cal)(4) = q.x();
    (*tcp_ur_cal)(5) = q.y();
    (*tcp_ur_cal)(6) = q.z();
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
      (*tcp_ur_cal)(3) = q.w();
      (*tcp_ur_cal)(4) = q.x();
      (*tcp_ur_cal)(5) = q.y();
      (*tcp_ur_cal)(6) = q.z();
    
      
    
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
    (*tcp_ur_cal)(0) = newCenter.x();
    (*tcp_ur_cal)(1) = newCenter.y();
    (*tcp_ur_cal)(2) = newCenter.z();
  } else {
      strs.str("");
      strs << GetName() << ":" << "num of input joints in  " << __FUNCTION__ 
          << " at line " << __LINE__ << " is out of scope" << std::endl;
      LOG_ERROR(strs);
      return -1;
  }
  strs.str("");
  strs << GetName() << ":" << " work frame Euler angle = " << newr.ToString()  << std::endl;
  LOG_INFO(strs);
  return 0;
}
/*
void XYZGantry::ForwardQDot(const Eigen::VectorXd &qdot, Eigen::VectorXd *aqdot) const  {
    std::ostringstream strs;
    if (!aqdot) {
      strs.str("");
      strs << "input parameter is null in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    if (!initialized_) {
      strs.str("");
      strs << "XYZGantry geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    aqdot->resize(DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*aqdot)(i) = qdot(i) * pitch_(i);
    }
}

void XYZGantry::BackwardQDot(const Eigen::VectorXd &aqdot, Eigen::VectorXd *qdot) const  {
    std::ostringstream strs;
    if (!qdot) {
      strs.str("");
      strs << "input parameter is null in function "
                  << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }
    if (!initialized_) {
      strs.str("");
      strs << "XYZGantry geometric parameters are not initialized"
                << " in function "
                << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return; 
    }
    qdot->resize(DoF_);
    for (size_t i=0; i < DoF_; i++) {
      (*qdot)(i) = aqdot(i) / pitch_(i);
    }
}
*/
}