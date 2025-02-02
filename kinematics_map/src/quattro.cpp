#include "kinematics_map/quattro.hpp"
// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::Quattro,
                       kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

Quattro::Quattro()
    : BaseKinematicMap(4, 3),
      branchFlags_(std::vector<int>(3, 0)),
      jointTurns_(std::vector<int>(4, 0)) {
  // always bottom branch and 0 turns
  // DoF_ = 3;
  // A_DoF_ = 4;
  branchFlags_[0] = 1;
  tipPoints.resize(DoF_);
  jnt_names_.resize(21);
  jnt_names_[0] = "JOINT_1_ACT";
  jnt_names_[1] = "JOINT_2_ACT";
  jnt_names_[2] = "JOINT_3_ACT";
  jnt_names_[3] = "JOINT_4_ACT";
  jnt_names_[4] = "uleg1_con";
  jnt_names_[5] = "con_lleg_1";
  jnt_names_[6] = "con2_lleg_1";
  jnt_names_[7] = "uleg2_con";
  jnt_names_[8] = "con_lleg_2";
  jnt_names_[9] = "con2_lleg_2";
  jnt_names_[10] = "uleg3_con";
  jnt_names_[11] = "con_lleg_3";
  jnt_names_[12] = "con2_lleg_3";
  jnt_names_[13] = "uleg4_con";
  jnt_names_[14] = "con_lleg_4";
  jnt_names_[15] = "con2_lleg_4";
  jnt_names_[16] = "con43_lleg_4";
  jnt_names_[17] = "con33_lleg_3";
  jnt_names_[18] = "con23_lleg_2";
  jnt_names_[19] = "con13_lleg_1";
  jnt_names_[20] = "lleg1_base";
}

Quattro::Quattro(const Eigen::VectorXd &parameters)
    : BaseKinematicMap(4, 3),
      // here branchFlags_ denotes the branch flag of FK of parallel robot
      branchFlags_(std::vector<int>(3, 0)),
      jointTurns_(std::vector<int>(4, 0)) {
  // DoF_ = 3;
  // A_DoF_ = 4;
  branchFlags_[0] = 1;
  tipPoints.resize(DoF_);
  SetGeometry(parameters);
  jnt_names_.resize(21);
  jnt_names_[0] = "JOINT_1_ACT";
  jnt_names_[1] = "JOINT_2_ACT";
  jnt_names_[2] = "JOINT_3_ACT";
  jnt_names_[3] = "JOINT_4_ACT";
  jnt_names_[4] = "uleg1_con";
  jnt_names_[5] = "con_lleg_1";
  jnt_names_[6] = "con2_lleg_1";
  jnt_names_[7] = "uleg2_con";
  jnt_names_[8] = "con_lleg_2";
  jnt_names_[9] = "con2_lleg_2";
  jnt_names_[10] = "uleg3_con";
  jnt_names_[11] = "con_lleg_3";
  jnt_names_[12] = "con2_lleg_3";
  jnt_names_[13] = "uleg4_con";
  jnt_names_[14] = "con_lleg_4";
  jnt_names_[15] = "con2_lleg_4";
  jnt_names_[16] = "con43_lleg_4";
  jnt_names_[17] = "con33_lleg_3";
  jnt_names_[18] = "con23_lleg_2";
  jnt_names_[19] = "con13_lleg_1";
  jnt_names_[20] = "lleg1_base";
}

void Quattro::SetGeometry(const Eigen::VectorXd &parameters) {
  if (parameters.size() >= 7) {
    R1_ = parameters(0);
    alpha_ = parameters(1);
    b1_ = parameters(2);
    c1_ = parameters(3);
    d1_ = parameters(4);
    h_ = parameters(5);
    r1_ = parameters(6);
    m_ = parameters(7);
    a_b1_ = sqrt(b1_ * b1_ + c1_ * c1_);
    delta1_ = atan2(c1_, b1_);
    diff_radius_ = R1_ - r1_ - m_ / sqrt(2);
    char_length_ = b1_ + d1_;
    initialized_ = true;
  }
}

int Quattro::JntToCart(const Eigen::VectorXd &q, Pose *p) {
  std::ostringstream strs;
  if (!p) {
    strs << ERR_DESCRIPTION[ERR_INPUT_POINTER_NULL] << ", in function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs << ERR_DESCRIPTION[ERR_ROB_PARAM_NOT_INITIALIZED]  //"Quattro geometric
                                                            //parameters are not
                                                            //initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  if (q.size() < DoF_) {
    strs << ERR_DESCRIPTION[ERR_INPUT_PARA_WRONG_DIM]  //"input joint vector has
                                                       //wrong dimension in
                                                       //function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  // std::vector<Vec> tipPoints;
  for (size_t i = 0; i < DoF_; i++) {
    double proj_radius = diff_radius_ + a_b1_ * cos(q(i) + delta1_);
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    tipPoints[i] = Vec(proj_radius * cos(angle), proj_radius * sin(angle),
                       -a_b1_ * sin(q(i) + delta1_));
  }
  // now compute the intersections of three spheres centered at tipPoints[i]
  // with radius c1_
  // step 1:  spheres with center at tipPoints[0] and tipPoints[1]
  // intersect at a circle C1, now compute the origin O1 and normal n1 of C1
  Vec O1 = (tipPoints[0] + tipPoints[1]) / 2.0;
  Vec n1 = tipPoints[1] - tipPoints[0];
  // norm of n1
  double norm_n1 = n1.Norm();
  // normalize n1
  if (norm_n1 < K_EPSILON) {
    strs
        << ERR_DESCRIPTION[ERR_QUATTRO_FK_OVERLAP]  //"Tip points of two large
                                                    //arms overlaps, no forward"
        << ", in function " << __FUNCTION__ << ", at line " << __LINE__
        << std::endl;
    LOG_ERROR(strs);
    return -ERR_QUATTRO_FK_OVERLAP;
  }
  n1 = n1 / norm_n1;
  // radius of C1
  double rad_c1 = sqrt(d1_ * d1_ - norm_n1 * norm_n1 / 4.0);
  // now compute the distance from  tipPoints[3] to the plane of C1
  Vec O1P3 = tipPoints[2] - O1;
  // projection of O1P3 along n1
  double d0 = n1.dot(O1P3);
  // radius of the circle C3: the intersection of sphere centered P3
  // intersects with plane containing C1
  double rad_c3 = sqrt(d1_ * d1_ - d0 * d0);
  // projection of O1P3 to plane of C1
  Vec O1O3 = O1P3 - d0 * n1;
  double norm_O1O3 = O1O3.Norm();
  if (norm_O1O3 < K_EPSILON) {
    strs << ERR_DESCRIPTION[ERR_QUATTRO_FK_3PT_LINE]  //"Tip points of three
                                                      //large arms in the same
                                                      //line,"
         << ", in function " << __FUNCTION__ << ", at line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_QUATTRO_FK_3PT_LINE;
  }
  // x axis of plane c1 is normalized vector of O1O3
  Vec x_c3 = O1O3 / norm_O1O3;
  Vec y_c3 = n1 * x_c3;  // right hand rule
  // now compute the intersection c1 and c3
  if (norm_O1O3 > rad_c1 + rad_c3) {
    strs << ERR_DESCRIPTION[ERR_QUATTRO_FK_CIRC_NO_INTERS]  //"two circles c1,
                                                            //c3 has no
                                                            //intersections,"
         << ", in function " << __FUNCTION__ << ", at line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_QUATTRO_FK_CIRC_NO_INTERS;
  }
  // always use positive branch, the solution could be -alpha too.
  double alpha =
      acos((norm_O1O3 * norm_O1O3 + rad_c1 * rad_c1 - rad_c3 * rad_c3) /
           (2 * norm_O1O3 * rad_c1));
  Vec PP;
  if (branchFlags_[0] == (int)eBranchLeft) {  // top branch
    PP = O1 + rad_c1 * (cos(alpha) * x_c3 + sin(alpha) * y_c3);
  } else {  // bottom branch
    PP = O1 + rad_c1 * (cos(alpha) * x_c3 - sin(alpha) * y_c3);
  }
  p->setTranslation(PP);
  p->setBranchFlags(branchFlags_);
  p->setJointTurns(jointTurns_);
  return 0;
}

int Quattro::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                       Pose *p, Twist *v) {
  std::ostringstream strs;
  if (!p || !v) {
    strs << ERR_DESCRIPTION[ERR_INPUT_POINTER_NULL] << ", in function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs << ERR_DESCRIPTION[ERR_ROB_PARAM_NOT_INITIALIZED]  //"Quattro geometric
                                                            //parameters are not
                                                            //initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  int ret = this->JntToCart(q, p);
  if (ret < 0) {
    return ret;
  }
  if (qdot.size() < DoF_) {
    strs << ERR_DESCRIPTION[ERR_INPUT_PARA_WRONG_DIM]  //"input joint vector has
                                                       //wrong dimension in
                                                       //function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -ERR_INPUT_PARA_WRONG_DIM;
  }
  Vec PP = p->getTranslation();
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // Eigen::VectorXd digJ1(A_DoF_);
  // we try to avoid use eigen library here
  std::vector<double> MTM(9, 0);
  Vec left;
  for (size_t i = 0; i < qdot.size(); i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos(q(i) + delta1_);
    double sq = sin(q(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(diff_radius_ * ca, diff_radius_ * sa, 0) + V2i;
    Vec V3i = PP - tipi;
    double diagJ1i = (Ax1i * V2i).dot(V3i) * qdot(i);
    /*
     *  M[0] M[1] M[2]
     *  M[3] M[4] M[5]
     *  M[6] M[7] M[8]
     */
    MTM[0] += V3i.x() * V3i.x();
    MTM[1] += V3i.x() * V3i.y();
    MTM[2] += V3i.x() * V3i.z();
    MTM[4] += V3i.y() * V3i.y();
    MTM[5] += V3i.y() * V3i.z();
    MTM[8] += V3i.z() * V3i.z();
    left += V3i * diagJ1i;
  }
  MTM[3] = MTM[1];
  MTM[6] = MTM[2];
  MTM[7] = MTM[5];
  // pdot = MTM^{-1} * left
  // MINORs of MTM
  std::vector<double> MN_MTM(9, 0);
  MN_MTM[0] = MTM[4] * MTM[8] - MTM[5] * MTM[7];
  MN_MTM[1] = -MTM[1] * MTM[8] + MTM[2] * MTM[7];
  MN_MTM[2] = MTM[1] * MTM[5] - MTM[2] * MTM[4];
  MN_MTM[3] = -MTM[3] * MTM[8] + MTM[5] * MTM[6];
  MN_MTM[4] = MTM[0] * MTM[8] - MTM[2] * MTM[6];
  MN_MTM[5] = -MTM[0] * MTM[5] + MTM[2] * MTM[3];
  MN_MTM[6] = MTM[3] * MTM[7] - MTM[4] * MTM[6];
  MN_MTM[7] = -MTM[0] * MTM[7] + MTM[1] * MTM[6];
  MN_MTM[8] = MTM[0] * MTM[4] - MTM[1] * MTM[3];
  double det_MTM = MTM[0] * MN_MTM[0] + MTM[1] * MN_MTM[3] + MTM[2] * MN_MTM[6];
  if (det_MTM < K_EPSILON) {
    strs
        << ERR_DESCRIPTION[ERR_QUATTRO_DIFF_FK_SINGULAR]  //"Quattro forward
                                                          //different kinematics
                                                          //is singular"
        << ", in function " << __FUNCTION__ << ", at line " << __LINE__
        << std::endl;
    LOG_ERROR(strs);
    return -ERR_QUATTRO_DIFF_FK_SINGULAR;
  }
  Vec PPdot;
  PPdot.set_x(
      (MN_MTM[0] * left.x() + MN_MTM[1] * left.y() + MN_MTM[2] * left.z()) /
      det_MTM);
  PPdot.set_y(
      (MN_MTM[3] * left.x() + MN_MTM[4] * left.y() + MN_MTM[5] * left.z()) /
      det_MTM);
  PPdot.set_z(
      (MN_MTM[6] * left.x() + MN_MTM[7] * left.y() + MN_MTM[8] * left.z()) /
      det_MTM);
  v->setLinearVel(PPdot);
  v->setAngularVel(Vec());
  return 0;
}

int Quattro::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                       const Eigen::VectorXd &qddot, Pose *p, Twist *v,
                       Twist *a) {
  std::ostringstream strs;
  if (!p || !v || !a) {
    strs << ERR_DESCRIPTION[ERR_INPUT_POINTER_NULL] << ", in function "
         << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return -ERR_INPUT_POINTER_NULL;
  }
  if (!initialized_) {
    strs << ERR_DESCRIPTION[ERR_ROB_PARAM_NOT_INITIALIZED]  //"Quattro geometric
                                                            //parameters are not
                                                            //initialized"
         << " in function " << __FUNCTION__ << ", line " << __LINE__
         << std::endl;
    LOG_ERROR(strs);
    return -ERR_ROB_PARAM_NOT_INITIALIZED;
  }
  int ret = this->JntToCart(q, qdot, p, v);
  if (ret < 0) {
    return ret;
  }
  Vec PP = p->getTranslation();
  Vec pdot = v->getLinearVel();
  std::vector<double> MTM(9, 0);
  Vec left;
  for (size_t i = 0; i < qddot.size(); i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos(q(i) + delta1_);
    double sq = sin(q(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(diff_radius_ * ca, diff_radius_ * sa, 0) + V2i;
    Vec V3i = PP - tipi;
    Vec V3idot = pdot - (Ax1i * V2i) * qdot(i);
    double sq_V3idot = -V3idot.sqNorm();
    double qdot_sq = qdot[i] * qdot[i];
    double V3i_c_Ax1i_dot_Ax1i_c_V2i = (V3i * Ax1i).dot(Ax1i * V2i) * qdot_sq;

    double diagJ1i = (Ax1i * V2i).dot(V3i) * qddot(i) +
                     V3i_c_Ax1i_dot_Ax1i_c_V2i + sq_V3idot;
    /*
     *  M[0] M[1] M[2]
     *  M[3] M[4] M[5]
     *  M[6] M[7] M[8]
     */
    MTM[0] += V3i.x() * V3i.x();
    MTM[1] += V3i.x() * V3i.y();
    MTM[2] += V3i.x() * V3i.z();
    MTM[4] += V3i.y() * V3i.y();
    MTM[5] += V3i.y() * V3i.z();
    MTM[8] += V3i.z() * V3i.z();
    left += V3i * diagJ1i;
  }
  MTM[3] = MTM[1];
  MTM[6] = MTM[2];
  MTM[7] = MTM[5];
  // pdot = MTM^{-1} * left
  // MINORs of MTM
  std::vector<double> MN_MTM(9, 0);
  MN_MTM[0] = MTM[4] * MTM[8] - MTM[5] * MTM[7];
  MN_MTM[1] = -MTM[1] * MTM[8] + MTM[2] * MTM[7];
  MN_MTM[2] = MTM[1] * MTM[5] - MTM[2] * MTM[4];
  MN_MTM[3] = -MTM[3] * MTM[8] + MTM[5] * MTM[6];
  MN_MTM[4] = MTM[0] * MTM[8] - MTM[2] * MTM[6];
  MN_MTM[5] = -MTM[0] * MTM[5] + MTM[2] * MTM[3];
  MN_MTM[6] = MTM[3] * MTM[7] - MTM[4] * MTM[6];
  MN_MTM[7] = -MTM[0] * MTM[7] + MTM[1] * MTM[6];
  MN_MTM[8] = MTM[0] * MTM[4] - MTM[1] * MTM[3];
  double det_MTM = MTM[0] * MN_MTM[0] + MTM[1] * MN_MTM[3] + MTM[2] * MN_MTM[6];
  if (det_MTM < K_EPSILON) {
    strs
        << ERR_DESCRIPTION[ERR_QUATTRO_DIFF_FK_SINGULAR]  //"Quattro forward
                                                          //different kinematics
                                                          //is singular"
        << ", in function " << __FUNCTION__ << ", at line " << __LINE__
        << std::endl;
    LOG_ERROR(strs);
    return -ERR_QUATTRO_DIFF_FK_SINGULAR;
  }
  Vec PPddot;
  PPddot.set_x(
      (MN_MTM[0] * left.x() + MN_MTM[1] * left.y() + MN_MTM[2] * left.z()) /
      det_MTM);
  PPddot.set_y(
      (MN_MTM[3] * left.x() + MN_MTM[4] * left.y() + MN_MTM[5] * left.z()) /
      det_MTM);
  PPddot.set_z(
      (MN_MTM[6] * left.x() + MN_MTM[7] * left.y() + MN_MTM[8] * left.z()) /
      det_MTM);
  a->setLinearVel(PPddot);
  a->setAngularVel(Vec());
  return 0;
}

int Quattro::CartToJnt(const Pose &p, Eigen::VectorXd *q) {
  if (!q) {
    std::cout << "input joint angle parameter is null in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -12;
  }
  if (!initialized_) {
    std::cout << "Quattro geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  if (q->size() != A_DoF_) {
    q->resize(A_DoF_);
  }
  Vec PP = p.getTranslation();
  for (size_t i = 0; i < A_DoF_; i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double t1 = ca * diff_radius_ - PP.x();
    double t2 = sa * diff_radius_ - PP.y();
    double A1 = t1 * t1 + t2 * t2 + PP.z() * PP.z() + a_b1_ * a_b1_;
    double A2 =
        2 * diff_radius_ * a_b1_ - 2 * a_b1_ * (ca * PP.x() + sa * PP.y());
    double A3 = 2 * PP.z() * a_b1_;
    double tmp1 = sqrt(A2 * A2 + A3 * A3);
    if (tmp1 < K_EPSILON) {
      std::cout << "Quattro arm " << i + 1 << " is singular, can not cal. "
                << "inverse kinematics in function " << __FUNCTION__
                << ", line " << __LINE__ << std::endl;
      return -19;
    }
    double tmp2 = (d1_ * d1_ - A1) / tmp1;
    if (tmp2 > 1 || tmp2 < -1) {
      std::cout << "Quattro arm " << i + 1 << " with tip point "
                << PP.ToString() << " is out of workspace, can not cal. "
                << "inverse kinematics in function " << __FUNCTION__
                << ", line " << __LINE__ << std::endl;
      return -19;
    }
    double beta = atan2(A2, A3);
    // always use the elbow-down solution for each arms
    (*q)(i) = M_PI - asin(tmp2) - beta - delta1_;
    if ((*q)(i) > M_PI) {
      (*q)(i) -= 2 * M_PI;
    }
    if ((*q)(i) < -M_PI) {
      (*q)(i) += 2 * M_PI;
    }
  }
  return 0;
}

int Quattro::CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                       Eigen::VectorXd *qdot) {
  if (!q || !qdot) {
    std::cout << "input pose and twist parameter is null in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -12;
  }
  if (!initialized_) {
    std::cout << "Quattro geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  if (q->size() != A_DoF_ || qdot->size() != A_DoF_) {
    q->resize(A_DoF_);
    qdot->resize(A_DoF_);
  }
  int ret = this->CartToJnt(p, q);
  if (ret < 0) {
    return ret;
  }
  Vec PP = p.getTranslation();
  Vec PPdot = v.getLinearVel();
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos((*q)(i) + delta1_);
    double sq = sin((*q)(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(diff_radius_ * ca, diff_radius_ * sa, 0) + V2i;
    Vec V3i = PP - tipi;
    double diagJ1i = (Ax1i * V2i).dot(V3i);
    double righti = V3i.dot(PPdot);
    if (abs(diagJ1i) < K_EPSILON) {
      std::cout << "arm " << i << " in IK singular "
                << " at function " << __FUNCTION__ << ", line " << __LINE__
                << std::endl;
      return -21;
    }
    (*qdot)(i) = righti / diagJ1i;
  }
  return 0;
}

int Quattro::CartToJnt(const Pose &p, const Twist &v, const Twist &a,
                       Eigen::VectorXd *q, Eigen::VectorXd *qdot,
                       Eigen::VectorXd *qddot) {
  if (!q || !qdot || !qddot) {
    std::cout << "input q, qdot, qddot pointers are null in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -12;
  }
  if (!initialized_) {
    std::cout << "Quattro geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  if (q->size() != A_DoF_ || qdot->size() != A_DoF_ ||
      qddot->size() != A_DoF_) {
    q->resize(A_DoF_);
    qdot->resize(A_DoF_);
    qddot->resize(A_DoF_);
  }
  int ret = this->CartToJnt(p, v, q, qdot);
  if (ret < 0) {
    return ret;
  }
  Vec PP = p.getTranslation();
  Vec PPdot = v.getLinearVel();
  Vec PPddot = a.getLinearVel();
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos((*q)(i) + delta1_);
    double sq = sin((*q)(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);

    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(diff_radius_ * ca, diff_radius_ * sa, 0) + V2i;
    Vec V3i = PP - tipi;
    Vec V3idot = PPdot - (Ax1i * V2i) * (*qdot)(i);
    double sq_V3idot = -V3idot.sqNorm();
    double qdot_sq = (*qdot)(i) * (*qdot)(i);
    double V3i_c_Ax1i_dot_Ax1i_c_V2i = (V3i * Ax1i).dot(Ax1i * V2i) * qdot_sq;

    double diagJ1i = (Ax1i * V2i).dot(V3i);
    double righti = V3i.dot(PPddot) - sq_V3idot - V3i_c_Ax1i_dot_Ax1i_c_V2i;
    if (abs(diagJ1i) < K_EPSILON) {
      std::cout << "arm " << i << " in IK singular "
                << " at function " << __FUNCTION__ << ", line " << __LINE__
                << std::endl;
      return -21;
    }
    (*qddot)(i) = righti / diagJ1i;
  }
  return 0;
}

int Quattro::CalcPassive(const Eigen::VectorXd &q, const Pose &p,
                         Eigen::VectorXd *qpassive) {
  if (!qpassive) {
    std::cout << "input qpassive pointer is null in function " << __FUNCTION__
              << " at line " << __LINE__ << std::endl;
    return -21;
  }
  if (qpassive->size() != 17) {
    qpassive->resize(17);
    /*
    std::cout << "input qpassive pointer has wrong size "
             << __FUNCTION__ << " at line " << __LINE__
             << std::endl;
   return -22;
     */
  }
  // there are totally 13 passive joints (and additional 4 passive joints,
  // the ones which are in the parallelogram rotating longer side, are
  // mimicing the other side

  // I list the passive joint name here
  // passive[0] = "uleg1_con"   **** from upper leg 1 to upper connecting link 1
  // between upper leg 1 and lower leg 11
  // passive[1] = "con_lleg_1"   ** from upper connecting link 1 and lower leg
  // 11 passive[2] = passive[1]: the other 1 from upper connecting link 1 and
  // lower leg 12 is same as passive[1] passive[3] = "uleg2_con"  *** from upper
  // leg 2 to upper connecting link 2 between upper leg 2 and lower leg 21
  // passive[4] = "con_lleg_2"  *** from upper connecting link 2 and lower leg
  // 21 passive[5] = passive[4]: the other 1 from upper connecting link 2 and
  // lower leg 22  is same as passive[4] passive[6] = "uleg3_con"  *** from
  // upper leg 3 to upper connecting link 3 between upper leg 3 and lower leg 31
  // passive[7] = "con_lleg_3"  *** from upper connecting link 3 and lower leg
  // 31 passive[8]=passive[7]: the other 1 from upper connecting link 3 and
  // lower leg 32  is same as passive[7] passive[9] = "uleg4_con"  *** from
  // upper leg 4 to upper connecting link 4 between upper leg 4 and lower leg 41
  // passive[10] = "con_lleg_4"  *** from upper connecting link 4 and lower leg
  // 41 passive[11]= passive[10]: the other 1 from upper connecting link 4 and
  // lower leg 42  is same as passive[10] passive[12] = "con43_lleg_4"  *** from
  // lleg_42 to bottom connecting link 4,  should be equal to -passive[10]
  // passive[13] = "con33_lleg_3"  *** from lleg_32 to bottom connecting link 3,
  // should be equal to -passive[7] passive[14] = "con23_lleg_2"  *** from
  // lleg_22 to bottom connecting link 2, should be equal to -passive[4]
  // passive[15] = "con13_lleg_1"  *** from lleg_12 to bottom connecting link 1,
  // should be equal to -passive[1] passive[16] = "lleg1_base"  *** from lleg1
  // to moving platform , should be euqal to -passive[0]

  // so we need to compute passive[0] to passive[7];
  Vec PP = p.getTranslation();
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos(q(i) + delta1_);
    double sq = sin(q(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    // unit_x, unit_y, Ax1i constitute a frame
    Vec unit_x = V2i / V2i.Norm();
    Vec Ax1i(-sa, ca, 0);
    Vec unit_y = Ax1i * unit_x;

    Vec tipi = Vec(diff_radius_ * ca, diff_radius_ * sa, 0) + V2i;
    Vec V3i = PP - tipi;
    double comp_V3i_z = V3i.dot(Ax1i);
    // get angle gamma1 between V3i and Ax1i
    double cgamma1 = comp_V3i_z / V3i.Norm();
    (*qpassive)(3 * i + 1) = acos(cgamma1) - M_PI / 2.0;
    (*qpassive)(3 * i + 2) = (*qpassive)(3 * i + 1);

    // projection of V3i towards the p
    Vec V3i_proj = V3i - comp_V3i_z * Ax1i;
    double cgamma2 = V3i_proj.dot(unit_x);
    double sgamma2 = V3i_proj.dot(unit_y);
    (*qpassive)(3 * i) =
        atan2(sgamma2, cgamma2) - (M_PI / 2.0 + QuattroRvizInitAngle);
  }
  (*qpassive)(12) = -(*qpassive)(10);
  (*qpassive)(13) = -(*qpassive)(7);
  (*qpassive)(14) = -(*qpassive)(4);
  (*qpassive)(15) = -(*qpassive)(1);
  (*qpassive)(16) = -(*qpassive)(0) - q(0) - delta1_;
  return 0;
}

}  // namespace kinematics_lib