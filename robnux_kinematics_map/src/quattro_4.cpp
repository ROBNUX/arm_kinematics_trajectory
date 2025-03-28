#include "robnux_kinematics_map/quattro_4.hpp"
// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::Quattro_4,
                       kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

Quattro_4::Quattro_4()
    : BaseKinematicMap(4, 4),
      branchFlags_(std::vector<int>(1, 0)),
      jointTurns_(std::vector<int>(4, 0)) {
  // always bottom branch and 0 turns
  // DoF_ = 4;
  // A_DoF_ = 4;
  tipPoints.resize(DoF_);
  polyCoef_.resize(9);
  jnt_names_.resize(24);
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
  jnt_names_[20] = "lleg1_base_1";
  jnt_names_[21] = "base_1_2";
  jnt_names_[22] = "lleg3_base_3";
  jnt_names_[23] = "base_3_4";
}

Quattro_4::Quattro_4(const Eigen::VectorXd &parameters)
    : BaseKinematicMap(4, 4),
      // here branchFlags_ denotes the branch flag of FK of parallel robot
      branchFlags_(std::vector<int>(1, 0)),
      jointTurns_(std::vector<int>(4, 0)) {
  // DoF_ = 4;
  // A_DoF_= 4;
  tipPoints.resize(DoF_);
  polyCoef_.resize(9);
  SetGeometry(parameters);
  jnt_names_.resize(24);
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
  jnt_names_[20] = "lleg1_base_1";
  jnt_names_[21] = "base_1_2";
  jnt_names_[22] = "lleg3_base_3";
  jnt_names_[23] = "base_3_4";
}

void Quattro_4::SetGeometry(const Eigen::VectorXd &parameters) {
  if (parameters.size() >= 11) {
    R1_ = parameters(0);
    alpha_ = parameters(1);
    b1_ = parameters(2);
    c1_ = parameters(3);
    d1_ = parameters(4);
    h_ = parameters(5);
    m_ = parameters(6);

    // the following 4 paramters describs the offsets from
    v1x_ = parameters(7);
    v1y_ = parameters(8);
    v2x_ = parameters(9);
    v2y_ = parameters(10);
    movingPlatformOffsets.resize(DoF_);

    movingPlatformOffsets[0] = Vec(v1x_, v1y_, 0);
    movingPlatformOffsets[1] = Vec(v2x_, v2y_, 0);
    movingPlatformOffsets[2] = Vec(v2x_, v2y_, 0);
    movingPlatformOffsets[3] = Vec(v1x_, v1y_, 0);

    a_b1_ = sqrt(b1_ * b1_ + c1_ * c1_);
    delta1_ = atan2(c1_, b1_);
    // diff_radius_ = R1_ - r1_ - m_ / sqrt(2);
    char_length_ = b1_ + d1_;
    initialized_ = true;
  }
}

int Quattro_4::JntToCart(const Eigen::VectorXd &q, Pose *p) {
  if (!p) {
    std::cout << "input pose parameter is null in function " << __FUNCTION__
              << ", line " << __LINE__ << std::endl;
    return -12;
  }
  if (!initialized_) {
    std::cout << "Quattro geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  if (q.size() < DoF_) {
    std::cout << "input joint vector has wrong dimension in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -13;
  }
  // std::vector<Vec> tipPoints;
  for (size_t i = 0; i < DoF_; i++) {
    double proj_radius = R1_ + a_b1_ * cos(q(i) + delta1_);
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    tipPoints[i] = Vec(proj_radius * cos(angle), proj_radius * sin(angle),
                       -a_b1_ * sin(q(i) + delta1_)) +
                   movingPlatformOffsets[i];
  }

  return FindRootsWithEigen(tipPoints[0], tipPoints[1], tipPoints[2],
                            tipPoints[3], d1_, m_, p);
}

int Quattro_4::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                         Pose *p, Twist *v) {
  if (!p || !v) {
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
  int ret = this->JntToCart(q, p);
  if (ret < 0) {
    return ret;
  }
  if (qdot.size() != A_DoF_) {
    std::cout << " input joint velocity dimension is wrong "
              << " in function " << __FUNCTION__ << " at line" << __LINE__
              << std::endl;
  }
  Vec PP = p->getTranslation();
  Quaternion qq = p->getQuaternion();
  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);

  // matrix to be inverted for solving the twist given joint velocity
  Matrix4d M2;
  Vector4d b;
  Vec V0(-m_ * syaw / 2.0, m_ * cyaw / 2.0, 0);
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    }
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos(q(i) + delta1_);
    double sq = sin(q(i) + delta1_);
    // step 2, calculate Jacobian matrix
    // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
    // where Ax1i is the unit vector passing through the axis of joint i
    // V2i is the vector of large arm, V3i is the vector of parallelogram vec
    // std::vector<double> digJ1(A_DoF_);
    // we try to avoid use eigen library here
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(R1_ * ca, R1_ * sa, 0) + V2i;
    Vec V3i = PP1 - tipi;
    b(i) = (Ax1i * V2i).dot(V3i) * qdot(i);

    M2(i, 0) = V3i.x();
    M2(i, 1) = V3i.y();
    M2(i, 2) = V3i.z();
    if (i < 2) {
      M2(i, 3) = -V3i.dot(V0);
    } else {
      M2(i, 3) = V3i.dot(V0);
    }
  }
  double det = M2.determinant();
  if (fabs(det) < K_EPSILON) {
    std::cout << "Quattro_4 is singular, can not compute forward velocity "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -40;
  }
  Vector4d x = M2.inverse() * b;
  Vec PPdot(x(0), x(1), x(2));
  v->setLinearVel(PPdot);
  Vec angularVel(0, 0, x(3));
  v->setAngularVel(angularVel);
  return 0;
}

int Quattro_4::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                         const Eigen::VectorXd &qddot, Pose *p, Twist *v,
                         Twist *a) {
  if (!p || !v || !a) {
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
  int ret = this->JntToCart(q, qdot, p, v);
  if (ret < 0) {
    return ret;
  }
  Vec PP = p->getTranslation();
  Quaternion qq = p->getQuaternion();
  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);

  Vec PPdot = v->getLinearVel();
  Vec angularVel = v->getAngularVel();
  /*
  Vec Euler(yaw, pitch, roll);
  Vec EulerDot;
  if (!Rotation::GetEulerVelZYX(Euler, angularVel, &EulerDot)) {
      std::cout << "Quattro_4 compute yawdot of current angularVel got error "
            << " in function "
            << __FUNCTION__ << ", line " << __LINE__ << std::endl;
      return -38;
  }
   */
  double yawdot = angularVel.z();  // EulerDot.x();
  double yawdot_sq = yawdot * yawdot;

  // matrix to be inverted for solving the twist given joint velocity
  Matrix4d M2;
  Vector4d b;
  Vec V0(-m_ * syaw / 2.0, m_ * cyaw / 2.0, 0);
  Vec V0dot(-m_ * cyaw / 2.0, -m_ * syaw / 2.0, 0);
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1, PP1dot;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
      PP1dot = PPdot - V0 * yawdot;
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
      PPdot = PPdot + V0 * yawdot;
    }
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos(q(i) + delta1_);
    double sq = sin(q(i) + delta1_);
    // step 2, calculate Jacobian matrix
    // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
    // where Ax1i is the unit vector passing through the axis of joint i
    // V2i is the vector of large arm, V3i is the vector of parallelogram vec
    // std::vector<double> digJ1(A_DoF_);
    // we try to avoid use eigen library here
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(R1_ * ca, R1_ * sa, 0) + V2i;
    Vec V3i = PP1 - tipi;
    Vec V3idot = PP1dot - (Ax1i * V2i) * qdot(i);
    double sq_V3idot = -V3idot.sqNorm();
    double VRyawdot_sq;
    if (i < 2) {
      VRyawdot_sq = V3i.dot(V0dot) * yawdot_sq;
    } else {
      VRyawdot_sq = -V3i.dot(V0dot) * yawdot_sq;
    }
    double V3i_c_Ax1i_dot_Ax1i_c_V2i = (V3i * Ax1i).dot(Ax1i * V2i);

    b(i) = (Ax1i * V2i).dot(V3i) * qddot(i) +
           V3i_c_Ax1i_dot_Ax1i_c_V2i * qdot(i) * qdot(i) + sq_V3idot +
           VRyawdot_sq;

    M2(i, 0) = V3i.x();
    M2(i, 1) = V3i.y();
    M2(i, 2) = V3i.z();
    if (i < 2) {
      M2(i, 3) = -V3i.dot(V0);
    } else {
      M2(i, 3) = V3i.dot(V0);
    }
  }
  double det = M2.determinant();
  if (fabs(det) < K_EPSILON) {
    std::cout << "Quattro_4 is singular, can not compute forward velocity "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -40;
  }
  Vector4d x = M2.inverse() * b;
  Vec PPddot(x(0), x(1), x(2));
  a->setLinearVel(PPdot);
  Vec angularAcc(0, 0, x(3));
  a->setAngularVel(angularAcc);
  return 0;
}

int Quattro_4::CartToJnt(const Pose &p, Eigen::VectorXd *q) {
  if (!q) {
    std::cout << "input joint angle parameter is null in function "
              << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    return -12;
  }
  if (!initialized_) {
    std::cout << "Quattro_4 geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  if (q->size() != A_DoF_) {
    q->resize(A_DoF_);
  }
  Vec PP = p.getTranslation();
  Quaternion qq = p.getQuaternion();

  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);
  // std::vector<Vec> tipRect(4);
  // Vec tipRect[0] = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0;
  // Vec tipRect[1] = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0;
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    }
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double t1 = ca * R1_ - PP1.x();
    double t2 = sa * R1_ - PP1.y();
    double A1 = t1 * t1 + t2 * t2 + PP1.z() * PP1.z() + a_b1_ * a_b1_;
    double A2 = 2 * R1_ * a_b1_ - 2 * a_b1_ * (ca * PP1.x() + sa * PP1.y());
    double A3 = 2 * PP1.z() * a_b1_;
    double tmp1 = sqrt(A2 * A2 + A3 * A3);
    if (tmp1 < K_EPSILON) {
      std::cout << "Quattro_4 arm " << i + 1 << " is singular, can not cal. "
                << "inverse kinematics in function " << __FUNCTION__
                << ", line " << __LINE__ << std::endl;
      return -19;
    }
    double tmp2 = (d1_ * d1_ - A1) / tmp1;
    if (tmp2 > 1 || tmp2 < -1) {
      std::cout << "Quattro_4 arm " << i + 1 << " with tip point "
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

int Quattro_4::CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
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
  Quaternion qq = p.getQuaternion();

  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);
  Vec PPdot = v.getLinearVel();
  Vec angularVel = v.getAngularVel();
  double yawdot = angularVel.z();

  Vec V0(-m_ * syaw / 2.0, m_ * cyaw / 2.0, 0);
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    }
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos((*q)(i) + delta1_);
    double sq = sin((*q)(i) + delta1_);
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(R1_ * ca, R1_ * sa, 0) + V2i;
    Vec V3i = PP1 - tipi;
    double diagJ1i = (Ax1i * V2i).dot(V3i);
    double righti;
    if (i < 2) {
      righti = V3i.dot(PPdot - V0 * yawdot);
    } else {
      righti = V3i.dot(PPdot + V0 * yawdot);
    }
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

int Quattro_4::CartToJnt(const Pose &p, const Twist &v, const Twist &a,
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
  Quaternion qq = p.getQuaternion();
  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);
  Vec angularVel = v.getAngularVel();
  Vec angularAcc = a.getAngularVel();
  double yawdot = angularVel.z();  // EulerDot.x();
  double yawdot_sq = yawdot * yawdot;
  double yawddot = angularAcc.z();

  // matrix to be inverted for solving the twist given joint velocity
  Matrix4d M2;
  Vector4d b;
  Vec V0(-m_ * syaw / 2.0, m_ * cyaw / 2.0, 0);
  Vec V0dot(-m_ * cyaw / 2.0, -m_ * syaw / 2.0, 0);
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1, PP1dot;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
      PP1dot = PPdot - V0 * yawdot;
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
      PPdot = PPdot + V0 * yawdot;
    }
    // assume all actuated arms are symmetrically arranged around a circle
    double angle = alpha_ + i * 2 * M_PI / A_DoF_;
    double ca = cos(angle);
    double sa = sin(angle);
    double cq = cos((*q)(i) + delta1_);
    double sq = sin((*q)(i) + delta1_);
    // step 2, calculate Jacobian matrix
    // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
    // where Ax1i is the unit vector passing through the axis of joint i
    // V2i is the vector of large arm, V3i is the vector of parallelogram vec
    // std::vector<double> digJ1(A_DoF_);
    // we try to avoid use eigen library here
    Vec V2i(a_b1_ * cq * ca, a_b1_ * cq * sa, -a_b1_ * sq);
    Vec Ax1i(-sa, ca, 0);
    Vec tipi = Vec(R1_ * ca, R1_ * sa, 0) + V2i;
    Vec V3i = PP1 - tipi;
    Vec V3idot = PP1dot - (Ax1i * V2i) * (*qdot)(i);
    double VRyawdot_sq;
    if (i < 2) {
      VRyawdot_sq = V3i.dot(V0dot) * yawdot_sq;
    } else {
      VRyawdot_sq = -V3i.dot(V0dot) * yawdot_sq;
    }
    double sq_V3idot = -V3idot.sqNorm();
    double V3i_c_Ax1i_dot_Ax1i_c_V2i = (V3i * Ax1i).dot(Ax1i * V2i);

    double diagJ1i = (Ax1i * V2i).dot(V3i);
    double righti;
    if (i < 2) {
      righti = V3i.dot(PPddot - V0 * yawddot);
    } else {
      righti = V3i.dot(PPddot + V0 * yawddot);
    }

    righti += -VRyawdot_sq - sq_V3idot -
              V3i_c_Ax1i_dot_Ax1i_c_V2i * (*qdot)(i) * (*qdot)(i);

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

int Quattro_4::CalcPassive(const Eigen::VectorXd &q, const Pose &p,
                           Eigen::VectorXd *qpassive) {
  if (!qpassive) {
    std::cout << "input qpassive pointer is null in function " << __FUNCTION__
              << " at line " << __LINE__ << std::endl;
    return -21;
  }
  if (qpassive->size() != 20) {
    qpassive->resize(20);
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
  // should be equal to -passive[1] passive[16] = "lleg1_base_1"  *** from lleg1
  // to moving platform link 1 , should be euqal to -passive[0] passive[17] =
  // "base_1_2" ** from moving platform link 1 to moving platform link 2
  // passive[18] = "lleg3_base_3" *** from lleg3 to moving platform link 3
  // passive[19] = "base_3_4" ** from moving platform link 3 to moving platform
  // link 4

  // so we need to compute passive[0] to passive[7];
  Vec PP = p.getTranslation();
  Quaternion qq = p.getQuaternion();
  double yaw, pitch, roll;
  if (!qq.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "Quattro_4 compute yaw of current pose got error "
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -37;
  }
  double cyaw = cos(yaw);
  double syaw = sin(yaw);
  // step 2, calculate Jacobian matrix
  // J1 qdot = M Pdot, J1 is a diagnal matrix diag( Ax1i * V2i)^T V3i
  // where Ax1i is the unit vector passing through the axis of joint i
  // V2i is the vector of large arm, V3i is the vector of parallelogram vec
  // we try to avoid use eigen library here
  // M = [V31^T; V32^T; V33^T;...]
  for (size_t i = 0; i < A_DoF_; i++) {
    Vec PP1;
    if (i < 2) {
      PP1 = PP - Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    } else {
      PP1 = PP + Vec(m_ * cyaw, m_ * syaw, 0) / 2.0 - movingPlatformOffsets[i];
    }
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

    Vec tipi = Vec(R1_ * ca, R1_ * sa, 0) + V2i;
    Vec V3i = PP1 - tipi;
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
  (*qpassive)(16) = -(*qpassive)(0) - q[0] - delta1_;
  (*qpassive)(17) = yaw - 3 * M_PI / 4;
  (*qpassive)(18) = -(*qpassive)(6) - q[2] - delta1_;
  (*qpassive)(19) = (*qpassive)(17);
  return 0;
}

int Quattro_4::SolvePolyRoots(std::vector<double> *solution) {
  if (!solution) {
    std::cout << "input parameter is null in function " << __FUNCTION__
              << " at line " << __LINE__ << std::endl;
    return -32;
  }
  // check if there are any higher order coefficients are 0,
  // then the problem can be simplified
  int ind = 0;
  for (ind = 0; ind < 8; ind++) {
    if (fabs(polyCoef_[ind]) > POLY_EPSILON) {
      break;
    }
  }

  int matDim = 8 - ind;
  // with the polynormial coeffcient, forming a matrix
  // and then using eigen for solving the eigenvalues
  // Matrix <double, matDim, matDim> M2;
  MatrixXd M2(matDim, matDim);

  for (int i = 0; i < matDim - 1; i++) {
    for (int j = 0; j < matDim; j++) {
      if (j != i + 1) {
        M2(i, j) = 0.0;
      } else {
        M2(i, j) = 1.0;
      }
    }
  }
  for (int j = 0; j < matDim; j++) {
    M2(matDim - 1, j) = -polyCoef_[8 - j] / polyCoef_[ind];
  }
  // now using eigen to solve the eigenvalues
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(M2);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  std::vector<std::tuple<float, Eigen::VectorXd>> eigen_vectors_and_values;

  for (int i = 0; i < eigen_values.size(); i++) {
    std::tuple<float, Eigen::VectorXd> vec_and_val(eigen_values[i],
                                                   eigen_vectors.row(i));
    eigen_vectors_and_values.push_back(vec_and_val);
  }
  std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
            [&](const std::tuple<float, Eigen::VectorXd> &a,
                const std::tuple<float, Eigen::VectorXd> &b) -> bool {
              return std::get<0>(a) <= std::get<0>(b);
            });
  int index = 0;
  for (auto const vect : eigen_vectors_and_values) {
    eigen_values(index) = std::get<0>(vect);
    eigen_vectors.row(index) = std::get<1>(vect);
    index++;
  }
  // now eigen_values is a vector containing actual eigen_values form
  // largest to smallest
  // first try to use the largest eigen_values
  if (index > 0) {
    solution->resize(index);
    for (int i = 0; i < index; i++) {
      solution->at(i) = eigen_values(i);
    }
  }
  return 0;
}

int Quattro_4::FindRootsWithEigen(const Vec &B1, const Vec &B2, const Vec &B3,
                                  const Vec &B4, const double &lower_arm_length,
                                  const double &move_platform_length,
                                  Pose *pos) {
  if (!pos) {
    std::cout << "input pos pointer is null in function " << __FUNCTION__
              << " at line " << __LINE__ << std::endl;
    return -35;
  }
  Vec n1 = B2 - B1;  // limb2 upper arm tip - limb1 uppder arm tip
  double norm_n1 = n1.Norm();
  if (norm_n1 < K_EPSILON) {
    std::cout << "B1, B2 overlapps in function " << __FUNCTION__ << " and line "
              << __LINE__ << std::endl;
    return -27;
  }
  n1 = n1 / norm_n1;  // normalize

  Vec n2 = B4 - B3;
  double norm_n2 = n2.Norm();
  if (norm_n2 < K_EPSILON) {
    std::cout << "B1, B2 overlapps in function " << __FUNCTION__ << " and line "
              << __LINE__ << std::endl;
    return -27;
  }
  n2 = n2 / norm_n2;  // normalize
  double csq = lower_arm_length * lower_arm_length;
  double norm_n1_sq = norm_n1 * norm_n1;
  double norm_n2_sq = norm_n2 * norm_n2;
  if (csq < norm_n1_sq / 4.0 || csq < norm_n2_sq / 4.0) {
    std::cout << "parallelogram length is too short to"
              << " close the loop, no FK solution in " << __FUNCTION__
              << " and line " << __LINE__ << std::endl;
    return -28;
  }
  double r1 = sqrt(csq - norm_n1_sq / 4.0);
  double r2 = sqrt(csq - norm_n2_sq / 4.0);

  // center of the circle formed by D1, recall D1 has same distance
  // (lower_arm_length) from B1 and B2
  Vec O1 = (B1 + B2) / 2.0;
  // center of the circle formed by D2, recall D2 has same distance
  // (lower_arm_length) from B3 and B4
  Vec O2 = (B3 + B4) / 2.0;

  // norm compute the matrix K1 and K2
  double matK1[9], matK2[9];
  double sg1 = n1.x();
  double cg1 = sqrt(1 - sg1 * sg1);
  if (cg1 < K_EPSILON) {
    std::cout << "It is singular in computing the basis of circles"
              << "in function " << __FUNCTION__ << " and line " << __LINE__
              << std::endl;
    return -28;
  }
  double st1 = -n1.y() / cg1;
  double ct1 = n1.z() / cg1;
  matK1[0] = r1 * cg1;
  matK1[1] = 0;
  matK1[2] = O1.x();
  matK1[3] = r1 * st1 * sg1;
  matK1[4] = r1 * ct1;
  matK1[5] = O1.y();
  matK1[6] = -r1 * ct1 * sg1;
  matK1[7] = r1 * st1;
  matK1[8] = O1.z();

  double sg2 = n2.x();
  double cg2 = sqrt(1 - sg2 * sg2);
  if (cg2 < K_EPSILON) {
    std::cout << "It is singular in computing the basis of circles"
              << "in function " << __FUNCTION__ << " and line " << __LINE__
              << std::endl;
    return -28;
  }
  double st2 = -n2.y() / cg2;
  double ct2 = n2.z() / cg2;
  matK2[0] = r2 * cg2;
  matK2[1] = 0;
  matK2[2] = O2.x();
  matK2[3] = r2 * st2 * sg2;
  matK2[4] = r2 * ct2;
  matK2[5] = O2.y();
  matK2[6] = -r2 * ct2 * sg2;
  matK2[7] = r2 * st2;
  matK2[8] = O2.z();

  double s1 = sqrt(matK1[6] * matK1[6] + matK1[7] * matK1[7]);
  double s2 = sqrt(matK2[6] * matK2[6] + matK2[7] * matK2[7]);
  if (s1 < K_EPSILON || s2 < K_EPSILON) {
    std::cout << "s1 and s2 are close to 0 "
              << "in function " << __FUNCTION__ << " and line " << __LINE__
              << std::endl;
    return -29;
  }
  double sta = matK1[6] / s1;
  double cta = matK1[7] / s1;
  double ta = atan2(sta, cta);
  double stb = matK2[6] / s2;
  double ctb = matK2[7] / s2;
  double tb = atan2(stb, ctb);

  // compute all pij, see paper J42_JMR.pdf
  double p11 = s1 / s2;
  double p12 = (matK1[8] - matK2[8]) / s2;
  double p21 = matK1[0] * cta - matK1[1] * sta;
  double p22 =
      matK1[0] * sta + matK1[1] * cta - (matK2[0] * stb + matK2[1] * ctb) * p11;
  double p23 = -matK2[0] * ctb + matK2[1] * stb;
  double p24 = matK1[2] - matK2[2] - (matK2[0] * stb + matK2[1] * ctb) * p12;
  double p25 = matK1[3] * cta - matK1[4] * sta;
  double p26 =
      matK1[3] * sta + matK1[4] * cta - (matK2[3] * stb + matK2[4] * ctb) * p11;
  double p27 = -matK2[3] * ctb + matK2[4] * stb;
  double p28 = matK1[5] - matK2[5] - (matK2[3] * stb + matK2[4] * ctb) * p12;

  double p21s = p21 * p21;
  double p22s = p22 * p22;
  double p25s = p25 * p25;
  double p23s = p23 * p23;
  double p24s = p24 * p24;
  double p27s = p27 * p27;
  double p26s = p26 * p26;
  double p11s = p11 * p11;
  double p28s = p28 * p28;
  double p12s = p12 * p12;
  double q3 = p21s + p25s;
  double q8 = p21 * p22 + p25 * p26;
  double q8s = q8 * q8;

  double q2 = 2 * q8;  // 2 * p21*p22 + 2*p25*p26;
  double q2s = q2 * q2;

  double q3s = q3 * q3;
  double q4 = p21 * p23 + p25 * p27;
  double q4s = q4 * q4;
  double q5 = p22 * p23 + p26 * p27;
  double q5s = q5 * q5;
  double q11 = p23s + p27s;
  double q6 = -q11 * p11s + p22s + p26s;
  double q6s = q6 * q6;
  double q7 = q2s + 2 * q3 * q6 - q3s + 4 * p11s * q4s - 4 * p11s * q5s - q6s;
  double q7s = q7 * q7;

  double q9 = p21 * p24 + p25 * p28;
  double q13 = 2 * q9;
  double q13s = q13 * q13;
  double q10 = p22 * p24 + p26 * p28;
  double q12 = p23 * p24 + p27 * p28;
  double q12s = q12 * q12;

  double q1 = -4 * q8 * q3 + 4 * q8 * q6 + 8 * p11s * q4 * q5;

  double q1s = q1 * q1;
  double ee = move_platform_length;
  double ees = ee * ee;
  double q14 = ees - p24s - p28s + (p12s - 1) * q11;
  double q14s = q14 * q14;
  double q15 = 2 * q10 - 2 * p11 * p12 * q11;
  double q15s = q15 * q15;
  double q16 = 8 * q8 * q9 + 2 * q3 * q15 - 2 * q15 * q6 + 8 * p11 * p12 * q4s -
               8 * p11 * p12 * q5s - 8 * p11s * q5 * q12;

  double q16s = q16 * q16;
  double q17 = -4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 8 * p11s * q4 * q12 +
               16 * p11 * p12 * q4 * q5;

  double q17s = q17 * q17;
  double q20 = 4 * p12s - 4;
  double q18 = -2 * q3 * q14 + q20 * q4s + q20 * q12s + q13s + q3s + q14s;

  double q18s = q18 * q18;

  double q19 = 4 * q9 * q3 - 4 * q9 * q14 + q20 * q4 * 2 * q12;

  double q19s = q19 * q19;

  double q21 = 4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q14 +
               8 * p11 * p12 * q4s + 8 * p11 * p12 * q12s + 2 * q20 * q5 * q12;

  double q21s = q21 * q21;
  double q22 = 4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + q20 * q4 * 2 * q5 +
               16 * p11 * p12 * q4 * q12;

  double q22s = q22 * q22;

  double q23 = -4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 +
               4 * p11s * q4 * 2 * q12 + 8 * p11 * p12 * q4 * 2 * q5;

  double q23s = q23 * q23;
  double q24 = 2 * q3 * q14 - q20 * q4s + q20 * q5s + q2s - q13s + 2 * q3 * q6 -
               2 * q6 * q14 - 2 * q3s + 4 * p11s * q4s + 4 * p11s * q12s +
               q15s + 16 * p11 * p12 * q5 * q12;

  double q24s = q24 * q24;

  polyCoef_[0] = q1s + q7s;

  polyCoef_[1] =
      2 *
          (8 * q8 * q9 + 2 * q3 * q15 - 2 * q15 * q6 + 8 * p11 * p12 * q4s -
           8 * p11 * p12 * q5s - 8 * p11s * q5 * q12) *
          (q2s + 2 * q3 * q6 - q3s + 4 * p11s * q4s - 4 * p11s * q5s - q6s) +
      2 * (-4 * q8 * q3 + 4 * q8 * q6 + 4 * p11s * q4 * 2 * q5) *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 8 * p11s * q4 * q12 +
           16 * p11 * p12 * q4 * q5);

  polyCoef_[2] =
      -2 * (q2s + 2 * q3 * q6 - q3s + 4 * p11s * q4s - 4 * p11s * q5s - q6s) *
          (2 * q3 * q14 - q20 * q4s + q20 * q5s + q2s - q13s + 2 * q3 * q6 -
           2 * q6 * q14 - 2 * q3s + 4 * p11s * q4s + 4 * p11s * q12s + q15s +
           16 * p11 * p12 * q5 * q12) +
      2 * (-4 * q8 * q3 + 4 * q8 * q6 + 8 * p11s * q4 * q5) *
          (4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + 2 * q20 * q4 * q5 +
           16 * p11 * p12 * q4 * q12) -
      q1s + q16s + q17s;

  polyCoef_[3] =
      -2 *
          (8 * q8 * q9 + 2 * q3 * q15 - 2 * q15 * q6 + 8 * p11 * p12 * q4s -
           8 * p11 * p12 * q5s - 8 * p11s * q5 * q12) *
          (2 * q3 * q14 - q20 * q4s + q20 * q5s + q2s - q13s + 2 * q3 * q6 -
           2 * q6 * q14 - 2 * q3s + 4 * p11s * q4s + 4 * p11s * q12s + q15s +
           16 * p11 * p12 * q5 * q12) +
      2 *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 8 * p11s * q4 * q12 +
           16 * p11 * p12 * q4 * q5) *
          (4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + 2 * q20 * q4 * q5 +
           16 * p11 * p12 * q4 * q12) -
      2 * (-4 * q8 * q3 + 4 * q8 * q6 + 8 * p11s * q4 * q5) *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 8 * p11s * q4 * q12 +
           16 * p11 * p12 * q4 * q5) -
      2 * (q2s + 2 * q3 * q6 - q3s + 4 * p11s * q4s - 4 * p11s * q5s - q6s) *
          (8 * q8 * q9 + 2 * q3 * q15 - 2 * q15 * q14 + 8 * p11 * p12 * q4s +
           8 * p11 * p12 * q12s + 2 * q20 * q5 * q12) +
      2 * (4 * q9 * q3 - 4 * q9 * q14 + 2 * q20 * q4 * q12) *
          (-4 * q8 * q3 + 4 * q8 * q6 + 8 * p11s * q4 * q5);

  polyCoef_[4] =
      -2 * (-4 * q8 * q3 + 4 * q8 * q6 + 4 * p11s * q4 * 2 * q5) *
          (4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + q20 * q4 * 2 * q5 +
           8 * p11 * p12 * q4 * 2 * q12) +
      2 * (4 * q9 * q3 - 4 * q9 * q14 + q20 * q4 * 2 * q12) *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 4 * p11s * q4 * 2 * q12 +
           8 * p11 * p12 * q4 * 2 * q5) +
      q22s -
      2 *
          (4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q6 + 8 * p11 * p12 * q4s -
           8 * p11 * p12 * q5s - 8 * p11s * q5 * q12) *
          (4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q14 + 8 * p11 * p12 * q4s +
           8 * p11 * p12 * q12s + 2 * q20 * q5 * q12) -
      q23s -
      2 * (q2s + 2 * q3 * q6 - q3s + 4 * p11s * q4s - 4 * p11s * q5s - q6s) *
          (-2 * q3 * q14 + q20 * q4s + q20 * q12s + q13s + q3s + q14s) +
      q24s;

  polyCoef_[5] =
      -2 *
          (4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q6 + 8 * p11 * p12 * q4s -
           8 * p11 * p12 * q5s - 8 * p11s * q5 * q12) *
          (-2 * q3 * q14 + q20 * q4s + q20 * q12s + q13s + q3s + q14s) -
      2 *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 4 * p11s * q4 * 2 * q12 +
           8 * p11 * p12 * q4 * 2 * q5) *
          (4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + q20 * q4 * 2 * q5 +
           8 * p11 * p12 * q4 * 2 * q12) +
      2 *
          (4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q14 + 8 * p11 * p12 * q4s +
           8 * p11 * p12 * q12s + 2 * q20 * q5 * q12) *
          (2 * q3 * q14 - q20 * q4s + q20 * q5s + q2s - q13s + 2 * q3 * q6 -
           2 * q6 * q14 - 2 * q3s + 4 * p11s * q4s + 4 * p11s * q12s + q15s +
           16 * p11 * p12 * q5 * q12) +
      2 * (4 * q9 * q3 - 4 * q9 * q14 + q20 * q4 * 2 * q12) *
          (4 * q8 * q3 - 4 * q8 * q14 + 4 * q9 * q15 + q20 * q4 * 2 * q5 +
           8 * p11 * p12 * q4 * 2 * q12) -
      2 * (4 * q9 * q3 - 4 * q9 * q14 + q20 * q4 * 2 * q12) *
          (-4 * q8 * q3 + 4 * q8 * q6 + 4 * p11s * q4 * 2 * q5);

  polyCoef_[6] =
      -2 * (4 * q9 * q3 - 4 * q9 * q14 + q20 * q4 * 2 * q12) *
          (-4 * q9 * q3 + 4 * q8 * q15 + 4 * q9 * q6 + 8 * p11s * q4 * q12 +
           16 * p11 * p12 * q4 * q5) +
      q21s - q22s +
      2 * (-2 * q3 * q14 + q20 * q4s + q20 * q12s + q13s + q3s + q14s) *
          (2 * q3 * q14 - q20 * q4s + q20 * q5s + q2s - q13s + 2 * q3 * q6 -
           2 * q6 * q14 - 2 * q3s + 4 * p11s * q4s + 4 * p11s * q12s + q15s +
           16 * p11 * p12 * q5 * q12) +
      q19s;

  polyCoef_[7] =
      2 *
          (4 * q8 * q13 + 2 * q3 * q15 - 2 * q15 * q14 + 8 * p11 * p12 * q4s +
           8 * p11 * p12 * q12s + 2 * q20 * q5 * q12) *
          (-2 * q3 * q14 + q20 * q4s + q20 * q12s + q13s + q3s + q14s) -
      2 * q19 * q22;

  polyCoef_[8] = q18s - q19s;

  // solve the polynomial
  // polyCoef_[0] * t^8 + polyCoef_[1] * t^7 + ... + polyCoef_[7] * t +
  // polyCoef_[8] = 0
  std::vector<double> sol;
  int ret = SolvePolyRoots(&sol);
  if (ret < 0) {
    return ret;
  }
  if (sol.empty()) {
    std::cout << "Polynomial has no solution using eigen"
              << " in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    return -33;
  }
  // sol[0] is sin(alpha')
  bool has_sol = false;
  Vec p1, p2;
  double yaw;
  for (size_t i = 0; i < sol.size(); i++) {
    // choose the root using the branchFlags_[0]
    double salphap = sol[i];
    // recall root itself is sin(alpha')
    double sbetap = p11 * salphap + p12;
    if ((fabs(salphap) > 1.0) || (fabs(sbetap) > 1.0)) {
      std::cout << "FK fails because the chosen root of polynomial is > 1"
                << " in function " << __FUNCTION__ << " at line " << __LINE__
                << std::endl;
      return -34;
    }
    // next branch flag, branchFlags_[1], determines which final solution
    // to use: asin/asin, PI-asin/asin, asin/PI-asin, or PI-asin/PI-asin
    double alpha, beta, ca, sa, cb, sb;
    if (branchFlags_[0] == 0) {
      alpha = asin(salphap) - ta;
      beta = asin(sbetap) - tb;
    } else if (branchFlags_[1] == 1) {
      alpha = M_PI - asin(salphap) - ta;
      beta = asin(sbetap) - tb;
    } else if (branchFlags_[1] == 2) {
      alpha = asin(salphap) - ta;
      beta = M_PI - asin(sbetap) - tb;
    } else {
      alpha = M_PI - asin(salphap) - ta;
      beta = M_PI - asin(sbetap) - tb;
    }
    ca = cos(alpha);
    sa = sin(alpha);
    cb = cos(beta);
    sb = sin(beta);
    p1.set_x(matK1[0] * ca + matK1[1] * sa + matK1[2]);
    p1.set_y(matK1[3] * ca + matK1[4] * sa + matK1[5]);
    p1.set_z(matK1[6] * ca + matK1[7] * sa + matK1[8]);

    p2.set_x(matK2[0] * cb + matK2[1] * sb + matK2[2]);
    p2.set_y(matK2[3] * cb + matK2[4] * sb + matK2[5]);
    p2.set_z(matK2[6] * cb + matK2[7] * sb + matK2[8]);

    Vec diffP = p2 - p1;
    double diffP_norm = diffP.Norm();
    yaw = atan2(diffP.y(), diffP.x());
    if (fabs(diffP_norm - ee) < QUATTRO_4_LINKLENGTH_DIFF &&
        (yaw > M_PI / 4 - K_EPSILON || yaw < -3 * M_PI / 4 + K_EPSILON) &&
        p2.z() < 0) {
      has_sol = true;
      break;
    } else {
      std::cout << "solution check fails, diffP_norm-ee= "
                << fabs(diffP_norm - ee) << ", yaw=" << yaw << std::endl;
    }
  }

  if (!has_sol) {
    std::cout << "FK for quattro 4 fails"
              << " in function " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    return -35;
  }
  pos->setTranslation((p1 + p2) / 2.0);
  Quaternion quat;
  quat.SetEulerZYX(yaw, 0.0, 0.0);
  pos->setQuaternion(quat);
  pos->setBranchFlags(branchFlags_);
  return 0;
}

}  // namespace kinematics_lib