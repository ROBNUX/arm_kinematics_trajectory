#include "kinematics_map/scara.hpp"

#include <cmath>

#include "kdl_common/pose.hpp"

// register plugin
PLUGINLIB_EXPORT_CLASS(kinematics_lib::Scara, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

Scara::Scara()
    : BaseKinematicMap(4, 4),
      alpha_(Eigen::VectorXd::Zero(4)),
      a_(Eigen::VectorXd::Zero(4)),
      d_(Eigen::VectorXd::Zero(4)),
      theta_(Eigen::VectorXd::Zero(4)) {
  jnt_names_.resize(4);
  jnt_names_[0] = "JOINT_1_ACT";
  jnt_names_[1] = "JOINT_2_ACT";
  jnt_names_[2] = "JOINT_3_ACT";
  jnt_names_[3] = "JOINT_4_ACT";
}

Scara::Scara(const Eigen::VectorXd &kine_para) : BaseKinematicMap(4, 4) {
  SetGeometry(kine_para);
  jnt_names_.resize(4);
  jnt_names_[0] = "JOINT_1_ACT";
  jnt_names_[1] = "JOINT_2_ACT";
  jnt_names_[2] = "JOINT_3_ACT";
  jnt_names_[3] = "JOINT_4_ACT";
}

// kine_para =[ [alpha], [a], [theta], [d] ]
void Scara::SetGeometry(const Eigen::VectorXd &kine_para) {
  if (initialized_) {
    std::cout << "Scara has already been initialized in " << __FUNCTION__
              << " and at line " << __LINE__ << std::endl;
    return;
  }
  if (kine_para.size() == 4 * DoF_) {
    for (size_t i = 0; i < DoF_; i++) {
      alpha_(i) = kine_para(i);
      a_(i) = kine_para(DoF_ + i);
      theta_(i) = kine_para(2 * DoF_ + i);
      d_(i) = kine_para(3 * DoF_ + i);
    }
    char_length_ = a_[1] + a_[2];
    initialized_ = true;
  } else {
    std::cout << "input parameters has dimension not equal to 4 in "
              << __FUNCTION__ << " line " << __LINE__ << std::endl;
  }
}

int Scara::JntToCart(const Eigen::VectorXd &q, Pose *p) {
  if (!p) {
    std::cout << "input pose pointer is null in " << __FUNCTION__
              << ", at line " << __LINE__ << std::endl;
  }
  d_(2) = q(2);
  theta_(0) = q(0);
  theta_(1) = q(1);
  theta_(3) = q(3);
  Frame tmp;
  for (size_t i = 0; i < DoF_; i++) {
    tmp = tmp * Frame::DH_Craig1989(a_(i), alpha_(i), d_(i), theta_(i));
  }

  // for scara, there is only 1 branch flag: elbow (up or down)
  std::vector<int> branchFlags(1, eBranchLeft);
  // 4 turn flags, actually only 3 turn flags (because joint 3 is prismatic
  std::vector<int> jointTurns(4, 0);
  double tmp_q;
  for (size_t i = 0; i < DoF_; i++) {
    if (i != 2) {
      jointTurns[i] = std::floor(q(i) / (2 * M_PI));
      tmp_q = q(i) - jointTurns[i] * 2 * M_PI;
      // then make sure [-PI, PI]
      // to have 0 turn here
      if (tmp_q > M_PI) {
        jointTurns[i] += 1;
      }
    }
    if (i == 1) {
      if (tmp_q >= 0 && tmp_q < M_PI) {
        branchFlags[0] = eBranchRight;  // righty
      } else {
        branchFlags[0] = eBranchLeft;  // lefty
      }
    }
  }
  p->setFrame(tmp);
  p->setBranchFlags(branchFlags);
  p->setJointTurns(jointTurns);
  return 0;
}

int Scara::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                     Pose *p, Twist *v) {
  return 0;
}

int Scara::JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                     const Eigen::VectorXd &qddot, Pose *p, Twist *v,
                     Twist *a) {
  return 0;
}

int Scara::CartToJnt(const Pose &pos, Eigen::VectorXd *q) {
  if (!q) {
    std::cout << "input joint angle pointer is null in " << __FUNCTION__
              << ", at line " << __LINE__ << std::endl;
    return -15;
  }
  if (!initialized_) {
    std::cout << "Scara geometric parameters are not initialized"
              << " in function " << __FUNCTION__ << ", line " << __LINE__
              << std::endl;
    return -16;
  }
  Vec p = pos.getTranslation();
  Quaternion quat = pos.getQuaternion();

  std::vector<int> branch;
  pos.getBranchFlags(&branch);
  if (branch.empty()) {
    std::cout << "input pose has no branch information"
              << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    return -49;
  }
  if (q->size() != DoF_) {
    q->resize(DoF_);
  }
  // computer prismatic joint value
  (*q)(2) = p.z() - (d_(0) + d_(1) + d_(2) + d_(3));
  double xy_length = sqrt(p.x() * p.x() + p.y() * p.y());
  if (xy_length > a_(1) + a_(2) ||
      xy_length < fabs(a_(1) - a_(2))) {  // then out of reach
    std::cout << "exception CartToJnt, scara, IK fails due to desire pose out"
              << " of reach in " << __FUNCTION__ << " at line " << __LINE__
              << std::endl;
    return -50;
  }
  // the orientation angle from origin to the origin of tool frame
  // in the XY-plane
  double angle_xy = atan2(p.y(), p.x());
  // angle between upper link and the vector (p.(x), p.(y))
  double offset_angle =
      acos((a_(1) * a_(1) + xy_length * xy_length - a_(2) * a_(2)) /
           (2 * a_(1) * xy_length));

  if (branch[0]) {  // if the configuration is righty
    (*q)(0) = angle_xy - offset_angle;
  } else {
    (*q)(0) = angle_xy + offset_angle;
  }
  (*q)(1) = atan2(p.y() - a_(1) * sin((*q)(0)), p.x() - a_(1) * cos((*q)(0))) -
            (*q)(0);

  // we have to keep all angles calculated from invkin() within -M_Pi and M_Pi
  if ((*q)(0) >= M_PI) {
    (*q)(0) -= 2 * M_PI;
  }
  if ((*q)(0) <= -M_PI) {
    (*q)(0) += 2 * M_PI;
  }

  if ((*q)(1) >= M_PI) {
    (*q)(1) -= 2 * M_PI;
  }
  if ((*q)(1) <= -M_PI) {
    (*q)(1) += 2 * M_PI;
  }

  std::vector<int> jointTurns;
  pos.getBranchFlags(&jointTurns);
  double yaw, pitch, roll;
  if (!quat.GetEulerZYX(&yaw, &pitch, &roll)) {
    std::cout << "exception, IK fails due to Quaternion to YPR error"
              << " in " << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    return -51;
  }
  (*q)(3) = yaw - (*q)(0) - (*q)(1);
  // first convert q->at(3) into [-pi, pi]
  double jointTurns3 = std::floor((*q)(3) / (2 * M_PI));
  double tmp_q = (*q)(3) - jointTurns3 * 2 * M_PI;
  // then make sure [-PI, PI]
  // to have 0 turn here
  if (tmp_q > M_PI) {
    jointTurns3 += 1;
  }
  // q->at(3) -= jointTurns3 * 2 * PI;
  (*q)(3) += (jointTurns[3] - jointTurns3) * 2 * M_PI;
  return 0;
}

int Scara::CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                     Eigen::VectorXd *qdot) {
  return 0;
}

int Scara::CartToJnt(const Pose &p, const Twist &v, const Twist &a,
                     Eigen::VectorXd *q, Eigen::VectorXd *qdot,
                     Eigen::VectorXd *qddot) {
  return 0;
}

}  // namespace kinematics_lib
