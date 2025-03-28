#include "robnux_kdl_common/pose.hpp"

namespace kinematics_lib {

Frame::Frame() {
  // all default values
}

Frame::Frame(const Quaternion& q, const Vec& p) : q_(q), p_(p) {
  r_ = Rotation(q_);
}

Frame::Frame(const Rotation& R, const Vec& V) {
  bool ret = R.GetQuaternion(&q_);
  p_ = V;
  r_ = R;
}

Frame::Frame(const Vec& z, const Vec& p) {
  r_ = Rotation(z);
  r_.GetQuaternion(&q_);
  p_ = p;
}

Frame::Frame(const Vec& V) { p_ = V; }

Frame::Frame(const Rotation& R) {
  bool ret = R.GetQuaternion(&q_);
  r_ = Rotation(q_);
}

Frame::Frame(const Frame& arg) {
  q_ = arg.getQuaternion();
  p_ = arg.getTranslation();
  r_ = arg.getRotation();
}

double Frame::operator()(int i, int j) {
  if ((0 <= i && i <= 3) && (0 <= j && j <= 3)) {
    if (i == 3) {
      if (j == 3) {
        return 1.0;
      } else {
        return 0.0;
      }
    } else if (j == 3) {
      return p_(i);
    } else {
      return r_(i, j);
    }
  } else {
    LOG_ERROR("input index to Frame(i,j) is out of range");
    throw KinematicsException("Frame index is out of range", -1);
  }
}

double Frame::operator()(int i, int j) const {
  if ((0 <= i && i <= 3) && (0 <= j && j <= 3)) {
    if (i == 3) {
      if (j == 3) {
        return 1.0;
      } else {
        return 0.0;
      }
    } else if (j == 3) {
      return p_(i);
    } else {
      return r_(i, j);
    }
  } else {
    LOG_ERROR("input index to Frame(i,j) is out of range");
    throw KinematicsException("Frame index is out of range", -1);
  }
}

Frame Frame::Inverse() const {
  Rotation tmp_r = this->r_.Inverse();
  Vec tmp_v = -(tmp_r * this->p_);
  return Frame(tmp_r, tmp_v);
}

Vec Frame::InverseMulti(const Vec& arg) const {
  Vec v = arg - this->p_;
  return this->r_.Inverse() * v;
}

Wrench Frame::InverseMulti(const Wrench& arg) const {
  Wrench tmp;
  tmp.setForce(this->r_.Inverse() * arg.getForce());
  tmp.setTorque(this->r_.Inverse() *
                (arg.getTorque() - this->p_ * arg.getForce()));
  return tmp;
}

Twist Frame::InverseMulti(const Twist& arg) const {
  Twist tmp;
  tmp.setAngularVel(this->r_.Inverse() * arg.getAngularVel());
  tmp.setLinearVel(this->r_.Inverse() *
                   (arg.getLinearVel() - this->p_ * arg.getAngularVel()));
  return tmp;
}

Frame& Frame::operator=(const Frame& arg) {
  this->q_ = arg.getQuaternion();
  this->p_ = arg.getTranslation();
  this->r_ = arg.getRotation();
  return *this;
}

Vec Frame::operator*(const Vec& arg) const { return this->r_ * arg + this->p_; }

Wrench Frame::operator*(const Wrench& arg) const {
  Wrench tmp;
  tmp.setForce(this->r_ * arg.getForce());
  tmp.setTorque(this->r_ * arg.getTorque() + this->p_ * tmp.getForce());
  return tmp;
}

Twist Frame::operator*(const Twist& arg) const {
  Twist tmp;
  tmp.setAngularVel(this->r_ * arg.getAngularVel());
  tmp.setLinearVel(this->r_ * arg.getLinearVel() +
                   this->p_ * tmp.getAngularVel());
  return tmp;
}

Frame operator*(const Frame& lhs, const Frame& rhs) {
  return Frame(lhs.getRotation() * rhs.getRotation(),
               lhs.getRotation() * rhs.getTranslation() + lhs.getTranslation());
}

Vec operator*(const Frame& lhs, const Vec& rhs) {
  return lhs.getRotation() * rhs + lhs.getTranslation();
}

Frame Frame::Identity() { return Frame(Rotation(), Vec()); }

void Frame::setTranslation(const Vec& trans) { this->p_ = trans; }

void Frame::setQuaternion(const Quaternion& q) {
  this->q_ = q.normalized();
  this->r_ = Rotation(q);
}

void Frame::setRotation(const Rotation& rot) {
  this->r_ = rot;
  rot.GetQuaternion(&this->q_);
}

Eigen::VectorXd Frame::ToEigenVecQuat() {
  Eigen::VectorXd fm(7);
  fm.block(0, 0, 3, 1) = p_.ToEigenVec();
  fm.block(3, 0, 4, 1) = q_.ToEigenVec();
  return fm;
}

Eigen::VectorXd Frame::ToEigenVecEulerZYX() {
  Eigen::VectorXd fm(6);
  fm.block(0, 0, 3, 1) = p_.ToEigenVec();
  fm.block(3, 0, 3, 1) = r_.ToEigenVecZYX();
  return fm;
}

Eigen::VectorXd Frame::ToEigenVecEulerZYZ() {
  Eigen::VectorXd fm(6);
  fm.block(0, 0, 3, 1) = p_.ToEigenVec();
  fm.block(3, 0, 3, 1) = r_.ToEigenVecZYZ();
  return fm;
}

Vec Frame::getTranslation() const { return this->p_; }
Quaternion Frame::getQuaternion() const { return this->q_.normalized(); }
Rotation Frame::getRotation() const { return this->r_; }

Frame Frame::DH_Craig1989(const double a, const double alpha, const double d,
                          const double theta) {
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);
  return Frame(
      Rotation(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca),
      Vec(a, -sa * d, ca * d));
}

Frame Frame::DH_Craig1989_EX(const double a, const double alpha,
                             const double beta, const double d,
                             const double theta) {
  double ca = cos(alpha);
  double sa = sin(alpha);
  double cb = cos(beta);
  double sb = sin(beta);
  double ct = cos(theta);
  double st = sin(theta);
  return Frame(Rotation(cb * ct, -st, sb * ct, sa * sb + ca * cb * st, ca * ct,
                        ca * sb * st - cb * sa, cb * sa * st - ca * sb, sa * ct,
                        ca * cb + sa * sb * st),
               Vec(a, -d * sa, d * ca));
}

Frame Frame::DH(const double a, const double alpha, const double d,
                const double theta) {
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);
  return Frame(
      Rotation(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca),
      Vec(a * ct, a * st, d));
}

bool operator==(const Frame& a, const Frame& b) {
  return (a.getQuaternion() == b.getQuaternion() &&
          a.getTranslation() == b.getTranslation());
}

/* APIs of Pose
 */
Pose::Pose() : numBranchFlags_(4), JntDoF_(10) {
  branchFlags_.resize(numBranchFlags_, eBranchLeft);
  jointTurns_.resize(JntDoF_, 0);
}

Pose::Pose(const std::vector<int>& branchFlags,
           const std::vector<int> jointTurns) {
  branchFlags_ = branchFlags;
  numBranchFlags_ = branchFlags_.size();
  jointTurns_ = jointTurns;
  JntDoF_ = jointTurns.size();
}

Pose::Pose(const Frame& ee_frame) : Frame(ee_frame) {}

Pose::Pose(const Frame& ee_frame, const std::vector<int>& branchFlags,
           const std::vector<int> jointTurns)
    : Frame(ee_frame) {
  branchFlags_ = branchFlags;
  numBranchFlags_ = branchFlags_.size();
  jointTurns_ = jointTurns;
  JntDoF_ = jointTurns.size();
}
/*
 *  set each component of a Pose
 */
void Pose::setFrame(const Frame& ee_frame) {
  this->p_ = ee_frame.getTranslation();
  this->q_ = ee_frame.getQuaternion();
  this->r_ = ee_frame.getRotation();
}

void Pose::setBranchFlags(const std::vector<int>& branchFlags) {
  this->branchFlags_ = branchFlags;
  this->numBranchFlags_ = this->branchFlags_.size();
}

void Pose::setJointTurns(const std::vector<int>& jointTurns) {
  this->jointTurns_ = jointTurns;
  this->JntDoF_ = this->jointTurns_.size();
}

/*
 * read out each component of a Pose, here we use pointer to save the time in
 * copy assignment if we return the data structure directly
 */
bool Pose::getFrame(Frame* fr) const {
  if (!fr) {
    LOG_ERROR("input pointer to Pose->getFrame() is null ");
    return false;
  }
  fr->setTranslation(this->p_);
  fr->setQuaternion(this->q_);
  fr->setRotation(this->r_);
  return true;
}

bool Pose::getBranchFlags(std::vector<int>* branchFlags) const {
  if (!branchFlags) {
    LOG_ERROR("input pointer to Pose->getBranchFlags() is null ");
    return false;
  }
  *branchFlags = this->branchFlags_;
  return true;
}

bool Pose::getJointTurns(std::vector<int>* jointTurns) const {
  if (!jointTurns) {
    LOG_ERROR("input pointer to Pose->getJointTurns() is null ");
    return false;
  }
  *jointTurns = this->jointTurns_;
  return true;
}

Pose& Pose::operator=(const Pose& arg) {
  this->p_ = arg.getTranslation();
  this->q_ = arg.getQuaternion();
  this->r_ = arg.getRotation();
  arg.getBranchFlags(&this->branchFlags_);
  arg.getJointTurns(&this->jointTurns_);
  this->numBranchFlags_ = this->branchFlags_.size();
  this->JntDoF_ = this->jointTurns_.size();
  return *this;
}

Pose::Pose(const Pose& arg) {
  this->p_ = arg.getTranslation();
  this->q_ = arg.getQuaternion();
  this->r_ = arg.getRotation();
  arg.getBranchFlags(&this->branchFlags_);
  arg.getJointTurns(&this->jointTurns_);
  this->numBranchFlags_ = this->branchFlags_.size();
  this->JntDoF_ = this->jointTurns_.size();
}

//! change the world and body frame
Pose operator*(const Frame& lhs, const Pose& rhs) {
  Pose tmp = rhs;
  Frame fr;
  rhs.getFrame(&fr);
  tmp.setFrame(lhs * fr);
  return tmp;
}

Pose operator*(const Pose& lhs, const Frame& rhs) {
  Pose tmp = lhs;
  Frame fr;
  lhs.getFrame(&fr);
  tmp.setFrame(fr * rhs);
  return tmp;
}
Eigen::VectorXd Pose::ToEigenVecPose() {
  Eigen::VectorXd fm(9);
  fm.block(0, 0, 7, 1) = ToEigenVecQuat();
  if (numBranchFlags_ > 0) {
    int flag = branchFlags_[0];
    for (size_t i = 1; i < numBranchFlags_; i++) {
      flag = flag * 2 + branchFlags_[i];
    }
    fm(7) = flag;
  } else {
    fm(7) = 0;
  }
  int turnValue = 0;
  int multi = 1;
  for (size_t i = 0; i < JntDoF_; i++) {
    int turn1 = jointTurns_[i];
    if (turn1 < 0) {
      turn1 += 16;  // when turn < 0, e.g. turn=-1, it is actually -1 + 16 =
                    // 15=F; turn =-2, it is actually -2 + 16= 14=E
    }
    turnValue += turn1 * multi;
    multi *= 16;
  }
  fm(8) = turnValue;
  return fm;
}
Eigen::VectorXd Pose::ToEigenVecPoseFull() {
  Eigen::VectorXd fm(9);
  fm = ToEigenVecPose();
  return fm;
}

refPose::refPose() : Pose() {}
refPose::refPose(const Frame& ee_frame, const Frame& base, const Frame& tool,
                 const std::vector<int>& ikBranchFlags,
                 const std::vector<int> ikJointTurns)
    : Pose(ee_frame, ikBranchFlags, ikJointTurns), base_(base), tool_(tool) {}
refPose::refPose(const refPose& arg) {
  this->p_ = arg.getTranslation();
  this->q_ = arg.getQuaternion();
  this->r_ = arg.getRotation();
  arg.getBranchFlags(&this->branchFlags_);
  arg.getJointTurns(&this->jointTurns_);
  this->numBranchFlags_ = this->branchFlags_.size();
  this->JntDoF_ = this->jointTurns_.size();
  arg.getBase(&this->base_);
  arg.getTool(&this->tool_);
}

void refPose::setDefaultPose(const Pose& arg) {
  base_ = Frame();
  tool_ = Frame();

  this->p_ = arg.getTranslation();
  this->q_ = arg.getQuaternion();
  this->r_ = arg.getRotation();
  arg.getBranchFlags(&this->branchFlags_);
  arg.getJointTurns(&this->jointTurns_);
  this->numBranchFlags_ = this->branchFlags_.size();
  this->JntDoF_ = this->jointTurns_.size();
}

void refPose::setBase(const Frame& base) { base_ = base; }
void refPose::setTool(const Frame& tool) { tool_ = tool; }
bool refPose::getBase(Frame* base) const {
  if (!base) {
    return false;
  }
  *base = base_;
  return true;
}
bool refPose::getTool(Frame* tool) const {
  if (!tool) {
    return false;
  }
  *tool = tool_;
  return true;
}

refPose& refPose::operator=(const refPose& arg) {
  p_ = arg.getTranslation();
  q_ = arg.getQuaternion();
  r_ = arg.getRotation();
  arg.getBranchFlags(&branchFlags_);
  arg.getJointTurns(&jointTurns_);
  numBranchFlags_ = branchFlags_.size();
  JntDoF_ = jointTurns_.size();
  arg.getBase(&this->base_);
  arg.getTool(&this->tool_);
  return *this;
}
bool refPose::getDefaultFrame(Frame* fr) const {
  if (!fr) {
    return false;
  }
  Frame temp(q_, p_);
  Frame temp1 = base_ * temp * tool_.Inverse();
  *fr = temp1;
  return true;
}
bool refPose::getDefaultPose(Pose* pose) const {
  if (!pose) {
    return false;
  }
  Frame temp(q_, p_);
  Frame temp1 = base_ * temp * tool_.Inverse();
  pose->setFrame(temp1);
  pose->setBranchFlags(branchFlags_);
  pose->setJointTurns(jointTurns_);
  return true;
}

bool refPose::getPoseUnderNewRef(const Frame& new_base, const Frame& new_tool,
                                 refPose* rpose) const {
  if (!rpose) {
    return false;
  }
  Frame temp(q_, p_);
  Frame temp1 = new_base.Inverse() * base_ * temp * tool_.Inverse() * new_tool;
  rpose->setFrame(temp1);
  rpose->setBranchFlags(branchFlags_);
  rpose->setJointTurns(jointTurns_);
  rpose->setBase(new_base);
  rpose->setTool(new_tool);
  return true;
}

bool CompareRobotConfigTurn(const std::vector<int>& cfg_1,
                            const std::vector<int>& cfg_2) {
  size_t num_cfg_1 = cfg_1.size();
  size_t num_cfg_2 = cfg_2.size();
  if (num_cfg_1 != num_cfg_2) {
    return false;
  }
  for (size_t i = 0; i < num_cfg_1; i++) {
    if (cfg_1[i] != cfg_2[i]) {
      return false;
    }
  }
  return true;
}

int RobnuxBranch2SingleInt(const std::vector<int>& cfg) {
  int out_int = 0;
  size_t numFlags = cfg.size();
  if (numFlags == 0) {
    out_int = 0;
  } else {
    out_int = cfg[0];  // branch[2] flip/nonflip,  branch[1]  up/down, branch[0]
                       // left/right
    for (size_t i = 1; i < cfg.size(); i++) {
      out_int = out_int * 2.0 + cfg[i];
    }
  }
  return out_int;
}

void SingleInt2RobnuxBranch(const int in_flag, std::vector<int>& cfg) {
  cfg.resize(3, 0);
  int i = 2;
  int flag = in_flag;
  while (flag >= 1 && i >= 0) {
    cfg[i] = flag % 2;
    flag = std::floor(flag / 2.0);
    i = i - 1;
  }
}

int RobnuxTurn2SingleInt(const std::vector<int>& turns) {
  int turnValue = 0;
  int multi = 1;
  for (size_t i = 0; i < turns.size(); i++) {
    int turn1 = turns[i];
    if (turn1 < 0) {
      turn1 += 16;  // when turn < 0, e.g. turn=-1, it is actually -1 + 16 =
                    // 15=F; turn =-2, it is actually -2 + 16= 14=E
    }
    turnValue += turn1 * multi;
    multi *= 16;
  }
  return turnValue;
}

void SingleInt2RobnuxTurn(const int flag, const int DoF,
                          std::vector<int>& ikJointTurns) {
  ikJointTurns.resize(DoF, 0);
  int i = 0;
  int in_flag = flag;
  while (in_flag >= 1 && i < DoF) {
    ikJointTurns[i] = in_flag % 16;
    if (ikJointTurns[i] >= 8) {
      ikJointTurns[i] -= 16;
    }
    in_flag = std::floor(in_flag / 16);
    i++;
  }
}

}  // namespace kinematics_lib
