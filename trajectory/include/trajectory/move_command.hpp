#ifndef MOVE_COMMAND_HPP
#define MOVE_COMMAND_HPP
#include <cmath>
#include <memory>

#include "boost/shared_ptr.hpp"
#include "kdl_common/common_constants.hpp"
#include "kdl_common/pose.hpp"
#include "kinematics_map/base_kinematics.hpp"

namespace kinematics_lib {
/*
 *  for PTP_REL, LIN_REL (i.e. for relative motion command, specifying which
 * frame is reference frame)
 */

enum IPO_MODE {
  ID_WORLD = 0,  // using world frame as reference frame
  ID_BASE = 1,   //  using base frame as reference frame
  ID_TOOL = 2,   // using tool frame as reference frame
};

class FrameData {
 public:
  FrameData() : baseNo_(0), toolNo_(0), ipo_(ID_WORLD) {}
  FrameData(const unsigned int baseID, const unsigned int toolID,
            const IPO_MODE& ipo)
      : baseNo_(baseID), toolNo_(toolID), ipo_(ipo) {}
  FrameData& operator=(const FrameData& arg);
  unsigned int baseNo_;  // base frame ID, 0 means world
  unsigned int toolNo_;  // tool frame ID, 0 mean default tool
  IPO_MODE ipo_;         // IPO mode
};

enum Motion_Command_Type {
  ID_NULL = 0,  // null command
  ID_LINE = 1,
  ID_ARC = 2,
  ID_PTP = 3
};

// in practice, user only set up the percentage of the maximal velocity/acc/jerk
class ProfilePercent {
 public:
  ProfilePercent() : vel_perc_(0), acc_perc_(0), jerk_perc_(0) {}

  ProfilePercent(const int vel_perc, const int acc_perc, const int jerk_perc)
      : vel_perc_(std::max(0, std::min(vel_perc, 100))),
        acc_perc_(std::max(0, std::min(acc_perc, 100))),
        jerk_perc_(std::max(0, std::min(jerk_perc, 100))) {}

  ProfilePercent& operator=(const ProfilePercent& pf) {
    vel_perc_ = pf.vel_perc_;
    acc_perc_ = pf.acc_perc_;
    jerk_perc_ = pf.jerk_perc_;
    return *this;
  }
  int vel_perc_;
  int acc_perc_;
  int jerk_perc_;
};

/*cartesian profile data*/
class ProfileData {
 public:
  ProfileData()
      : max_vel_t_(0),
        max_acc_t_(0),
        max_jerk_t_(0),
        max_vel_r_(0),
        max_acc_r_(0),
        max_jerk_r_(0) {}

  ProfileData(const double max_vel_t, const double max_acc_t,
              const double max_jerk_t, const double max_vel_r,
              const double max_acc_r, const double max_jerk_r)
      : max_vel_t_(max_vel_t),
        max_acc_t_(max_acc_t),
        max_jerk_t_(max_jerk_t),
        max_vel_r_(max_vel_r),
        max_acc_r_(max_acc_r),
        max_jerk_r_(max_jerk_r) {}

  ProfileData& operator=(const ProfileData& pf) {
    if (this != &pf) {
      max_vel_t_ = pf.max_vel_t_;
      max_acc_t_ = pf.max_acc_t_;
      max_jerk_t_ = pf.max_jerk_t_;
      max_vel_r_ = pf.max_vel_r_;
      max_acc_r_ = pf.max_acc_r_;
      max_jerk_r_ = pf.max_jerk_r_;
      return *this;
    }
  }
  // translational maximal speed, acceleration, and jerk
  double max_vel_t_;
  double max_acc_t_;
  double max_jerk_t_;
  // rotational maximal speed, acceleration, and jerk
  double max_vel_r_;
  double max_acc_r_;
  double max_jerk_r_;
};

/*
 * joint space profile, we only need one
 */
class JntProfile {
 public:
  JntProfile() : max_vel_(0), max_acc_(0), max_jerk_(0) {}
  JntProfile(const double max_vel, const double max_acc, const double max_jerk)
      : max_vel_(max_vel), max_acc_(max_acc), max_jerk_(max_jerk) {}
  JntProfile& operator=(const JntProfile& pf) {
    if (this != &pf) {
      max_vel_ = pf.max_vel_;
      max_acc_ = pf.max_acc_;
      max_jerk_ = pf.max_jerk_;
      return *this;
    }
  }
  double max_vel_;
  double max_acc_;
  double max_jerk_;
};

/*
class LocData {
 public:
   LocData(): x_(0), y_(0), z_(0), q0_(1), q1_(0), q2_(0), q3_(0) {
   }
   LocData(const double x, const double y, const double z,
           const double q0, const double q1, const double q2,
           const double q3) : x_(x), y_(y), z_(z), q0_(q0),
           q1_(q1), q2_(q2), q3_(q3) {
   }
   LocData & operator=(const LocData &lc) {
      x_ = lc.x_;
      y_ = lc.y_;
      z_ = lc.z_;
      q0_ = lc.q0_;
      q1_ = lc.q1_;
      q2_ = lc.q2_;
      q3_ = lc.q3_;
      return *this;
  }
   double x_;
   double y_;
   double z_;
   double q0_;
   double q1_;
   double q2_;
   double q3_;
};
*/

/*
 * EulerZYX data
 */
class LocData {
 public:
  LocData()
      : x_(0.0),
        y_(0.0),
        z_(0.0),
        A_(0.0),
        B_(0.0),
        C_(0.0),
        branch_(0),
        turns_(0) {}
  LocData(const double x, const double y, const double z, const double A,
          const double B, const double C, const int branch, const int turns)
      : x_(x),
        y_(y),
        z_(z),
        A_(A),
        B_(B),
        C_(C),
        branch_(branch),
        turns_(turns) {}
  LocData& operator=(const LocData& lc) {
    if (this != &lc) {
      x_ = lc.x_;
      y_ = lc.y_;
      z_ = lc.z_;
      A_ = lc.A_;
      B_ = lc.B_;
      C_ = lc.C_;
      branch_ = lc.branch_;
      turns_ = lc.turns_;
      return *this;
    }
  }
  double x_;
  double y_;
  double z_;
  double A_;  // rotation around x axis, roll
  double B_;  // rotation around y axis, pitch
  double C_;  // rotation around z axis, yaw

  // Mitsubishi definition
  // branch[2] flip/nonflip,  branch[1]  up/down, branch[0] left/right
  int branch_;
  // when turn < 0, e.g. turn=-1, it is actually -1 + 16 = 15=F; turn =-2, it is
  // actually -2 + 16= 14=E
  int turns_;
  // we have API to convert LocData branch and turns into robnux branch and
  // turns
};

/*
 * base class of motion command
 */
class MotionCommand {
 public:
  MotionCommand(const Motion_Command_Type& cmdID,
                const unsigned int percent = 0)
      : cmd_ID_(cmdID),
        approx_percent_(percent),
        initialized_(false),
        totalArcLength_(0) {}

  MotionCommand(const Motion_Command_Type& cmdID, const Pose& start,
                const Pose& goal, const ProfileData& pf,
                const unsigned int percent = 0)
      : cmd_ID_(cmdID),
        start_(start),
        goal_(goal),
        pf_(pf),
        approx_percent_(percent),
        initialized_(false),
        totalArcLength_(0) {}

  //! functions to access the properties of the object
  virtual Motion_Command_Type GetCMDID() const { return cmd_ID_; }
  //! get approximate distance (m for cartesian motion, radian for PTP motion)
  virtual double GetApproxDistance() const {
    return approx_percent_ * this->GetTotalArcLength() / 100.0;
  }
  //! get Cartesian profile
  virtual ProfileData GetProfile() { return this->pf_; }

  // For PTP, we need to be able to set start frame to initialize the Command
  virtual void SetStartPose(const Pose& start) { start_ = start; }
  // For PTP, we need to be able to set start frame to initialize the Command
  virtual void SetGoalPose(const Pose& goal) { goal_ = goal; }

  virtual Pose GetStartPose() const { return start_; }
  virtual Pose GetDestPose() const { return goal_; }
  //! get total arc length to be traveled by this command
  // when this command is blending with the next one, we need to
  // compare the approximate distance with half distant of
  // the next command to make sure we can
  // use balanced blending, e.g., the end approx_dist of this command
  // and the starting approx_dist of the next command is same
  virtual double GetTotalArcLength() const { return this->totalArcLength_; }

  virtual bool isInitialized() { return initialized_; }
  virtual ~MotionCommand() {}

  void setScriptLine(int line) { scriptline_ = line; };
  int getScriptLine() { return scriptline_; };
  unsigned int getPercent() { return approx_percent_; };

 protected:
  // command type
  Motion_Command_Type cmd_ID_;

  // start/goal destination
  Pose start_, goal_;

  // profile (max vel, acc, jerk used in this command)
  ProfileData pf_;
  // approximate pecentage at dest pose of this command
  unsigned int approx_percent_;
  // whether has been initialized_ correctly
  bool initialized_;
  // total arc length of this command
  double totalArcLength_;
  // script line
  int scriptline_ = 0;
};

// linear motion command
class LineMotionCommand : public MotionCommand {
 public:
  LineMotionCommand(const Pose& start, const Pose& goal, const ProfileData& pf,
                    // 0 percent means exact reaching
                    const int percent = 0)
      : MotionCommand(ID_LINE, start, goal, pf,
                      std::max(0, std::min(percent, 100))) {
    Vec sp = start_.getTranslation();
    Vec ep = goal_.getTranslation();
    totalArcLength_ = (ep - sp).Norm();
    if (totalArcLength_ >= MIN_TRANS_DIST) {
      initialized_ = true;
    }
  }
};

// Arc motion
class ArcMotionCommand : public MotionCommand {
 public:
  //! constructor
  // @percent is the percentage of approximation in continuous blending
  ArcMotionCommand(const Pose& start, const Pose& mid, const Pose& end,
                   const ProfileData& pf, const int percent = 0)
      : MotionCommand(ID_ARC, start, end, pf,
                      std::max(0, std::min(percent, 100))) {
    Vec p1 = start.getTranslation();
    Vec p2 = mid.getTranslation();
    Vec p3 = end.getTranslation();

    Vec v1 = p2 - p1;
    Vec v2 = p3 - p1;
    double v1v1, v2v2, v1v2;
    v1v1 = v1.dot(v1);
    v2v2 = v2.dot(v2);
    v1v2 = v1.dot(v2);
    double det = v1v1 * v2v2 - v1v2 * v1v2;
    if (det < EPSILON_CIRC_3P) {
      std::ostringstream strs;
      strs << "three points p1, p2, p3 are singular"
           << " impossible to find a circle passing through them in"
           << __FUNCTION__ << " line " << __LINE__ << std::endl;
      LOG_ERROR(strs);
      return;
    }

    double base = 0.5 / det;
    double k1 = base * v2v2 * (v1v1 - v1v2);
    double k2 = base * v1v1 * (v2v2 - v1v2);
    Vec center_t = p1 + v1 * k1 + v2 * k2;
    center_ = start;
    center_.setTranslation(center_t);  // center
    Vec xAxis_ = p1 - center_t;
    this->radius_ = xAxis_.Norm();

    if (this->radius_ < EPSILON_MIN_RADIUS) {
      std::ostringstream strs;
      strs << "the circle passing through them"
           << " has too small radius" << std::endl;
      LOG_ERROR(strs);
      return;
    }
    xAxis_ = xAxis_ / this->radius_;
    // determine the y-axis is quite tricky, need to consider 3 cases
    Vec v3 = (p2 - center_t) / this->radius_;
    Vec v4 = (p3 - center_t) / this->radius_;
    double cosv3 = v3.dot(xAxis_);
    double cosv4 = v4.dot(xAxis_);

    Vec dy1 = v3 - cosv3 * xAxis_;
    double norm_dy1 = dy1.Norm();

    Vec dy2 = v4 - cosv4 * xAxis_;
    double norm_dy2 = dy2.Norm();

    Vec yAxis_;
    //  case 1: p1,p2 lines on a diameter
    if (norm_dy1 < EPSILON_CIRC_3P) {
      yAxis_ = -dy2 / norm_dy2;
    } else if (norm_dy2 < EPSILON_CIRC_3P) {
      // case 2: p1, p3 lines on a diameter
      yAxis_ = dy1 / norm_dy1;
    } else if (dy1.dot(dy2) < 0) {
      // case 3:  p2, p3 lines two sides of x-axis
      yAxis_ = dy1 / norm_dy1;
    } else if (cosv3 >= cosv4) {
      yAxis_ = dy1 / norm_dy1;
    } else {
      yAxis_ = -dy2 / norm_dy2;
    }
    Vec zAxis_ = xAxis_ * yAxis_;

    center_.setRotation(Rotation(xAxis_, yAxis_, zAxis_));
    this->startAng_ = 0;
    this->endAng_ = std::atan2(v4.dot(yAxis_), cosv4);
    this->totalArcLength_ = fabs(endAng_ - startAng_) * this->radius_;
    // requires minimal length to execute the motion
    if (totalArcLength_ >= MIN_TRANS_DIST) {
      initialized_ = true;
    }
  }

  // get center of the arc
  Frame GetArcCenter() const { return center_; }
  double GetArcRadius() const { return radius_; }
  double GetArcStartAng() const { return startAng_; }
  double GetArcEndAng() const { return endAng_; }
  Pose GetMidPose() const { return mid_; }

 private:
  // center of the circle, center_.getRotation gives the
  // the orientaion of the circle, XY frame align with circle plane, Z normals
  // to circle plane
  Frame center_;
  // radius in meters
  double radius_ = 0;
  // startAng of the arc
  double startAng_ = 0;
  // end Ang of the arc
  double endAng_ = 0;
  // middle pose
  Pose mid_;
};

// Point_TO_Point motion command
class PTPMotionCommand : public MotionCommand {
 public:
  PTPMotionCommand(const Pose& start_pose, const Pose& goal_pose,
                   const std::vector<JntProfile>& pf,
                   const boost::shared_ptr<BaseKinematicMap>& armKM,
                   const int percent = 0);

  ~PTPMotionCommand() {}

  Eigen::VectorXd* GetStartJnts() { return &start_jnt_; }
  Eigen::VectorXd* GetGoalJnts() { return &goal_jnt_; }
  std::vector<JntProfile>* GetJntProfile() { return &jnt_pf_; }
  /*
  virtual double GetTotalArcLength() const {
    return this->totalJntLength_;
  }
   */

  boost::shared_ptr<BaseKinematicMap> GetArmMap() const { return armKM_; }
  // get Robot DoF_
  size_t GetDoF() const { return DoF_; }
  // get robot number of actuators
  size_t GetActDoF() const { return A_DoF_; }

 private:
  // start and goal joint vector
  Eigen::VectorXd start_jnt_, goal_jnt_, diff_jnt_;
  // number of independent joints in the start and goal joint vectors
  size_t DoF_;
  // number of actuators
  size_t A_DoF_;
  // joint space profile
  std::vector<JntProfile> jnt_pf_;
  // kinematic map object, attached to a given robot model
  boost::shared_ptr<BaseKinematicMap> armKM_;
  // total arc length of this command
  double totalJntLength_;
};

}  // namespace kinematics_lib

#endif /*MOVE_COMMAND_HPP  */
