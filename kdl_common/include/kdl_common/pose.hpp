/*
 * File:   pose.hpp
 * Author: Leon Liu, ROBNUX LLC (leon@robnux.net)
 *
 * Created on April 4, 2021, 5:38 PM
 */

#ifndef POSE_HPP
#define POSE_HPP

#include <vector>

#include "kdl_common/rotation.hpp"
#include "kdl_common/twistWrench.hpp"
#include "kdl_common/vec.hpp"

namespace kinematics_lib {
/*
 * @brief 6-D Frame class that represents the position and
 * orientation of a frame w.r.t. another frame,
 * or represents a rigid motion in SE(3)
 */
class COMMON_API Frame {
 public:
  Frame();
  Frame(const Quaternion& q, const Vec& p);
  //! frame based upon the z axis of a frame, and the origin p
  Frame(const Vec& z, const Vec& p);

  /*
   * Serialize the frame including both position and orientation
   * @Euler=true, output Euler angle, else quaternion
   */
  std::string ToString(bool Euler) const {
    std::ostringstream strs;
    strs << p_.ToString() << q_.ToString(Euler);
    std::string str = strs.str();
    return str;
  }

  Frame(const Rotation& R, const Vec& V);

  //! The rotation matrix defaults to identity
  Frame(const Vec& V);
  //! The position matrix defaults to zero
  Frame(const Rotation& R);

  //! The copy constructor. Normal copy by value semantics.
  Frame(const Frame& arg);

  //!  Treats a frame as a 4x4 matrix and returns element i,j
  //!  Access to elements 0..3,0..3, bounds are checked when
  // NDEBUG is not set
  double operator()(int i, int j);

  //!  Treats a frame as a 4x4 matrix and returns element i,j
  //!    Access to elements 0..3,0..3, bounds are checked when
  // NDEBUG is not set
  double operator()(int i, int j) const;

  // = Inverse
  //! Gives back inverse transformation of a Frame
  Frame Inverse() const;

  //! The same as p2=R.Inverse()*p but more efficient.
  Vec InverseMulti(const Vec& arg) const;

  //! The same as p2=R.Inverse()*p but more efficient.
  Wrench InverseMulti(const Wrench& arg) const;

  //! The same as p2=R.Inverse()*p but more efficient.
  Twist InverseMulti(const Twist& arg) const;

  //! Normal copy-by-value semantics.
  Frame& operator=(const Frame& arg);

  //! Transformation of the base to which the vector
  //! is expressed.
  Vec operator*(const Vec& arg) const;

  //! Transformation of both the force reference point
  //! and of the base to which the wrench is expressed.
  //! look at Rotation*Wrench operator for a transformation
  //! of only the base to which the twist is expressed.
  //!
  //! Complexity : 24M+18A
  Wrench operator*(const Wrench& arg) const;

  //! Transformation of both the velocity reference point
  //! and of the base to which the twist is expressed.
  //! look at Rotation*Twist for a transformation of only the
  //! base to which the twist is expressed.
  //!
  //! Complexity : 24M+18A
  Twist operator*(const Twist& arg) const;

  //! Composition of two frames.
  friend COMMON_API Frame operator*(const Frame& lhs, const Frame& rhs);
  friend COMMON_API Vec operator*(const Frame& lhs, const Vec& rhs);

  //! @return the identity transformation
  static Frame Identity();

  // set the translation vector
  void setTranslation(const Vec& trans);
  // set the orientation matrix/quaternion
  void setQuaternion(const Quaternion& q);
  void setRotation(const Rotation& rot);

  // get the translation and orienation
  Vec getTranslation() const;
  Quaternion getQuaternion() const;
  Rotation getRotation() const;

  /*
  // DH_Craig1989 : constructs a transformationmatrix
  // T_link(i-1)_link(i) with the Denavit-Hartenberg convention as
  // described in the Craigs book: Craig, J. J.,Introduction to
  // Robotics: Mechanics and Control, Addison-Wesley,
  // isbn:0-201-10326-5, 1986.
  //
  // Note that the frame is a redundant way to express the information
  // in the DH-convention.
  // \verbatim
  // Parameters in full : a(i-1),alpha(i-1),d(i),theta(i)
  //
  //  axis i-1 is connected by link i-1 to axis i numbering axis 1
  //  to axis n link 0 (immobile base) to link n
  //
  //  link length a(i-1) length of the mutual perpendicular line
  //  (normal) between the 2 axes.  This normal runs from (i-1) to
  //  (i) axis.
  //
  //  link twist alpha(i-1): construct plane perpendicular to the
  //  normal project axis(i-1) and axis(i) into plane angle from
  //  (i-1) to (i) measured in the direction of the normal
  //
  //  link offset d(i) signed distance between normal (i-1) to (i)
  //  and normal (i) to (i+1) along axis i joint angle theta(i)
  //  signed angle between normal (i-1) to (i) and normal (i) to
  //  (i+1) along axis i
  //
  //   First and last joints : a(0)= a(n) = 0
  //   alpha(0) = alpha(n) = 0
  //
  //   PRISMATIC : theta(1) = 0 d(1) arbitrarily
  //
  //   REVOLUTE : theta(1) arbitrarily d(1) = 0
  //
  //   Not unique : if intersecting joint axis 2 choices for normal
  //   Frame assignment of the DH convention : Z(i-1) follows axis
  //   (i-1) X(i-1) is the normal between axis(i-1) and axis(i)
  //   Y(i-1) follows out of Z(i-1) and X(i-1)
  //
  //     a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
  //     alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
  //     d(i)       = distance from X(i-1) to X(i) along Z(i)
  //     theta(i)   = angle between X(i-1) to X(i) along Z(i)
  // \endverbatim
  */
  static Frame DH_Craig1989(const double a, const double alpha, const double d,
                            const double theta);
  // for parallel joint axes (e.g. z_{i-1} and z_i), besides alpha_i, theta_i,
  // we also models possible error about y axis by beta_i
  static Frame DH_Craig1989_EX(const double a, const double alpha,
                               const double beta, const double d,
                               const double theta);

  // DH : constructs a transformationmatrix T_link(i-1)_link(i) with
  // the Denavit-Hartenberg convention as described in the original
  // publictation: Denavit, J. and Hartenberg, R. S., A kinematic
  // notation for lower-pair mechanisms based on matrices, ASME
  // Journal of Applied Mechanics, 23:215-221, 1955.

  /*  Conventional DH parameters definition:
   *  link numbering and joint number are same as Craig's method
   *  joint i-1 is connected by link i-1 to joint i (also same as
   * Craig's method)
   *  The difference is:
   *  T_i= R_z(theta_i)*T_z(d_i) * R_x(alpha_i) * T_z(a_i)
   * where z_{i-1} is along axis of Joint i, x_{i-1} is the common normal
   * between z_{i-2} to z_{i-1}, y_{i-1}= z_{i-1} cross x_{i-1},
   * so frame i-1 has its origin on the axis of joint i;
   * when i=1, d_0 could be chose as 0.
   *
   * wheren i=n,  frame n could be chosen so that z_n and z_n-1 parallel,
   * so that
   * alpha_n=0
   *
   *
   */
  static Frame DH(const double a, const double alpha, const double d,
                  const double theta);
  //! do not use operator == because the definition of Equal(.,.) is slightly

  friend COMMON_API bool operator==(const Frame& a, const Frame& b);
  /*
   * The following functions are for converting to Eigen vectors
   */
  Eigen::VectorXd ToEigenVecQuat();
  Eigen::VectorXd ToEigenVecEulerZYX();
  Eigen::VectorXd ToEigenVecEulerZYZ();

 protected:
  Quaternion q_;
  Vec p_;
  Rotation r_;
};

/*
 * Robot IK sometimes has 16/8/4 solutions, each solution has 3-4 branch flags,
 * each flag could take eBranchLeft or eBranchRight values
 */
enum COMMON_API ConfigBranchStatus {
  eBranchLeft = 0,  // or also called top in quattro, or elbow up (left)
  eBranchRight = 1  // or also called bottom in quattro, or elbow down (right)
};

/*
 * Pose is a class about End-effector position/orientation/branch w.r.t. default
 * base frame (robot base) and default tool (frange)
 */
class COMMON_API Pose : public Frame {
 public:
  Pose();
  // default Frame, but assigning the known ikBranchFlags and ikJointTurns
  Pose(const std::vector<int>& ikBranchFlags,
       const std::vector<int> ikJointTurns);
  // assigning known Frame, but with default ikBranchFlags and ikJointTurns
  Pose(const Frame& ee_frame);
  Pose(const Frame& ee_frame, const std::vector<int>& ikBranchFlags,
       const std::vector<int> ikJointTurns);
  // copy constructor
  Pose(const Pose& arg);

  /*
   * Serialize the frame including both position and orientation
   * @Euler=true, output Euler angle, else quaternion
   */
  std::string ToString(bool Euler) const {
    std::string str = Frame::ToString(Euler);
    std::ostringstream strs;
    strs << " branch= ";
    for (size_t i = 0; i < numBranchFlags_; i++) {
      strs << branchFlags_[i] << ",";
    }
    strs << "turns = ";
    for (size_t i = 0; i < JntDoF_; i++) {
      strs << jointTurns_[i] << ",";
    }
    str += strs.str();
    return str;
  }

  /*
   *  set each component of a Pose
   */
  void setFrame(const Frame& ee_frame);
  void setBranchFlags(const std::vector<int>& ikBranchFlags);
  void setJointTurns(const std::vector<int>& ikJointTurns);

  /*
   * read out each component of a Pose, here we use pointer
   * to save the time in copy assignment
   * if we return the data structure directly
   */
  bool getFrame(Frame* fr) const;
  bool getBranchFlags(std::vector<int>* ikBranchFlags) const;
  bool getJointTurns(std::vector<int>* ikJointTurns) const;

  //! assignement operators
  Pose& operator=(const Pose& arg);
  //! change the world and body frame
  friend COMMON_API Pose operator*(const Frame& lhs, const Pose& rhs);
  friend COMMON_API Pose operator*(const Pose& lhs, const Frame& rhs);
  Eigen::VectorXd ToEigenVecPose();
  Eigen::VectorXd ToEigenVecPoseFull();

 protected:
  // IK branch flag vector, e.g.
  // ikBranchFlags_[0] means over head or not
  // ikBranchFlags_[1] means elbow up / down
  // ikBranchFalgs_[2] means wrist flips up / down
  std::vector<int> branchFlags_;
  //! number of ik branch flags
  size_t numBranchFlags_;

  // Recall Joint[i] and Joint[i]+360 gives the same end-effector Frame,
  // but they should
  // give rise to different Pose,  [-180 degree , 180degree] is turn 0,
  // [180degree, 540 degree] is turn 1
  // [-540 degree, -180degree] is turn -1
  std::vector<int> jointTurns_;
  //! number of joints, or Joint degrees of freedom
  size_t JntDoF_;
};

/*
 * refPose is a children class of Pose which has non-default base and tool
 * frames
 */
class COMMON_API refPose : public Pose {
 public:
  refPose();
  refPose(const Frame& ee_frame, const Frame& base, const Frame& tool,
          const std::vector<int>& ikBranchFlags,
          const std::vector<int> ikJointTurns);
  // copy constructor
  refPose(const refPose& arg);

  // set default pose, i.e. canonical pose w.r.t. default base and tool
  void setDefaultPose(const Pose& ps);
  /*
   *  set each component of a Pose
   */
  void setBase(const Frame& base);
  void setTool(const Frame& tool);
  /*
   * read out each component of a Pose, here we use pointer
   * to save the time in copy assignment
   * if we return the data structure directly
   */
  bool getBase(Frame* base) const;
  bool getTool(Frame* tool) const;

  //! assignement operators
  refPose& operator=(const refPose& arg);

  // get default pose, i.e., w.r.t. default base and tool
  bool getDefaultFrame(Frame* fr) const;
  bool getDefaultPose(Pose* pose) const;
  // get refPose w.r.t. a given new set of base and tool
  bool getPoseUnderNewRef(const Frame& new_base, const Frame& new_tool,
                          refPose* rpose) const;

  std::string ToString(bool Euler) const {
    std::string str = Pose::ToString(Euler);
    std::ostringstream strs;
    strs << " base= " << base_.ToString(Euler) << ",";
    strs << " tool = " << tool_.ToString(Euler) << std::endl;
    str += strs.str();
    return str;
  }

 protected:
  Frame base_, tool_;
};

COMMON_API bool CompareRobotConfigTurn(const std::vector<int>& cfg_1,
                                       const std::vector<int>& cfg_2);
COMMON_API int RobnuxBranch2SingleInt(const std::vector<int>& cfg);

COMMON_API void SingleInt2RobnuxBranch(const int cfg_data,
                                       std::vector<int>& cfg);

COMMON_API int RobnuxTurn2SingleInt(const std::vector<int>& turns);

COMMON_API void SingleInt2RobnuxTurn(const int turn, const int DoF,
                                     std::vector<int>& ikTurns);

}  // namespace kinematics_lib

#endif /* POSE_HPP */
