#ifndef KINEMATICS_LIB_BASE_KINEMATICS_HPP_
#define KINEMATICS_LIB_BASE_KINEMATICS_HPP_
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <vector>

#include "Logger/Logger.h"
#include "kdl_common/pose.hpp"
#include "kdl_common/vec.hpp"
#include "kinematics_map/kinematics_exportdecl.h"

using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
template <typename MatrixType>
using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;
template <typename MatrixType>
using EigenDMap = Eigen::Map<MatrixType, 0, EigenDStride>;

namespace kinematics_lib {

class KINEMATICS_API BaseKinematicMap {
 public:
  /*
   *  set (canonical) kinematic parameters
   */
  virtual void SetGeometry(const Eigen::VectorXd &parameters) = 0;

  /*
   * Calculate forward position kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q_in input joint coordinates position
   * @param p: pointer to pose
   *
   * @return < 0 (error code) when something went wrong
   *
   * Note: this function uses the flag (useCalibrated_) to adopt canonical
   * parameters or calibrated model parameters in FK
   *
   */

  virtual int JntToCart(const Eigen::VectorXd &q, Pose *p) = 0;

  /**
   * Calculate forward position and velocity kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q_in input joint coordinates
   * @param qdot_in input joint velocity
   * @param p, pdot, output cartesian position and velocity
   *
   * @return < 0 (error code) if something went wrong
   *
   * Note: this function uses the flag (useCalibrated_) to adopt canonical
   * parameters or calibrated model parameters in FK
   */
  virtual int JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                        Pose *p, Twist *v) = 0;

  /*
   * Calculate forward position, velocity and accelaration
   * kinematics, from joint coordinates to cartesian coordinates
   *
   * @param q_in: input joint coordinates
   * @param qdot_in: input joint vel
   * @param qddot_in: input joint acc
   * @param p, pdot, pddot: output cartesian position, velocity
   * and acceleration
   *
   * @return if < 0 something went wrong
   * Note: this function uses the flag (useCalibrated_) to adopt canonical
   * parameters or calibrated model parameters in FK
   */
  virtual int JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                        const Eigen::VectorXd &qddot, Pose *p, Twist *v,
                        Twist *a) = 0;

  /*
   * Calculate inverse kinematics based upon End-effector Cartsian pose
   * and the known kinematic parameters. These parameters could be
   * canonical parameters or after-calibration parameters depending on
   * the flag (useCalibrated_)
   */
  virtual int CartToJnt(const Pose &p, Eigen::VectorXd *q) = 0;

  virtual int CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                        Eigen::VectorXd *qdot) = 0;

  virtual int CartToJnt(const Pose &p, const Twist &v, const Twist &a,
                        Eigen::VectorXd *q, Eigen::VectorXd *qdot,
                        Eigen::VectorXd *qddot) = 0;

  //! obtain the passive joint angles for visualization purpose (e.g.
  // for displaying in rviz)
  //@q, input active joint angle vector
  //@p, input moving platform pose
  //@qpassive, output joint angle vector for passive joints
  virtual int CalcPassive(const Eigen::VectorXd &q, const Pose &p,
                          Eigen::VectorXd *qpassive) = 0;

  //! get the namelist for joints (including active and passive)
  // for publishing joint angles to simulation agent
  virtual std::vector<std::string> *GetJntNames() { return &jnt_names_; }

  //! is parameter initialized
  bool isInitialized() const { return initialized_; }

  //! get number of DoFs
  size_t GetDoF() const { return DoF_; }
  //! get number of active joints
  size_t GetActDoF() const { return A_DoF_; }
  //! get characteristic lenght
  double GetCharLength() const { return char_length_; }
  virtual void SetDefaultBaseOff(const EigenDRef<Eigen::VectorXd> &baseoff,
                                 const EigenDRef<Eigen::VectorXd> &subbaseoff) {
    std::ostringstream strs;
    if (baseoff.size() != 7) {
      strs.str("");
      strs << GetName() << ":"
           << "input of SetDefaultBaseOff = " << baseoff
           << ", has wrong dimension" << std::endl;
      LOG_ERROR(strs);
      return;
    }

    Vec t(baseoff.segment(0, 3));
    Quaternion q(baseoff(3), baseoff(4), baseoff(5), baseoff(6));
    defaultBaseOff_.setTranslation(t);
    defaultBaseOff_.setQuaternion(q);
    strs.str("");
    strs << GetName() << ":"
         << "SetDefaultBaseOff = " << defaultBaseOff_.ToString(true)
         << ", quat=" << defaultBaseOff_.ToString(false)
         << ", in edgen=" << baseoff.transpose() << std::endl;
    LOG_INFO(strs);

    // set subBaseoff
    if (subbaseoff.size() != 7) {
      strs.str("");
      strs << GetName() << ":"
           << "subbaseoff input of SetDefaultBaseOff = " << subbaseoff
           << ", has wrong dimension" << std::endl;
      LOG_ERROR(strs);
      return;
    }

    Vec t1(subbaseoff.segment(0, 3));
    Quaternion q1(subbaseoff(3), subbaseoff(4), subbaseoff(5), subbaseoff(6));
    sub_defaultBaseOff_.setTranslation(t1);
    sub_defaultBaseOff_.setQuaternion(q1);
    strs.str("");
    strs << GetName() << ":"
         << "SetSubDefaultBaseOff = " << sub_defaultBaseOff_.ToString(true)
         << ", quat=" << sub_defaultBaseOff_.ToString(false)
         << ", in edgen=" << subbaseoff.transpose() << std::endl;
    LOG_INFO(strs);
  }

  virtual void SetDefaultBaseOff(const Eigen::VectorXd &baseoff,
                                 const Eigen::VectorXd &subbaseoff) {
    std::ostringstream strs;
    if (baseoff.size() != 7) {
      strs.str("");
      strs << GetName() << ":"
           << "input of SetDefaultBaseOff = " << baseoff
           << ", has wrong dimension" << std::endl;
      LOG_ERROR(strs);
      return;
    }

    Vec t(baseoff.segment(0, 3));
    Quaternion q(baseoff(3), baseoff(4), baseoff(5), baseoff(6));
    defaultBaseOff_.setTranslation(t);
    defaultBaseOff_.setQuaternion(q);
    strs.str("");
    strs << GetName() << ":"
         << "SetDefaultBaseOff = " << defaultBaseOff_.ToString(true)
         << ", quat=" << defaultBaseOff_.ToString(false)
         << ", in edgen=" << baseoff.transpose() << std::endl;
    LOG_INFO(strs);

    // set subBaseoff
    if (subbaseoff.size() != 7) {
      strs.str("");
      strs << GetName() << ":"
           << "subbaseoff input of SetDefaultBaseOff = " << subbaseoff
           << ", has wrong dimension" << std::endl;
      LOG_ERROR(strs);
      return;
    }

    Vec t1(subbaseoff.segment(0, 3));
    Quaternion q1(subbaseoff(3), subbaseoff(4), subbaseoff(5), subbaseoff(6));
    sub_defaultBaseOff_.setTranslation(t1);
    sub_defaultBaseOff_.setQuaternion(q1);
    strs.str("");
    strs << GetName() << ":"
         << "SetSubDefaultBaseOff = " << sub_defaultBaseOff_.ToString(true)
         << ", quat=" << sub_defaultBaseOff_.ToString(false)
         << ", in edgen=" << subbaseoff.transpose() << std::endl;
    LOG_INFO(strs);
  }
  virtual void GetDefaultBaseOff(EigenDRef<Eigen::VectorXd> *baseoff,
                                 EigenDRef<Eigen::VectorXd> *subbaseoff) {
    std::ostringstream strs;
    if (!baseoff || !subbaseoff) {
      strs.str("");
      strs << GetName() << ":"
           << "input pointers to " << __FUNCTION__ << " is null" << std::endl;
      LOG_ERROR(strs);
      return;
    }
    *baseoff = defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getDefaultBaseOff =" << *baseoff << std::endl;
    LOG_INFO(strs);

    *subbaseoff = sub_defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getSubDefaultBaseOff =" << *subbaseoff << std::endl;
    LOG_INFO(strs);
  }

  virtual void GetDefaultBaseOff(Eigen::VectorXd *baseoff,
                                 Eigen::VectorXd *subbaseoff) {
    std::ostringstream strs;
    if (!baseoff || !subbaseoff) {
      strs.str("");
      strs << GetName() << ":"
           << "input pointers to " << __FUNCTION__ << " is null" << std::endl;
      LOG_ERROR(strs);
      return;
    }
    *baseoff = defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getDefaultBaseOff =" << *baseoff << std::endl;
    LOG_INFO(strs);

    *subbaseoff = sub_defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getSubDefaultBaseOff =" << *subbaseoff << std::endl;
    LOG_INFO(strs);
  }

  //! return the name of this robot
  virtual std::string GetName() const = 0;

  virtual ~BaseKinematicMap() {}

 protected:
  BaseKinematicMap(const size_t active_dof, const size_t dof)
      : A_DoF_(active_dof),
        DoF_(dof),
        char_length_(1.0),
        initialized_(false),
        useCalibrated_(false),
        isDHCalibrated_(false) {}
  // number of actuated joints
  size_t A_DoF_;
  // degree of freedom of end-effector
  size_t DoF_;
  // character length, used for converting joint dist to cart dist
  double char_length_;
  //! joint names, for publishing JointState messages used for rviz simulation
  std::vector<std::string> jnt_names_;
  //! whether parameters has been initialized
  bool initialized_;
  //! shall we use caliberated model in all FK and IKs?
  bool useCalibrated_;
  //! is robot (DH) Caliberated
  bool isDHCalibrated_;

  // base offset w.r.t. designated world frame (i.e. from default base to world)
  Frame defaultBaseOff_, sub_defaultBaseOff_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_BASE_KINEMATICS_HPP_ */
