#ifndef KINEMATICS_LIB_BASE_KINEMATICS_HPP_
#define KINEMATICS_LIB_BASE_KINEMATICS_HPP_
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kinematics_map/kinematics_exportdecl.h"

using EigenDStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
template <typename MatrixType>
using EigenDRef = Eigen::Ref<MatrixType, 0, EigenDStride>;
template <typename MatrixType>
using EigenDMap = Eigen::Map<MatrixType, 0, EigenDStride>;

namespace kinematics_lib {

class KINEMATICS_API BaseKinematicMap {
 public:
  /*
   *  @brief set (canonical) kinematic parameters
   *  @param parameters, the vector of parameters that characterize
   * the kinematic model
   */
  virtual void SetGeometry(const Eigen::VectorXd &parameters) = 0;

  /*
   * @brief Calculate forward position kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q_in input joint coordinates position
   * @param p: output pose of TCP
   *
   * @return < 0 (error code) when something went wrong
   *
   * Note: this function uses the flag (useCalibrated_) to adopt canonical
   * parameters or calibrated model parameters in FK
   *
   */

  virtual int JntToCart(const Eigen::VectorXd &q, Pose& p) = 0;

  /**
   * @brief Calculate forward position and velocity kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q input joint coordinates
   * @param qdot input joint velocity
   * @param p, v output cartesian position and velocity
   *
   * @return < 0 (error code) if something went wrong
   *
   * Note: this function uses the flag (useCalibrated_) to adopt canonical
   * parameters or calibrated model parameters in FK
   */
  virtual int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                        Pose& p, Twist& v) = 0;

  /*
   * @brief Calculate forward position, velocity and accelaration
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
  virtual int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                        const Eigen::VectorXd& qddot, Pose& p, Twist& v,
                        Twist& a) = 0;

  /*
   * @brief Calculate inverse kinematics based upon End-effector Cartsian pose
   * and the known kinematic parameters. Note: IK solution is based upon
   * the canonical kinematic parameters
   * @param p desired pose of TCP
   * @param q output joint angle vector
   * @return if < 0 something went wrong
   * Note: this function always uses canonical parameters in IK,
   * and the flag (useCalibrated_) will not affect the IK solution
   */
  virtual int CartToJnt(const Pose& p, Eigen::VectorXd& q) = 0;

  /*
     * @brief Calculate inverse kinematics based upon End-effector Cartsian pose and
     * velocity, and the known kinematic parameters. Note: IK solution is based
     * upon the canonical kinematic parameters
     * @param p desired pose of TCP
     * @param v desired twist of TCP
     * @param q output joint angle vector
     * @param qdot output joint velocity vector
     * @return if < 0 something went wrong
     * Note: this function always uses canonical parameters in IK,
     * and the flag (useCalibrated_) will not affect the IK solution
   */
  virtual int CartToJnt(const Pose& p, const Twist& v, Eigen::VectorXd& q,
                        Eigen::VectorXd& qdot) = 0;

  /*
   * @brief inverse kinematics based upon End-effector Cartsian pose,
   * velocity and acceleration, and the known kinematic parameters. Note: IK
   * solution is based upon the canonical kinematic parameters
   * @param p desired pose of TCP
   * @param v desired twist of TCP
   * @param a desired acceleration of TCP
   * @param q output joint angle vector
   * @param qdot output joint velocity vector
   * @param qddot output joint acceleration vector
   * @return if < 0 something went wrong
   * Note: this function always uses canonical parameters in IK,
   * and the flag (useCalibrated_) will not affect the IK solution
  */
  virtual int CartToJnt(const Pose& p, const Twist& v, const Twist& a,
                        Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                        Eigen::VectorXd& qddot) = 0;

  
  /*
    * @brief Calculate the Jacobian matrix for the kinematic model
    * @param kine_para, the vector of parameters that characterize
    * the kinematic model, e.g. DH parameters for serial arm, or geometric parameters for parallel arm
    * @param p, the pose of the end-effector
    * @param Jp_t, the output translational part of the Jacobian matrix
    * @param Jp_r, the output rotational part of the Jacobian matrix
    * @param world_jac, whether we want to cal. the base Jacobian matrix or world jacobian,
    *  default to be base jacobian 
    * @return if < 0 something went wrong
   */
  virtual int CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    bool world_jac = false) = 0;

  /*
   * @brief Calculate the passive joint angles for visualization purpose (e.g.
   * for displaying in rviz)
   @param q, input active joint angle vector
   @paramp, input moving platform pose
   @param qpassive, output joint angle vector for passive joints
   */
  virtual int CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                          Eigen::VectorXd& qpassive) = 0;

  /*
   * @brief get the namelist for joints (including active and passive)
   * for publishing JointState messages used for rviz simulation
   */
  virtual std::vector<std::string>& GetJntNames();

  /*
   * @brief check if the kinematic model is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const;

  /*
   * @brief get number of DoFs
   * @return number of DoFs
   */
  size_t GetDoF() const;
  
  /*
   * @brief get number of active joints
   * @return number of active joints
   */
  size_t GetActDoF() const;
  
  /*
   * @brief whether we shall use calibrated model in all FK
   * @param useCalibrated, true if we want to use calibrated model, false if
   * we want to use canonical model
   */
  void SetUseCalibrated(const bool useCalibrated);

  /*
   * @brief set default base offset
   * @param baseoff, the vector of default base offset parameters
   * w.r.t. the designated world frame (i.e. from default base to world)
   */
  virtual void SetDefaultBaseOff(const EigenDRef<Eigen::VectorXd>& baseoff);
                         
  /*
   * @brief get default base offset
   * @param baseoff, the vector of default base offset parameters
   * w.r.t. the designated world frame (i.e. from default base to world)
   */
  virtual void GetDefaultBaseOff(EigenDRef<Eigen::VectorXd>& baseoff);
                            

  /*
   * @brief get the name of this kinematic model
   * @return name of this kinematic model
   */
  virtual std::string GetName() const = 0;

  virtual ~BaseKinematicMap();

 protected:
  BaseKinematicMap(const size_t active_dof, const size_t dof);

  // number of actuated joints
  size_t A_DoF_;

  // degree of freedom of end-effector
  size_t DoF_;

  //! joint names, for publishing JointState messages used for rviz simulation
  std::vector<std::string> jnt_names_;

  //! whether parameters has been initialized
  bool initialized_;

  //! shall we use caliberated model in all FKs?
  bool useCalibrated_;

  //! is robot (DH) Caliberated
  bool isDHCalibrated_;

  // base offset w.r.t. designated world frame (i.e. from default base to world)
  Frame defaultBaseOff_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_BASE_KINEMATICS_HPP_ */
