#ifndef KINEMATICS_LIB_SCARA_HPP_
#define KINEMATICS_LIB_SCARA_HPP_
#include "simple_motion_logger/Logger.h"
#include "kdl_common/rotation.hpp"
#include "kdl_common/vec.hpp"
#include "kinematics_map/base_kinematics.hpp"
#include "pluginlib/class_list_macros.h"

namespace kinematics_lib {

class KINEMATICS_API Scara : public BaseKinematicMap {
 public:
  //! default constructor
  Scara();
  //! constructor with vector of dh parameters
  //! recall we use Craig's DH convention, but not original DH convention
  //! @param dhPara, the vector of DH parameters
  Scara(const Eigen::VectorXd &kine_para);
  /*
    const Eigen::VectorXd &alpha,
    const Eigen::VectorXd &a,
    const Eigen::VectorXd &d,
    const Eigen::VectorXd &theta); */

  /*
   * set geometry parameters
   * @param parameters, the vector of parameters that characterize
   * the kinematic model
   */
  void SetGeometry(const Eigen::VectorXd &kine_para) override;
  /*
              const Eigen::VectorXd &alpha,
              const Eigen::VectorXd &a,
              const Eigen::VectorXd &d,
              const Eigen::VectorXd &theta); */
  /*
   * Calculate forward position kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q_in input joint coordinates position
   * @param p: pointer to pose
   *
   * @return < 0 (error code) when something went wrong
   *
   *
   */

  int JntToCart(const Eigen::VectorXd &q, Pose *p) override;

  /**
   * Calculate forward position and velocity kinematics, from
   * joint coordinates to cartesian coordinates.
   *
   * @param q_in input joint coordinates
   * @param qdot_in input joint velocity
   * @param p, v, output cartesian position and velocity
   *
   * @return < 0 (error code) if something went wrong
   *
   *
   */
  int JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, Pose *p,
                Twist *v) override;

  /*
   * Calculate forward position, velocity and accelaration
   * kinematics, from joint coordinates to cartesian coordinates
   *
   * @param q_in: input joint coordinates
   * @param qdot_in: input joint vel
   * @param qddot_in: input joint acc
   * @param p, v, a: output cartesian position, velocity
   * and acceleration
   *
   * @return if < 0 something went wrong
   */
  int JntToCart(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                const Eigen::VectorXd &qddot, Pose *p, Twist *v,
                Twist *a) override;

  int CartToJnt(const Pose &p, Eigen::VectorXd *q) override;

  int CartToJnt(const Pose &p, const Twist &v, Eigen::VectorXd *q,
                Eigen::VectorXd *qdot) override;

  int CartToJnt(const Pose &p, const Twist &v, const Twist &a,
                Eigen::VectorXd *q, Eigen::VectorXd *qdot,
                Eigen::VectorXd *qddot) override;

  //! compute other passive joints
  //! here we don't have passive joints for scara, so we directly return 0
  int CalcPassive(const Eigen::VectorXd &q, const Pose &p,
                  Eigen::VectorXd *qpassive) override {
    // as scara has no passive joint, so directly return 0
    return 0;
  }

  //! get name
  std::string GetName() const override { return std::string("Scara"); }

 private:
  // alpha angle vector in Craig DH convention, alpha_c_, parameters after
  // calibration
  Eigen::VectorXd alpha_, alpha_c_;
  // a offset vector in Craig DH convention
  Eigen::VectorXd a_, a_c_;
  // d offset vector in Craig DH convention
  Eigen::VectorXd d_, d_c_;
  // theta_ in Craig DH convention
  Eigen::VectorXd theta_, theta_c_;
  // initialized flag
  bool initialized_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_SCARA_HPP_ */
