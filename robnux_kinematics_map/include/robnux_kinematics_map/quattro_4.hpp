#ifndef KINEMATICS_LIB_QUATTRO_4_HPP_
#define KINEMATICS_LIB_QUATTRO_4_HPP_
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
//#include <Eigen/Core >
#include <eigen3/Eigen/Eigenvalues>

#include "pluginlib/class_list_macros.h"

using namespace Eigen;

namespace kinematics_lib {
/*
 * 4-DoF Quattro robot: this robot has 4 actuators (motors), and 4-DoF
 * end-effector with XYZ translational DoFs and yaw DoF
 */
class KINEMATICS_API Quattro_4 : public BaseKinematicMap {
 public:
  Quattro_4();
  Quattro_4(const Eigen::VectorXd &parameters);

  /*
   * set geometry parameters
   * @param parameters, the vector of parameters that characterize
   * the kinematic model
   */
  void SetGeometry(const Eigen::VectorXd &parameters) override;
  //                 const std::vector<int>  &branchFlags);
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
  // compute other passive joints
  int CalcPassive(const Eigen::VectorXd &q, const Pose &p,
                  Eigen::VectorXd *qpassive) override;

  //! get branchFlags_
  std::vector<int> GetBranchFlags() const { return branchFlags_; }
  //! get turns
  std::vector<int> GetJntTurns() const { return jointTurns_; }
  //! get name
  virtual std::string GetName() const { return std::string("Quattro_4"); }

 private:
  // static platform radius
  double R1_;
  // offset angle of first arm plane w.r.t. world x-axis
  double alpha_;
  // large arm length
  double b1_;
  // elbow joint axis offset w.r.t. large arm central plane
  double c1_;
  // length of parallelogram
  double d1_;
  // width of parallelogram
  double h_;
  // moving platform side length: the length of
  // two rigid links connected to limb (2,3) or (4,1)
  double m_;
  // offset of the center of moving platform link (1,2) w.r.t tip of lower
  // limb 1 (bottom of parallelogram),
  // and w.r.t. tip of lower limb 2 (bottom of parallelogram)
  double v1x_,
      v1y_;  // because moving platform is always horizontal, so no z component
  double v2x_,
      v2y_;  // because moving platform is always horizontal, so no z component
  // v3 = [-v1x_, -v1y_],  v4 = [-v2x_, -v2y_]

  // initialized flag
  bool initialized_;
  // actual large arm length
  double a_b1_;
  // extra offset angle of large arm because of c1_
  double delta1_;
  // diff between static/moving platform radius
  // double diff_radius_;
  // branch flags
  // there are two branch flags here
  // it picks which root one-by-one until finding the correct solution in
  // possibly up to 16 roots, in fact, we have to go through all roots to verify
  // if the FK is success or not branchFlags_[0] takes value in (0,3) (because
  // each root of polynormial, there could be 4 solutions to sinA=a, sinB=b if
  // branchFlags_[1] = 0, then we choose (asin, asin), if branchFlags_[1] = 1,
  // then we choose (PI-asin, asin) if branchFlags_[1] = 2, then we choose
  // (asin, PI-asin) if branchFlags_[1] = 3, then we choose (PI-asin, PI-asin)
  std::vector<int> branchFlags_;
  // turn flags
  std::vector<int> jointTurns_;

  // tmp variables
  std::vector<Vec> tipPoints;
  std::vector<Vec> movingPlatformOffsets;
  // 8th polynomial equation
  // a0x^8 + a1x^7 + a2x^6 + a7 x^1 +a8 = 0;
  std::vector<double> polyCoef_;

  //! solve roots of 8th order polynomial
  int SolvePolyRoots(std::vector<double> *solution);
  //! compute the root of this 8th order polynomial
  //@B1,B2,B3,B4 are tip points of upper limbs while applying offsets of
  // the fixed vectors v_i of moving platform
  //@lower_arm_length: the length of the passive parallelogram
  //@move_platform_length: the length of the link linking lower limb tips (2,3)
  // or tips (1,4)
  int FindRootsWithEigen(const Vec &B1, const Vec &B2, const Vec &B3,
                         const Vec &B4, const double &lower_arm_length,
                         const double &move_platform_length, Pose *pos);
};

}  // namespace kinematics_lib
#endif /* KINEMATICS_LIB_QUATTRO_4_HPP_ */
