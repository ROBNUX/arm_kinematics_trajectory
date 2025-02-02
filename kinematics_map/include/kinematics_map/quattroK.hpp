/*
 * Quatto kinematics: keba convention
 */
#ifndef KINEMATICS_LIB_QUATTROK_HPP_
#define KINEMATICS_LIB_QUATTROK_HPP_
#include "kdl_common/vec.hpp"
#include "kinematics_map/base_kinematics.hpp"
#include "pluginlib/class_list_macros.h"
namespace kinematics_lib {
class KINEMATICS_API QuattroK : public BaseKinematicMap {
 public:
  QuattroK();
  QuattroK(const Eigen::VectorXd &parameters);

  /*
   * set geometry parameters
   * @param parameters, the vector of parameters that characterize
   * the kinematic model
   */
  void SetGeometry(const Eigen::VectorXd &parameters) override;
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
  virtual std::string GetName() const { return std::string("QuattroK"); }

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
  // offset between the tip bar of parallelogram and moving platform hinge joint
  double r1_;
  // moving platform side length
  double m_;

  // initialized flag
  bool initialized_;
  // actual large arm length
  double a_b1_;
  // extra offset angle of large arm because of c1_
  double delta1_;
  // diff between static/moving platform radius
  double diff_radius_;
  // branch flags
  std::vector<int> branchFlags_;
  // turn flags
  std::vector<int> jointTurns_;

  // tmp variables
  std::vector<Vec> tipPoints;
};

}  // namespace kinematics_lib
#endif /* KINEMATICS_LIB_QUATTRO_HPP_ */
