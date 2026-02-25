#ifndef KINEMATICS_LIB_QUATTRO_HPP_
#define KINEMATICS_LIB_QUATTRO_HPP_
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace kinematics_lib {
class KINEMATICS_API Quattro : public BaseKinematicMap {
 public:
  Quattro();

  void SetGeometry(const Eigen::VectorXd& parameters) override;
 
  int JntToCart(const Eigen::VectorXd& q, Pose& p) override;

  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, Pose& p,
                Twist& v) override;
            
  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                const Eigen::VectorXd& qddot, Pose& p, Twist& v,
                Twist& a) override;

  int CartToJnt(const Pose& p, Eigen::VectorXd& q) override;

  int CartToJnt(const Pose& p, const Twist& v, Eigen::VectorXd& q,
                Eigen::VectorXd& qdot) override;

  int CartToJnt(const Pose& p, const Twist& v, const Twist& a,
                Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                Eigen::VectorXd& qddot) override;
  // compute other passive joints
  int CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                  Eigen::VectorXd& qpassive) override;
    
  int CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    const bool reduction =false) override;
  //! get name
  virtual std::string GetName() const { return std::string("Quattro"); }

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
