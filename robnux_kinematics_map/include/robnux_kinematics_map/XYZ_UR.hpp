#ifndef KINEMATICS_LIB_XYZ_UR_HPP_
#define KINEMATICS_LIB_XYZ_UR_HPP_
#include "robnux_kinematics_map/serialArm.hpp"
#include "simple_motion_logger/Logger.h"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kinematics_map/UJNT.hpp"
#include "robnux_kinematics_map/XYZ.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <memory>
namespace kinematics_lib {
// separate-mode XYZ + UR
class KINEMATICS_API XYZ_UR : public BaseKinematicMap {
 public:
   XYZ_UR();

   void SetGeometry(const Eigen::VectorXd& kine_para) override;
  
   int JntToCart(const Eigen::VectorXd& q,
                       Pose& p) override;

   int CartToJnt(const Pose &p, Eigen::VectorXd& q) override; 

   int JntToCart(const Eigen::VectorXd& q,
                const Eigen::VectorXd& qdot,
                Pose& p, Twist& v) override;
    
   int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                const Eigen::VectorXd& qddot, Pose& p, Twist& v,
                Twist& a) override;
        
   int CartToJnt(const Pose& p, const Twist& v,
                 Eigen::VectorXd& q,
                 Eigen::VectorXd& qdot) override;
  
   int CartToJnt(const Pose &p, const Twist &v, const Twist &a,
                Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                Eigen::VectorXd& qddot) override;

   int CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    bool world_jac = false) override;


  int CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                  Eigen::VectorXd& qpassive) override;

  void SetUseCalibrated(const bool useCalibrated);

  void SetDefaultBaseOff(const Eigen::VectorXd& baseoff);
                         
  void GetDefaultBaseOff(Eigen::VectorXd& baseoff) const;

  void GetPitchAndBacklash(Eigen::VectorXd& pitch,
                           Eigen::VectorXd& backlash) const;

  void SetPitchAndBacklash(const Eigen::VectorXd& pitch,
                           const Eigen::VectorXd& backlash);

  void ResetCalibration() override;

   std::string GetName() const {
       return std::string("XYZ_UR");
   }

 private:
   //! define two robots
   std::shared_ptr<BaseKinematicMap>  UR, XYZ;
};

}

#endif
