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
        
   int CartToJnt(const Pose& p, const Twist& v,
                 Eigen::VectorXd& q,
                 Eigen::VectorXd& qdot) override;
  


   int CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    bool world_jac = false) override;

  void  UpdateConfigTurn(const Eigen::VectorXd& theta,
                         const Eigen::VectorXd& d,
                        std::vector<int>&  branchFlags,
                        std::vector<int>&  jointTurns) const;
                                  
   bool PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                        const Eigen::MatrixXd& Jp_r,
                        Eigen::MatrixXd& Js_t,
                        Eigen::MatrixXd& Js_r,
                        const bool reduction = false) override;

   double  PickCartErr(const Eigen::Vector3d& errT,
                      const Eigen::Vector3d& errR, 
                      Eigen::VectorXd& b,
                      bool reduction = false) override;
   
   void UpdateDH(const Eigen::VectorXd& orig_dh,
                 const Eigen::VectorXd& jnt,
                 Eigen::VectorXd& new_dh) const override;

   void UpdateDH(const Eigen::VectorXd& jnt,
                 Eigen::VectorXd& theta,
                 Eigen::VectorXd& d) const override;

   std::string GetName() const {
       return std::string("XYZ_UR");
   }

 private:
   //! define two robots
   std::shared_ptr<BaseKinematicMap>  UR, XYZ;
};

}

#endif
