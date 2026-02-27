#ifndef KINEMATICS_LIB_SINGLEAXISMODULE_HPP_
#define KINEMATICS_LIB_SINGLEAXISMODULE_HPP_
#include "robnux_kinematics_map/serialArm.hpp"
#include "simple_motion_logger/Logger.h"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kinematics_lib {
   
class KINEMATICS_API singleAxisModule : public serialArm {
 public:
  singleAxisModule();

  int CartToJnt(const Pose& p, Eigen::VectorXd& q) override; 

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


   //! get name
   virtual std::string GetName() const {
       return std::string("singleAxisModule");
   }
};

}

#endif    /* KINEMATICS_LIB_SINGLEAXISMODULE_HPP_ */
