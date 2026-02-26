#ifndef KINEMATICS_LIB_SIXAXIS_1_HPP_
#define KINEMATICS_LIB_SIXAXIS_1_HPP_
#include "robnux_kinematics_map/serialArm.hpp"
#include "simple_motion_logger/Logger.h"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/vec.hpp"
namespace kinematics_lib {
   
class KINEMATICS_API SixAxis_1 : public serialArm {
 public:
  SixAxis_1();

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
   std::string GetName() const {
       return std::string("SixAxis_1");
   }

   /* old parameters used for reference */
   /*
        double s1; // the distance between the base and trunk centers (the centers should be along the z axis) ;          
        double a1; // the distance (minimum) between the trunk and shoulder joint axis
        double s2;  // the distance (minimum)  between the trunk axis and (upper)arm plane (of motion)   
        double a2; // the length of (upper) arm  which is the distance between shoulder and elbow axis
        double s3; //  the distance between planes of motion of arm and forearm   
        double a3; // the distance between elbow axis and wrist axis (or center) 
        double s4; //  the length of forearm  
        double l; // the length of y joint of the wrist y to last z 
    */    

};

}

#endif




   
        
