/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   serialArm.hpp
 * Author: leon
 *
 * Created on February 12, 2022, 5:38 PM
 */

#ifndef ROBNUX_KINEMATICS_MAP_INCLUDE_SERIALARM_HPP_
#define ROBNUX_KINEMATICS_MAP_INCLUDE_SERIALARM_HPP_
#include "robnux_kinematics_map/base_kinematics.hpp"

namespace kinematics_lib {
   
class KINEMATICS_API serialArm : public BaseKinematicMap {
 public:
  serialArm(size_t DoF = 6);

  void SetGeometry(const Eigen::VectorXd& kine_para) override;


  int JntToCart(const Eigen::VectorXd& q,
                       Pose& p) override;
        
  int JntToCart(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& qdot,
                        Pose& p, Twist& v) override;
        
        

  int JntToCart(const Eigen::VectorXd& q,
                         const Eigen::VectorXd& qdot,
                         const Eigen::VectorXd& qddot,
                         Pose& p, Twist& v, Twist& a) override; 
        
        

   int CartToJnt(const Pose &p, Eigen::VectorXd& q) override; 
  
   int CartToJnt(const Pose &p, const Twist &v,
                         Eigen::VectorXd& q,
                         Eigen::VectorXd& qdot) override;
 
   int CartToJnt(const Pose &p, const Twist& v, const Twist& a,
                         Eigen::VectorXd& q,
                         Eigen::VectorXd& qdot,
                         Eigen::VectorXd& qddot) override;

   int CalcJacobian(const Eigen::VectorXd& kine_para,
                    Pose& p,
                    Eigen::MatrixXd& Jp_t,
                    Eigen::MatrixXd& Jp_r,
                    bool world_jac = false) override;

  /*
   * @brief update the DH parameters based upon the joint feedback, for example, 
   * for a revolute joint, the theta will be updated based upon the joint angle feedback, 
   * and for a prismatic joint, the d will be updated based upon the joint position feedback
   * @param q, the input joint feedback
   * @param theta, the output updated theta vector
   * @param d, the output updated d vector
   */ 
   virtual void UpdateDH(const Eigen::VectorXd& q,
                         Eigen::VectorXd& theta,
                         Eigen::VectorXd& d);

   /*
    * @brief update the configuration turn and branch flags based upon the joint feedback, 
    * for example, for a revolute joint, the theta will be updated based upon the joint angle feedback,
    * and the branch flags and turn flags will be updated based upon the joint angle feedback
    * @param theta, the input theta vector (updated based upon joint feedback)
    * @param d, the input d vector (updated based upon joint feedback for prismatic joint)
    * @param branchFlags, the output updated branch flags
    * @param jointTurns, the output updated joint turn flags
    */
   virtual void  UpdateConfigTurn(const Eigen::VectorXd& theta,
                                  const Eigen::VectorXd& d,
                                  std::vector<int>&  branchFlags,
                                  std::vector<int>&  jointTurns) const;
    
    
   int CalcPassive(const Eigen::VectorXd& q,
                   const Pose& p,
                   Eigen::VectorXd& qpassive) override;

  //! a submatrix of the full Jacobian(for calib) that corresponds to 
   // robot usual jacobian
   virtual bool PickSubJacobian(const Eigen::MatrixXd& Jp_t,
                                const Eigen::MatrixXd& Jp_r,
                                Eigen::MatrixXd& Js_t,
                                Eigen::MatrixXd& Js_r,
                                bool reduction=false);  // whether we pick reduction upto subgroup only


   // given trans and euler angle error, pick a sub error vector matching robot model, and return the aboslute error norm
   virtual double  PickCartErr(const Eigen::Vector3d& errT,
                               const Eigen::Vector3d& errR, 
                               Eigen::VectorXd& b,
                               bool reduction = false);

 protected:
  // All following DH parameters are follow Craig's DH convention
  // alpha are angles between z_i and z_{i_1} about x_i
  Eigen::VectorXd alpha_, alpha_c_;
  // a offset vector in Craig DH convention
  Eigen::VectorXd a_, a_c_;
  // d offset vector in Craig DH convention
  Eigen::VectorXd d_, d_c_;
  // theta_ in Craig DH convention
  Eigen::VectorXd theta_, theta_c_;
};

}
#endif /* ROBNUX_KINEMATICS_MAP_INCLUDE_SERIALARM_HPP_ */

