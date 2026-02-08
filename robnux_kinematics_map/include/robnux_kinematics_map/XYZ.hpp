#ifndef KINEMATICS_LIB_XYZGantry_HPP_
#define KINEMATICS_LIB_XYZGantry_HPP_
#include "serialArm.hpp"
#include <common/vec.hpp>
#include <common/rotation.hpp>
#include <Logger/Logger.h>
namespace kinematics_lib {
   
class KINEMATICS_API XYZGantry : public serialArm {
 public:
  //! default constructor
  XYZGantry();
  //! constructor with vector of dh parameters
  //! recall we use Craig's DH convention, but not original DH convention
  //! @param dhPara, the vector of DH parameters
  XYZGantry(const std::vector<double> &kine_para);

  //! canonical IK
  virtual int CartToJnt(const Pose &p, std::vector<double> *q); 

  //! set turn and config flags based upon actual theta and d
  virtual void  UpdateConfigTurn(const std::vector<double> & theta,
                                 const std::vector<double> &d,
                                 std::vector<int>  *branchFlags,
                                 std::vector<int>  *jointTurns) const;
                                  
   //! given Eigen::VectorXd  config,  convert it to std::<int>  branchFlags
   /*
   virtual void ConvertBranchFlag(const Eigen::VectorXd &inFlag,
                                  std::vector<int> *branchFlags) const;
                                  */

   //! a submatrix of the full Jacobian(for calib) that corresponds to 
   // robot usual jacobian
   virtual bool PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r,
                                const bool reduction = false);
   
   // pick translation and rotational jacobian for parameter part.
   virtual bool PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r, 
                                       Eigen::MatrixXd *Js_t1, 
                                       Eigen::MatrixXd *Js_r1,
                                       const bool reduction = false); 
   
   // given trans and euler angle error, pick a sub error vector matching
   // robot model, and return the aboslute error norm
   virtual double  PickCartErr(const Eigen::Vector3d &errT,
                               const Eigen::Vector3d &errR, 
                               Eigen::VectorXd *b,
                               const bool reduction = false);
   
   //! virtual function for updating actual DH parameters based upon joint feedback
   // orig_dh=<alpha_1, alpha_2, .., alpha_k, a_1,...a_k, theta_1,...,theta_k, d_1,...,d_k>
   // jnt angles def. depends on specific robot type
   virtual void UpdateDH(const std::vector<double> &orig_dh,
                         const Eigen::VectorXd &jnt,
                         std::vector<double> *new_dh) const;

   //! virtual function for updating actual DH parameters based upon joint feedback
   // Note: alpha, a, theta, d, beta has their initial values, which will be updated
   // based upon jnt input
   virtual void UpdateDH(const Eigen::VectorXd &jnt,
                         std::vector<double> *theta,
                         std::vector<double> *d
                         ) const;

   int ErrCompensateBase(const Eigen::MatrixXd &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 Eigen::VectorXd *tcp_ur_uncal,
                                 Eigen::VectorXd *tcp_ur_cal
                                 )  override;
   /*
   //! convert jnt dot to those after pitching
   void ForwardQDot(const Eigen::VectorXd &qdot, Eigen::VectorXd *aqdot) const override;
   //! convert "joint space" jnt dot to those physical joint before pitching
   void BackwardQDot(const Eigen::VectorXd &aqdot, Eigen::VectorXd *qdot) const override;
   */ 
   //! get name
   virtual std::string GetName() const {
       return std::string("XYZGantry");
   }
};

}

#endif
