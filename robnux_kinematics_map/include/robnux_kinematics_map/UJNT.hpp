#ifndef KINEMATICS_LIB_UJNT_HPP_
#define KINEMATICS_LIB_UJNT_HPP_
#include "serialArm.hpp"
#include <common/vec.hpp>
#include <common/rotation.hpp>
#include <Logger/Logger.h>
namespace kinematics_lib {
//! UJNT (R-R) modules, which combined with XYZ constitutes the 5-axis module
class KINEMATICS_API UJNT : public serialArm {
 public:
  //! default constructor
  UJNT();
  //! constructor with vector of dh parameters
  //! recall we use Craig's DH convention, but not original DH convention
  //! @param dhPara, the vector of DH parameters
  UJNT(const std::vector<double> &kine_para);

  //! canonical IK
  int CartToJnt(const Pose &p, std::vector<double> *q); 


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
  int JntToCart(const std::vector<double> & q,
                       Pose *p);

  // get specific joint frame (DH frames)
  //bool GetDHFrame(const std::vector<double> & q,
  //                         const int  jnt_index, Frame *fm) const

  int JntToCart(const std::vector<double> &q,
                     const std::vector<double> &qdot,
                     Pose *p, Twist *v);

  int CartToJnt(const Pose &p, const Twist &v,
                     std::vector<double> *q,
                     std::vector<double> *qdot);

  int CalcJacobian(const std::vector<double> &kine_para,
                    Pose *p,
                    Eigen::MatrixXd * Jp_t,
                    Eigen::MatrixXd * Jp_r,
                    const bool reduction = false) override;

  //! check regularity of calibrated DH parameters compared with the original DH parameters
  int CalibSanityCheck(const std::vector<double> &cal_DH);

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

   bool PickRotSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r);
   
   // pick translation and rotational jacobian for parameter part.
   virtual bool PickSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r, 
                                       Eigen::MatrixXd *Js_t1, 
                                       Eigen::MatrixXd *Js_r1,
                                       const bool reduction = false);

    bool PickRotSubJacobianForPara(const Eigen::MatrixXd &Jp_t,
                                       const Eigen::MatrixXd &Jp_r, 
                                       Eigen::MatrixXd *Js_t1, 
                                       Eigen::MatrixXd *Js_r1);
   
   // given trans and euler angle error, pick a sub error vector matching
   // robot model, and return the aboslute error norm
   virtual double  PickCartErr(const Eigen::Vector3d &errT,
                               const Eigen::Vector3d &errR, 
                               Eigen::VectorXd *b,
                               const bool reduction = false);
    
    double  PickRotCartErr(const Eigen::Vector3d &errT,
                               const Eigen::Vector3d &errR, 
                               Eigen::VectorXd *b);
   
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

   //! One important utility function used for ErrCompensationDH above
   int OptimizeJntAfterCalib(const std::vector<double> &init_jnt,
                             const refPose &ps,
                             std::vector<double> *opt_jnt) override;

    //! homotopy algorithm
   int HomotopyAlg1(const std::vector<double> &init_jnt0,
                   const Frame &userTool,
                   std::vector<double> *init_jnt);

   //! get name
   virtual std::string GetName() const {
       return std::string("UJNT");
   }
};

}

#endif
