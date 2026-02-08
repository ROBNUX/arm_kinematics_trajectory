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
  serialArm();
  serialArm(const Eigen::VectorXd &kine_para);

  void SetGeometry(const Eigen::VectorXd& kine_para);


  int JntToCart(const std::vector<double> & q,
                       Posep) override;
        
  int JntToCart(const std::vector<double> &q,
                        const std::vector<double> &qdot,
                        Pose& p, Twist& v) override;
        
        

  int JntToCart(const std::vector<double> &q,
                         const std::vector<double> &qdot,
                         const std::vector<double> &qddot,
                         Pose& p, Twist& v, Twist& a) override; 
        
        

   int CartToJnt(const Pose &p, std::vector<double> *q) {
       // no generalized IK, so we return error here to be overrided
       // by children class
       return -1001;
   }
  
   virtual int CartToJnt(const Pose &p, const Twist &v,
                         std::vector<double> *q,
                         std::vector<double> *qdot);
 
   virtual int CartToJnt(const Pose &p, const Twist& v, const Twist& a,
                         std::vector<double> *q,
                         std::vector<double> *qdot,
                         std::vector<double> *qddot);

   int CalcJacobian(const std::vector<double> &kine_para,
                    Pose *p,
                    Eigen::MatrixXd * Jp_t,
                    Eigen::MatrixXd * Jp_r,
                    const bool reduction =false) override;

   //! compute other passive joints
   //! here we don't have passive joints for scara, so we directly return 0
   virtual int CalcPassive(const std::vector<double> &q,
                           const Pose &p,
                           std::vector<double> *qpassive) {
       // as scara has no passive joint, so directly return 0
       return 0;
   }

   //! convert qdot to actual qdot in joint space
   // virtual void ForwardQDot(const Eigen::VectorXd &qdot, Eigen::VectorXd *aqdot) const;
   //! convert actual qdot to qdot in physical joint space
   // virtual void BackwardQDot(const Eigen::VectorXd &aqdot, Eigen::VectorXd *qdot) const;
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

   virtual void UpdateDH(const Eigen::VectorXd &delta_p,
                         Eigen::VectorXd * base,
                         std::vector<double> *alpha,
                         std::vector<double> *a,
                         std::vector<double> *theta,
                         std::vector<double> *d,
                         std::vector<double> *beta,
                         Eigen::Vector3d *tool,
                         EigenDRef<Eigen::Matrix3d>  *laser2CartMap=NULL);
    
    virtual void UpdateDH2(const Eigen::VectorXd &delta_p,
                         Eigen::VectorXd * base,
                         std::vector<double> *alpha,
                         std::vector<double> *a,
                         std::vector<double> *theta,
                         std::vector<double> *d,
                         std::vector<double> *beta,
                         Eigen::Vector3d *tool,
                         EigenDRef<Eigen::Matrix3d>  *laser2CartMap_l=NULL,
                         EigenDRef<Eigen::Matrix3d>  *laser2CartMap_r=NULL
                         );

   //! set turn and config flags based upon actual theta and d
   virtual void  UpdateConfigTurn(const std::vector<double> & theta,
                                  const std::vector<double> &d,
                                  std::vector<int>  *branchFlags,
                                  std::vector<int>  *jointTurns) const;
                                  
   //! given size_t  inconfig,  convert it to std::<int>  branchFlags
   virtual void ConvertBranchFlag(const size_t inFlag,
                                   std::vector<int> *branchFlags) const;
   //! given size_t  inconfig,  convert it to std::<int>  ikTurnsFlags
   virtual void ConvertMultiTurnFlag(const size_t inFlag,
                                   std::vector<int> *ikTurnsFlags)  const;

   //! a submatrix of the full Jacobian(for calib) that corresponds to 
   // robot usual jacobian
   virtual bool PickSubJacobian(const Eigen::MatrixXd  &Jp_t,
                                const Eigen::MatrixXd &Jp_r,
                                Eigen::MatrixXd *Js_t,
                                Eigen::MatrixXd *Js_r,
                                const bool reduction=false);  // whether we pick reduction upto subgroup only
   
   // pick translation and rotational jacobian for parameter part.
   virtual bool PickSubJacobianForPara(const Eigen::MatrixXd &Jt_p,
                                       const Eigen::MatrixXd &Jp_r, 
                                       Eigen::MatrixXd *Js_t1, 
                                       Eigen::MatrixXd *Js_r1,
                                       const bool reduction = false); 
   
   // given trans and euler angle error, pick a sub error vector matching robot model, and return the aboslute error norm
   virtual double  PickCartErr(const Eigen::Vector3d &errT,
                                           const Eigen::Vector3d &errR, 
                                           Eigen::VectorXd *b,
                                           const bool reduction = false);

   //! get parameter set after caliberation
   bool GetCalibParamSet(EigenDRef<Eigen::VectorXd> *cal_DH) override;
   bool GetCalibParamSet(Eigen::VectorXd *cal_DH) override;

   //! load calibrated set of parameters from file
   bool LoadCalibParamSet(const std::vector<double> &cal_DH) override;

   //! homotopy algorithm
   int HomotopyAlg(const std::vector<double> &init_jnt0,
                   const Vec &toolOffset,
                   std::vector<double> *init_jnt) override;
   
   //! check regularity of calibrated DH parameters compared with the original DH parameters
   int CalibSanityCheck(const std::vector<double> &cal_DH);

   //! verify if calibration is good or not, usually using tested selected
   // paths
   virtual Eigen::VectorXd VerifyCalibrate(
              const EigenDRef<Eigen::VectorXd> &base, // base offset used in measuring
              const EigenDRef<Eigen::VectorXd> &tool,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              const EigenDRef<Eigen::Matrix3d> &laserMat,
              EigenDRef<Eigen::MatrixXd> *calib_cart,
              EigenDRef<Eigen::MatrixXd> *orig_err,
              EigenDRef<Eigen::MatrixXd> *calib_err);
    
    virtual Eigen::VectorXd VerifyCalibrateComb(
              const EigenDRef<Eigen::VectorXd> &base, // base offset used in measuring
              const EigenDRef<Eigen::VectorXd> &tool,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              const int second_start_index,
              const EigenDRef<Eigen::Matrix3d> &laserMat_l,   // 3 by 3 laser measure matrix
              const EigenDRef<Eigen::Matrix3d> &laserMat_r,   // 3 by 3 laser measure matrix
              EigenDRef<Eigen::MatrixXd> *calib_cart,
              EigenDRef<Eigen::MatrixXd> *orig_err,
              EigenDRef<Eigen::MatrixXd> *calib_err);

   //! for calibration purpose using an array of lasers
   //@init_base_offset: initial base offset (w.r.t. which measureMents is reference to)
   //@init_tool_offset:  the initial tool offset (or known/or estimated tool)
   //@laser2CartMap: 3 by 3 transformation matrix from laser displacement vec
   // to Cartesian displacement vec
   //@cart_measure, an array of 3d Vecs, that represents the measured cart position
   // of tool tip (w.r.t. init_tool_offset)
   //@qd_array: an array of joint angle vectors
   //@laser_measure: an array of 3d vecs, that represents the measured readings of a number of lasers
   // Note: cart_measure, qa_array, laser_measure should all have same number of
   // columns
   //@final_base_offset, we might choose to optimize final_base_offset
   //@final_tool_offset, we might choose to optimize final_tool_offset
   double CalibrateLaserMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              EigenDRef<Eigen::Matrix3d> *finalLaser2CartMap,
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) override;

       //! for calibration purpose using an array of lasers
   //! Note: this function differs from the above regular CalibrateLaserMethod in that
   // it has two sets of data, cart1:cart2, jnt1:jnt2, laser1:laser2, has two Laser2CartMap,
   // and two sets of data is separated by a data indices: secondset_start_index
   // i.e., [0:secondset_start_index-1] is first set, while [secondset_start_index:end]
   // for the second set
   //@init_base_offset: initial base offset (w.r.t. which measureMents is reference to)
   //@init_tool_offset:  the initial tool offset (or known/or estimated tool)
   //@laser2CartMap: 3 by 3 transformation matrix from laser displacement vec
   // to Cartesian displacement vec
   //@cart_measure, an array of Cartesian vectors, that represents the measured cart position
   // of tool tip (w.r.t. init_tool_offset)
   //@qd_array: an array of joint angle vectors
   //@laser_measure: an array of 3d vecs, that represents the measured readings of a number of lasers
   // Note: cart_measure, qa_array, laser_measure should all have same number of
   // columns
   //@final_base_offset, we might choose to optimize final_base_offset
   //@final_tool_offset, we might choose to optimize final_tool_offset
   //@output: matching error after compensation for extra samples not used for calib
   virtual double CalibrateLaserCombMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap_l,
              const EigenDRef<Eigen::Matrix3d> &laser2CartMap_r,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              const EigenDRef<Eigen::MatrixXd> &laser_measure,
              const int secondset_start_index,
              EigenDRef<Eigen::Matrix3d> *finalLaser2CartMap_l,
              EigenDRef<Eigen::Matrix3d> *finalLaser2CartMap_r,
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) override;

   //! calibrate robot parameters with direct measuring method(if such measuring
   // is feasible (still considering sensor random fixturing, we have to use rel. dist. cal.)
   double CalibrateDirectMethod(
              const Eigen::VectorXd &init_base_offset,
              const Eigen::VectorXd &init_tool_offset,
              const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
              const EigenDRef<Eigen::MatrixXd> &measureMents,
              const EigenDRef<Eigen::MatrixXd> &qa_array,
              Eigen::VectorXd *final_base_offset,
              Eigen::VectorXd *final_tool_offset) override;

   //! calibrate the DH parameters using plane probing method
   double CalibratePlaneMethod(
       //const EigenDRef<Eigen::Vector3d> &init_plane,
       const Eigen::VectorXd &init_base_offset,
       const Eigen::VectorXd & init_tool_offset,
       const EigenDRef<Eigen::MatrixXd> &qa_measure,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_array,
       //EigenDRef<Eigen::Vector3d> *final_plane,
       Eigen::VectorXd *final_base_offset,
       Eigen::VectorXd *final_tool_offset) override;

   //! get joint coordinates based upon pose, note: here input pose is coming form python
   // so it will convert to kinematics_lib::Pose, and jnt is also a Eigen vector
   bool GetJntFromPose(const Eigen::VectorXd &pose,
                       Eigen::VectorXd *jnt);

     //! get Cartesian coordinates based upon joint feedback, for python use
   bool GetPoseFromJnt(const Eigen::VectorXd &jnt,
                       Eigen::VectorXd *pose);

    // get joint frame at Jnt[jnt_index]
    bool GetDHFrame(const std::vector<double> & q,
                    const int  jnt_index, Frame *fm) const;

   double CalibrateLaserCoplanar(
       const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
       const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
       const EigenDRef<Eigen::MatrixXd> &laserMat_x,
       const EigenDRef<Eigen::MatrixXd> &laserMat_y,
       const EigenDRef<Eigen::MatrixXd> &laserMat_z,
       const EigenDRef<Eigen::Vector3d> &laser_scale) override;

  double LaserCalibrateCoplanar(
       const Eigen::MatrixXd &cart_measure_x,  // cartesian coordinates reported from robot
       const Eigen::MatrixXd &cart_measure_y,
       const Eigen::MatrixXd &cart_measure_z,
       const Eigen::MatrixXd &laserMat_x,
       const Eigen::MatrixXd &laserMat_y,
       const Eigen::MatrixXd &laserMat_z,
       const Eigen::Vector3d &laser_scale) override;

    
    double CalibrateLaserOrientation(
       const EigenDRef<Eigen::MatrixXd> &jnt_measure, //  | DoF * 1 | DoF * 1 | DoF * 1 | DoF * 1 | ..... Every surface: 1 column points
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // | 8 * 3| 8 * 3| 8 * 3| 8 * 3|  .... every surface: 3 pts
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,   // not used
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const double max_laser_dist,
       const int numPtsInEachPlane, // number of points in each plane
       const std::vector<int> surfaceArrays) override;    // [1.... surfaceArrays[0]] using same plane
                                                          // [surfaceArrays[0]+, ..., surfaceArrays[1]], next plane
    
    double LaserCalibrateOrientation(
       const Eigen::MatrixXd &jnt_measure, //  | DoF * 1 | DoF * 1 | DoF * 1 | DoF * 1 | ..... Every surface: 1 column points
       const Eigen::MatrixXd &cart_measure,  // | 8 * 3| 8 * 3| 8 * 3| 8 * 3|  .... every surface: 3 pts
       const Eigen::VectorXd &laserMat_z_measure,   // not used
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const double max_laser_dist,
       const int numPtsInEachPlane, // number of points in each plane
       const std::vector<int> surfaceArrays) override; 

   //! jnt_x is 3 columns, which forms the largest triangle in the laser detected x-plane
   //! jnt_y is 3 columns, which forms the largest triangle in the laser detected y-plane
   //! jnt_z is 3 columns, which forms the largest triangle in the laser detected z-plane 
   //! note this function will be combind with CalibrateLaserCoplanar 
   /*
   double CalibrateSurfNormal(
       const EigenDRef<Eigen::MatrixXd> &jnt_x,  // vertical between surface normals
       const EigenDRef<Eigen::MatrixXd> &jnt_y,
       const EigenDRef<Eigen::MatrixXd> &jnt_z);
    */

   // the following is the single laser version of the above function
   double CalibrateLaserCoplanarSing(
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::VectorXd> &laser_measure,
       const double laser_scale,
       const int axisChannel) override;

   Eigen::VectorXd VerifyProbing(
       const EigenDRef<Eigen::VectorXd> & tool_offset,
       const EigenDRef<Eigen::MatrixXd> &qa_measure, 
       const EigenDRef<Eigen::MatrixXd> &cart_array);

   Eigen::VectorXd VerifyCoplanar(
               const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
               const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
               const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
               const EigenDRef<Eigen::MatrixXd> &laserMat_x,
               const EigenDRef<Eigen::MatrixXd> &laserMat_y,
               const EigenDRef<Eigen::MatrixXd> &laserMat_z,
               const EigenDRef<Eigen::Vector3d> &laser_scale) override;

    Eigen::VectorXd VerifyCoplanarSing(
       const EigenDRef<Eigen::MatrixXd> &cart_measure,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::VectorXd> &laser_measure,
       const double laser_scale,
       const int axisChannel) override;

   int ErrCompensateBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,
                         const Eigen::VectorXd &orig_tool,
                         EigenDRef<Eigen::VectorXd> *comp_base_uncal,
                         EigenDRef<Eigen::VectorXd> *comp_base,
                         EigenDRef<Eigen::MatrixXd> *origCart,
                         EigenDRef<Eigen::MatrixXd> *compCart,
                         EigenDRef<Eigen::VectorXd> *relq_cano,    // need to find out the desired orientation of laser in canonical model w.r.t. workpiece frame
                         EigenDRef<Eigen::VectorXd> *relq_calib    // need to find out the desired orientaiton of laser in calibrated model w.r.t. ...
                         ) override;
   
  /*
   * single-point cartesian error compensation  
   * @p desired pose including base/tool and cartesian data
   * @cp modified point
   */ 
   int ErrCompCart(const refPose &p,
                   const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
                   refPose *cp);

   /*
    * single-point joint error compensation
    * @p desired pose including base/tool and cartesian data
    * @cq modified joint angles
    */ 
   int ErrCompJnt(const refPose &p, std::vector<double> *cq);

   //! use calibrated model to compensate input trajectories
   int  ErrCompensationDH(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd &bestTool,
        const EigenDRef<Eigen::MatrixXd> &d_traj,
        EigenDRef<Eigen::MatrixXd> *md_traj) override;

   int  ErrCompensationDH(
        const Frame &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Frame &origBase,  // uncalibrated base that filtered path references to
        const Frame &bestTool,
        const Pose &d_traj,
        Pose *md_traj,
        Eigen::VectorXd *d_j_traj,
        Eigen::VectorXd *md_j_traj,
        Pose *a_traj) override;
   
    /*
    * Trajectory correction through calibrated DH models (for testing purpose)
    * @d_traj: input desired trajectory that robot tries to achieve, NOTE:
    * d_traj is w.r.t. default base and tool
    * @d_j_traj: supposed desired joint_traj in the canonical model
    * @md_traj:modified desired trajectory that compensates robot model errors
    * @md_j_traj: modified desired joint traj 
    * @a_traj: actual Cartesian trajectory with md_j_traj as joint input, 
    * and calibrated parameters as model
    */
   virtual int  ErrCompensationDH(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd &bestTool,
        const EigenDRef<Eigen::MatrixXd> &d_traj,   // desired cartesian traj under canonical model (i.e. robot original traj)
        EigenDRef<Eigen::MatrixXd> *md_traj,  // compensated cartesian traj
        EigenDRef<Eigen::MatrixXd> *d_j_traj,  // desired joint traj (corresponds to d_traj in Cartesian space)
        EigenDRef<Eigen::MatrixXd> *md_j_traj,  //  modified joint traj (corresponds to compensated traj)
        EigenDRef<Eigen::MatrixXd> *a_traj);    // actual cartesian traj under calibrated model 


   virtual double CalibrateTool(
        const EigenDRef<Eigen::MatrixXd> &qa_array,   // joint array measurement
        const EigenDRef<Eigen::VectorXd> &measureMents,  // distance sensor measurement
        const EigenDRef<Eigen::Vector3d> &init_normal,   // initial normal of sensor top surface
        EigenDRef<Eigen::VectorXd> *final_tool_offset      // output of the tool offset
        );

   virtual double CalibrateLaserTCP(
      // first 2 sets of 4-pt edge scan to determine the tcp orientation
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_4pt_low,
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_4pt_high,
      // another 3 set of 4-pt edge scan with different orientation
      const EigenDRef<Eigen::MatrixXd> &jnt_mes_12pt,
      EigenDRef<Eigen::VectorXd> *final_tool_offset      // output of the tool offset
      );

   //! One important utility function used for ErrCompensationDH above
   int OptimizeJntAfterCalib(const std::vector<double> &init_jnt,
                             const refPose &ps,
                             std::vector<double> *opt_jnt) override;
   

   
   virtual std::string GetName() const override {
       return std::string("serial arm");
   }

   bool resetCalibration();

 protected:
  // alpha angle vector in Craig DH convention, alpha_c_, parameters after
  // calibration
  // Note: for scara robot d_[2], theta_[0], theta_[1], theta_[3]
  // are only offset, the actual joints are feedback joint angles from motors
  // plus the above 4 offsets: e.g.,
  //  theta[0] = actual_j[0] = theta_[0] + jnt_feedback[0]
  //  theta[1] = actual_j[1] = theta_[1] + jnt_feedback[1]
  //  d[2] = actual_j[2] = d_[2] + jnt_feedback[2]
  //  theta[3] = actual_j[3] = theta_[3] + jnt_feedback[3]
  // note: parameters to be calibrated include
  // alpha[0], alpha[1], alpha[2], alpha[3] (are actual values)
  // a[0], a[1], a[2], a[3] (are actual values)
  //  d_[0], d_[1], d_[2], d_[3] (d_[2] is initial value of prismatic joint)
  // theta_[0], theta_[1], theta_[2], theta_[3] (theta_[0], theta_[1],
  // theta_[3] are initial values of angular values) 

  // alpha are angles between z_i and z_{i_1} about x_i
  std::vector<double> alpha_, alpha_c_;
  // new added:  beta_ are extra angles that models the rotation of z_i r.t. z_{i-1} about y_i
  std::vector<double> beta_, beta_c_;
  // a offset vector in Craig DH convention
  std::vector<double> a_, a_c_;
  // d offset vector in Craig DH convention
  std::vector<double> d_, d_c_;
  // theta_ in Craig DH convention
  std::vector<double> theta_, theta_c_;

  // robot base offset, default to {0,0,0,0,0,0}, i.e. it matches with the world frame
  // Frame base_off_c_;
  // RNN algorithm
  RNN alg_;

  // cached velocity change for sam algorithm
  Eigen::VectorXd delta_p_old_cache_;
  bool resetCache_;
};

}
#endif /* SERIALARM_HPP */

