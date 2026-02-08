#ifndef KINEMATICS_LIB_XYZ_UR_HPP_
#define KINEMATICS_LIB_XYZ_UR_HPP_
#include <common/vec.hpp>
#include <common/rotation.hpp>
#include <common/utility.hpp>
#include <Logger/Logger.h>
#include "kinematics_exportdecl.h"
#include "UJNT.hpp"
#include "XYZ.hpp"
#include <memory>
namespace kinematics_lib {
// separate-mode XYZ + UR
class KINEMATICS_API XYZ_UR : public BaseKinematicMap {
 public:
  //! default constructor
  XYZ_UR();

  //! constructor with vector of dh parameters
  //! recall we use Craig's DH convention, but not original DH convention
  //! @param dh_UR: dh parameters for separate UR robot
  //! @param dh_XYZ: dh parameters for separate XYZ robot
  XYZ_UR(const std::vector<double> &dh_UR, const std::vector<double> &dh_XYZ);

  //! canonical IK
  int CartToJnt(const Pose &p, std::vector<double> *q); 


  /*
  * set geometry parameters
  * @param parameters, the vector of parameters that characterize
  * the kinematic model, i.e. we need to separate out dh_UR and dh_XYZ from kine_para
  */
  void SetGeometry(const std::vector<double> &kine_para);

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
  int JntToCart(const std::vector<double> &q,
                const std::vector<double> &qdot,
                Pose *p, Twist *v);
        
        
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
   int JntToCart(const std::vector<double> &q,
                         const std::vector<double> &qdot,
                         const std::vector<double> &qddot,
                         Pose  *p, Twist *v, Twist *a) {
     return -1;
   }
  
   int CartToJnt(const Pose &p, const Twist &v,
                         std::vector<double> *q,
                         std::vector<double> *qdot);
 
   int CartToJnt(const Pose &p, const Twist& v, const Twist& a,
                         std::vector<double> *q,
                         std::vector<double> *qdot,
                         std::vector<double> *qddot) {
    return -1;
   }


   //! given size_t  inconfig,  convert it to std::<int>  branchFlags
   void ConvertBranchFlag(const size_t inFlag,
                                   std::vector<int> *branchFlags) const;
   void ConvertMultiTurnFlag(const size_t inFlag,
                                   std::vector<int> *ikTurnsFlags)  const;

   int CalcJacobian(const std::vector<double> &kine_para,
                            Pose *p,
                            Eigen::MatrixXd * Jp_t,
                            Eigen::MatrixXd * Jp_r,
                            const bool reduction = false);

   /*
    * DH calibration based of XYZ robot first using laser probing
    */
   double CalibrateLaserCoplanar(
       const EigenDRef<Eigen::MatrixXd> &cart_measure_x,  // cartesian coordinates reported from robot
       const EigenDRef<Eigen::MatrixXd> &cart_measure_y,
       const EigenDRef<Eigen::MatrixXd> &cart_measure_z,
       const EigenDRef<Eigen::MatrixXd> &laserMat_x,
       const EigenDRef<Eigen::MatrixXd> &laserMat_y,
       const EigenDRef<Eigen::MatrixXd> &laserMat_z,
       const EigenDRef<Eigen::Vector3d> &laser_scale);
  
   double LaserCalibrateCoplanar(
       const Eigen::MatrixXd& cart_measure_x,  // cartesian coordinates reported from robot
       const Eigen::MatrixXd& cart_measure_y,
       const Eigen::MatrixXd& cart_measure_z,
       const Eigen::MatrixXd& laserMat_x,
       const Eigen::MatrixXd& laserMat_y,
       const Eigen::MatrixXd& laserMat_z,
       const Eigen::Vector3d& laser_scale);

  void GetDefaultBaseOff(EigenDRef<Eigen::VectorXd> *baseoff,
                         EigenDRef<Eigen::VectorXd> *subbaseoff);

  void SetOrientNormal(const EigenDRef<Eigen::Vector3d>  &normal) override;

  //! Robot origin calibration using 3 plane, and each plane found the same pt on R
  double CalibrateLaserOrigin(
       // jnt measure matrix  j1 / j2 /j3 /j4 /j5
       const EigenDRef<Eigen::MatrixXd> &jnt_measure,
       // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
       const EigenDRef<Eigen::MatrixXd> &cart_measure,
       // z laser values
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure,
       const int laser_channel,
       const double laser_scale,  // laser scale for the above laser channel
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser distance, under which, laser reading is 0
       // 4 in a plane in each direction or 8 for a complete plane, then totally 8 * 3, 24 pts
       const int numPtsInEachOrientPlane) override;

  double LaserCalibrateOrigin(
       // jnt measure matrix  j1 / j2 /j3 /j4 /j5
       const Eigen::MatrixXd &jnt_measure,
       // cart     x/y/z/Orient, |p1, p2, p3, p4| for A1  |p5, p6, p7, p8| for A1'
       const Eigen::MatrixXd &cart_measure,
       // z laser values
       const Eigen::VectorXd &laserMat_z_measure,
       const int laser_channel,  // using which channel of lasr (x or y or z) for measuring
       const double laser_scale,  // laser scale for the above laser channel
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser dist, under which, laser reading is 0
       // 4 in a plane in each direction or 8 for a complete plane, then totally 8 * 3, 24 pts
       const int numPtsInEachOrientPlane) override;

   /*
    * DH calibration of the UR robot after XYZ robot calibration using special laser probing 
    */

   double CalibrateLaserOrientation(
       const EigenDRef<Eigen::MatrixXd> &jnt_measure,   //  | 5 * 6 | 5 * 6 | 5 * 6 | 5 * 6 | ..... Every surface: 6 column points, jnt4/jnt5 is constant for each surface
       const EigenDRef<Eigen::MatrixXd> &cart_measure,   //  | 8 * 6 | 8 * 6 | 8 * 6| 8 * 6 |  ...... Every surface: 6 column cart points 
       const EigenDRef<Eigen::VectorXd> &laserMat_z_measure, // | 1 * 6| 1 * 6 | 1 * 6 | 1 * 6| .......  Every surface: 6 column laser z readings
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const int laser_channel,    // which channel of laser used for orientation calibration
       const double laser_scale,   // laser scale
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist,  // maximal laser distance, under which, laser reading is 0
       const int numPtsInEachPlane,
       const std::vector<int> surfaceArrays);    // ........... how many points in each plane, e.g., 3, 
   
   double LaserCalibrateOrientation(
       const Eigen::MatrixXd &jnt_measure,
       const Eigen::MatrixXd &cart_measure,
       const Eigen::VectorXd &laserMat_z_measure,
       //const EigenDRef<Eigen::Vector3d> &init_normal,   // init normal vector
       const int laser_channel,
       const double laser_scale,
       const double laser_value, // to be finished tomorrow, could be reconfigurable from GUI
       const double max_laser_dist, // maximal laser dist, under which, laser reading is 0
       const int numPtsInEachPlane,
       const std::vector<int> surfaceArrays);

    /* 
    *  computing base, as well as tcp of workobj w.r.t. UR flange if UR is used for holding object
    */
    int ErrCompensateBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 EigenDRef<Eigen::VectorXd> *comp_base_uncal,
                                 EigenDRef<Eigen::VectorXd> *comp_base,
                                 EigenDRef<Eigen::MatrixXd> *origCart,  // orig cart points in measuring base
                                 EigenDRef<Eigen::MatrixXd> *compCart,  // comp cart points in measuring base
                                 EigenDRef<Eigen::VectorXd> *relq_cano,    // need to find out the desired orientation of laser in canonical model w.r.t. workpiece frame
                                 EigenDRef<Eigen::VectorXd> *relq_calib    // need to find out the desired orientaiton of laser in calibrated model w.r.t. ...
                                 );
    
    int ErrCompensateBase(const Eigen::MatrixXd &jnt_base_measures,
                                 const Eigen::VectorXd &orig_tool,
                                 Eigen::VectorXd *comp_base_uncal,
                                 Eigen::VectorXd *comp_base
                                 ) override;

    // using vision calibrate subTCP of xyz_ur mechanism
    int ErrVisionUserBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,  //(x,y, *, u, r)
                                const Eigen::VectorXd &orig_tool,  // tcp of vision (maybe)
                                EigenDRef<Eigen::VectorXd> *sub_tcp_uncal,    // uncalibrated tcp of UR workobj w.r.t UR
                                EigenDRef<Eigen::VectorXd> *sub_tcp_cal    // calibrated tcp of UR workobj w.r.t. UR
                               ) override;

    int ErrVisionUserBase(const Eigen::MatrixXd &jnt_base_measures,  //(x,y, *, u, r)
                                const Eigen::VectorXd &orig_tool,  // tcp of vision (maybe)
                                Eigen::VectorXd *sub_tcp_uncal,    // uncalibrated tcp of UR workobj w.r.t UR
                                Eigen::VectorXd *sub_tcp_cal    // calibrated tcp of UR workobj w.r.t. UR
                               );

    // using manual disp path calibrate subTCP
    int ErrManualPathUserBase(const EigenDRef<Eigen::MatrixXd> &jnt_base_measures,  //(x,y, z, u, r)
                                const Eigen::VectorXd &orig_tool,  // used tcp (default 0)
                                const int numXPts, // No. points in X plane
                                const int numYPts, // No. points in Y plane
                                const int numZPts, // No. points in Z plane
                                EigenDRef<Eigen::MatrixXd> *workobj_uncal,    // each col is a workobj frame (datum ref.)  appended with a UR jnt, so 8 * numLocs
                                EigenDRef<Eigen::MatrixXd> *workobj_cal    // each col is a workobj frame (data ref.)   appended with a UR jnt, so 8 * numLocs
                               ) override;
    // manual path to relpath w.r.t user frame
    int ManualPathToRelPath(const EigenDRef<Eigen::MatrixXd> &manPath,  //(x,y, *, u, r)
                                  const EigenDRef<Eigen::MatrixXd> &sub_tcp_cal,    // calibrated sub tcp
                                  EigenDRef<Eigen::MatrixXd> *relPath) override;

    int ManualPathToRelPath(const Eigen::MatrixXd &manPath,  //(x,y, *, u, r)
                                  const Eigen::MatrixXd &sub_tcp_cal,   // calibrated sub tcp
                                  Eigen::MatrixXd *relPath) override;

    
    int GenerateOriginMeasures(const EigenDRef<Eigen::MatrixXd> &jnt_in,
                               const int numPtsInPlanes, 
                               EigenDRef<Eigen::MatrixXd> *jnt_out) override;

    int  GenerateOriginMeasures(const Eigen::MatrixXd &jnt_in,
                                const int numPtsInPlanes,
                                Eigen::MatrixXd *jnt_out) override;
    
    /*
     *  5-axis path filtering
     */
    int  ErrCompensationDH(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd &bestTool,
        const EigenDRef<Eigen::MatrixXd> &d_traj,
        EigenDRef<Eigen::MatrixXd> *md_traj,
        EigenDRef<Eigen::MatrixXd> *d_j_traj,
        EigenDRef<Eigen::MatrixXd> *md_j_traj,
        EigenDRef<Eigen::MatrixXd> *a_traj);
    
    int  MultiLocFiltering(
        const Eigen::MatrixXd &calibBase,  // actual user base (or calibrated base) at each Loc of multiLocs, relative to which, desired relative path is based upon
        const Eigen::MatrixXd &origBase,  // uncalibrated base
        const Eigen::VectorXd &bestTool,  // used tool
        const EigenDRef<Eigen::MatrixXd> &d_traj,  // desired relative path (usually based upon CAD file)
        EigenDRef<Eigen::MatrixXd> *md_traj,
        EigenDRef<Eigen::MatrixXd> *d_j_traj,
        EigenDRef<Eigen::MatrixXd> *md_j_traj,
        EigenDRef<Eigen::MatrixXd> *a_traj);
    
    int  PathFiltering(
        const Eigen::VectorXd &calibBase,  // actual user base (or calibrated base) where the desired trajectory references to
        const Eigen::VectorXd &origBase,  // uncalibrated base that filtered path references to
        const Eigen::VectorXd &bestTool,
        const Eigen::MatrixXd &d_traj,
        Eigen::MatrixXd *md_traj,
        Eigen::MatrixXd *d_j_traj,
        Eigen::MatrixXd *md_j_traj,
        Eigen::MatrixXd *a_traj);

   void SetDefaultBaseOff(const EigenDRef<Eigen::VectorXd> &baseoff,
                          const EigenDRef<Eigen::VectorXd> &subbaseoff);
   
   void SetDefaultBaseOff(const Eigen::VectorXd &baseoff,
                          const Eigen::VectorXd &subbaseoff);

   void SanityLooseBound(const double coef);

   void DecayParam(const double coef) override;

   void SetPitchCoef(const EigenDRef<Eigen::VectorXd> &pitch);
   void SetPitchCoef(const Eigen::VectorXd &pitch) override;

   void SetRidgeCoef(const double linear_coef, const double ang_coef, const int cyc_mod, const double sam_region_scale,
                     const int coplanar_normal_option) override;


   bool GetCalibParamSet(EigenDRef<Eigen::VectorXd> *cal_DH);

   bool GetCalibParamSet(Eigen::VectorXd *cal_DH) override;


   bool LoadCalibParamSet(const std::vector<double> &cal_DH);

   //! calibration sanity check
   int CalibSanityCheck(const std::vector<double> &cal_DH);


   void SetUsingCalibratedModel(bool useCalibratedModel);


   int SetDependJacobianColumns(const std::vector<size_t> & d_cols) override;

   //! has robot been calibrated
   bool  isCalibrated();

   //! is parameter initialized
   bool isInitialized() const;
   //! reset calibration model
   bool resetCalibration();

   //! get name
   virtual std::string GetName() const {
       return std::string("XYZ_UR");
   }

 private:
   //! define two robots
   std::shared_ptr<BaseKinematicMap>  UR, XYZ;
};

}

#endif
