/*
 * this implements a python interpreter class, that exposes linear/ARC/PTP/and other commands used in python (for ease of user programming)
 */
#ifndef ROBINTP_INTP_HPP
#define ROBINTP_INTP_HPP
#include "robnux_trajectory/move_command.hpp"
#include "robnux_trajectory/trajectory_buffer.hpp"
#include "robnux_trajectory/s_curve.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "std_msgs/Float64.h"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/common_constants.hpp"
#include <eigen3/Eigen/Core>
#include "dsl_intp/intp_exportdecl.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"
#include "pluginlib/class_loader.h"
#include <chrono>
#include <thread>
#include <csignal>
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

void catch_signals() {
  auto handler = [](int code) {
      throw std::runtime_error("SIGNAL " + std::to_string(code)); };
  signal(SIGINT, handler);
  signal(SIGTERM, handler);
  signal(SIGKILL, handler);
}

using namespace std::chrono;
namespace kinematics_lib {
 static const double RAD2DEG=57.2957805;
 class INTP_API CreateRobot {
  public:
   //! Create the interpreter for interpreting commands in python-type robot language
   //@robName, which robot this interpreter ties to
   //@parameter, kinematic parameters related to this robot
   //@pf, how fast this robot could be (including max vel, acc, jerk)
   //@enable_motion_control,  do we enable robot control thread or not?
   CreateRobot(const std::string &robName,
               const EigenDRef<Eigen::VectorXd>& kine_para,
               const EigenDRef<Eigen::VectorXd>& defaultBaseOff,
               const EigenDRef<Eigen::VectorXd>& defaultSubBaseOff,
               const ProfileData &pf);
   ~CreateRobot();



   //! set joint feedback
   bool SetJntFeedback(const EigenDRef<Eigen::VectorXd>& jnt);


   //! start the two threads
   bool StartMotion();

   //!stop the two threads
   bool StopMotion();

   //!has robot finished all commands
   bool MotionDone(); 

   //!pause motion
   bool PauseMotion();

   //!resume motion
   bool ResumeMotion();
   
   //! send output data
   void SendOutputData(const Eigen::VectorXd& jnt);

   //! get feedback
   void GetInputData();

   //! set robot current joint position
   //! set speed scale
   bool SetSpeedScale(const ProfilePercent &perc);

   //! get speed scale
   ProfilePercent GetSpeedScale() const {
     return current_spd_percent_;
   }

   //! set joint profile
   bool SetJntProfile(const int jnt_index, const JntProfile& jpf);

   //! Line command (using Quaternion input)
   bool LIN(const LocData & loc, const FrameData &fd,
            const unsigned int appr_perc = 0);

   //! ARC command
   //@loc_1, loc_2: point 2,3 in 3-point method that determines an arc, however,
   // here we require loc_1 and loc_2 use the same Base and Tool frames
   bool ARC(const LocData & loc_1, const LocData &loc_2,
            const FrameData &fd, const unsigned int appr_perc = 0);

   //! PTP command with Cartesian pose input
   bool PTP(const LocData &center, 
            const FrameData &fd, const unsigned int appr_perc =0);
  
   //! PTP command with joint space input
   bool PTPJ(const EigenDRef<Eigen::VectorXd>& jnt,
             const unsigned int appr_perc=0);

   //! relative command
   bool LIN_REL(const LocData& rel_loc, const FrameData& fd,
                const unsigned int appr_perc = 0);

   //! PTP relative command
   bool PTP_REL(const LocData& rel_loc, const FrameData& fd,
                const unsigned int appr_perc = 0);

   //! Robot command accepting and processing thread functions
   void RobCmdInpThreadEntry();
   //! Trajectory executing thread
   void ExecTrajThreadEntry();
   //! publish joints to simulation or real targets
   void publishJnt(const Eigen::VectorXd& jnt, const Pose &p);

   //! get Cartesian coordinates based upon joint feedback, for python use
   bool GetCartFromJnt(const EigenDRef<Eigen::VectorXd> &jnt,
                       EigenDRef<Eigen::VectorXd> &cart);
  
   //! get Cartesian coordinates based upon joint feedback, for python use
   bool GetPoseFromJnt(const EigenDRef<Eigen::VectorXd> &jnt,
                       EigenDRef<Eigen::VectorXd> &pose);
   
   //! get Cartesian coordinates based upon joint feedback, for python use
   bool ForwardKin(const EigenDRef<Eigen::VectorXd> &jnt,
                   LocData& pose);

      //! get Cartesian coordinates based upon joint feedback, for python use
   bool GetJntFromPose(const EigenDRef<Eigen::VectorXd> &pose,
                       EigenDRef<Eigen::VectorXd> &jnt);

   bool InverseKin(const LocData& pose,
                        EigenDRef<Eigen::VectorXd> &jnt);

   void ShutDown();

  private:
   int DoF_;  // degrees of freedom
   // TrajectoryBuffer object for adding new command into command buffer, and then converting command buffer
   // into trajectory buffer
   std::shared_ptr<TrajectoryBuffer> tjBuff_;
   // there are two threads here, (1) interpreter thread (2) executing trajectory thread
   pthread_t  intp_, execTraj_;
   // time scale for gradually slowing down during pause/stop, as well as time scale for gradually speed up
   // when resume is hit 
   double time_scale_;
   // SCurveProfile for smoothingly pause and resume
   SCurveProfile prof_pause_, prof_resume_;
   // for remembering how many cycles till fully paused or resumed
   int pause_cycles_ = 0, resume_cycles_=0;
   // current spd percentage
   ProfilePercent current_spd_percent_;
   //  robot profile data, this is protained to each robot
   // should be passed into the object in the constructor
   ProfileData pf_;
   std::vector<JntProfile> jpf_;
   
   //! robot current joint angles, and desired joint angles
   Eigen::VectorXd current_jp_, jp_d_, jv_d_, ja_d_;
   Eigen::VectorXd last_jp_d_;  // last desired joint angles

   //! robot current pose, and initial pose (coming from very first feedback)
   refPose current_pose_;

   // latest goal, Note: in this class, all Cartesian pose under a given FrameData will be converted to
   // the Frame variable under Canonical kinematic model, i.d. under BaseNo=0 (default base,
   // or base offset is identity matrix ), ToolNo=0 (default tool, or tool offset is identity matrix)
   refPose last_goal_;

   //! kinematic map related to this robot (note: here for embedded version, we dind't try out
   // to use the mechanism of plugin load to reduce the size of binary and without using boost)
   boost::shared_ptr<BaseKinematicMap>  armMap_;
   pluginlib::ClassLoader<BaseKinematicMap> armMap_loader_;

   // thread for parsing robot commands
   static void *RobCmdInpThreadEntryFunc(void *myObject);
   // thread for executing robot commands
   static void *ExecTrajThreadEntryFunc(void *myObject);
   // whether initialized correctly
   bool initialized_;
   // trajectory task state machine
   // 0: pre_start
   // 1: start
   // 2: stop
   // 3: pause
   unsigned int traj_task_sm_;
   // command processing state machine
   // 0: pre_start
   // 1: start
   // 2: stop
   // 3: pause
   unsigned int rob_task_sm_;
   // flags that was set from users
   bool start_motion_, stop_motion_, pause_motion_;

   // period of robot task and traj task
   milliseconds rob_task_period_{20};  // default 20 ms, in the future, it should come from configurable parameters
   milliseconds traj_task_period_{2};  // default 2 ms, in the future, it should come from configurable parameters 

   //execution step
   double traj_exec_step_ = 0.002;  // default 2ms
   // if gets the jnt feedback yet
   bool feedback_done_;
   // sometimes we want to throw away the commands
   bool bypassCMDQueue_;
   // shutdown robot
   bool shutdown_;
   //! base frames and tool frames data set
   std::vector<Frame> bases_;
   std::vector<Frame> tools_;

      // this is the frame used for traj compensation
   Frame current_base_, current_tool_;
   
   // joint and cartesian publisher
   ros::Publisher pub_joint_cmd_, pub_cart_cmd_;
   std::vector<ros::Publisher> pub_joint_control_;
 };
}
#endif  /* ROBINTP_INTP_HPP */