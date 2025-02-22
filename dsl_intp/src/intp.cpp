#include "dsl_intp/intp.hpp"
#include "simple_motion_logger/Logger.h"

using namespace ROBNUXLogging;

namespace kinematics_lib
{
  CreateRobot::CreateRobot(const std::string&robName,
                           const EigenDRef<Eigen::VectorXd>& kine_para,
                           const EigenDRef<Eigen::VectorXd>& defaultBaseOff,
                           const EigenDRef<Eigen::VectorXd>& defaultSubBaseOff,
                           const ProfileData& pf) : 
                                time_scale_(1.0), // initial time scale is always 1.0
                                pf_(pf),
                                armMap_loader_("kinematics_map", "kinematics_lib::BaseKinematicMap"),
                                initialized_(false), // always not initialized in the beginning
                                traj_task_sm_(0), rob_task_sm_(0),
                                start_motion_(false),
                                stop_motion_(false),
                                pause_motion_(false),
                                feedback_done_(false),
                                bypassCMDQueue_(false),
                                shutdown_(false) {
    int argc = 0;
    ros::init(argc, NULL, robName);
    std::ostringstream strs;
    current_spd_percent_.vel_perc_ = 10;
    current_spd_percent_.acc_perc_ = 10;
    current_spd_percent_.jerk_perc_ = 10;
    armMap_ = armMap_loader_.createInstance(robName);
    if (!armMap_) {
      return;
    }
    armMap_->SetGeometry(kine_para);
    armMap_->SetDefaultBaseOff(defaultBaseOff, defaultSubBaseOff);
    // initialize base and tool frame data sets to identity default frame
    bases_.resize(8, Frame());
    tools_.resize(8, Frame());

    DoF_ = armMap_->GetActDoF();
    
    jpf_.resize(DoF_);
    current_jp_.resize(DoF_);
    jp_d_.resize(DoF_);
    jv_d_.resize(DoF_);
    ja_d_.resize(DoF_); 
    tjBuff_ = std::make_shared<TrajectoryBuffer>(armMap_);

    // traj pause/resume time-scale change profile
    // we hope to have transition time about 0.1s (50 periods) to complete pause and stop motion
    // similarly we wish to spend same time for resuming motion
    prof_pause_.setConstraints(20, 40, 60);
    if (!prof_pause_.setBoundaryCondition(1, 0, 0, 0)) {
      strs.str("");
      strs << "profile for pausing motion fails" << std::endl;
      LOG_ERROR(strs);
      return; // initialize failed
    }
    prof_resume_.setConstraints(20, 40, 60);
    if (!prof_resume_.setBoundaryCondition(0, 1, 0, 0)) {
      strs.str("");
      strs << "profile for resuming motion fails" << std::endl;
      LOG_ERROR(strs);
      return; // initialize failed
    }

      // initialize two publishers
    ros::NodeHandle nh("~");
    pub_joint_cmd_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 2);
    pub_cart_cmd_ = nh.advertise<geometry_msgs::PoseStamped>("cart_pose", 2);
    pub_joint_control_.resize(DoF_);
    // initialize jnt cmd topic names and publisher
    for (int i=0; i< DoF_; i++) {
        std::string topic = "joint" +std::to_string(i+1) + 
                        "_position_controller/command";
        pub_joint_control_[i] = nh.advertise<std_msgs::Float64>(topic, 10);
    }

      // initialize two threads, one for adding commands to cmd buffer, and then converting
      // cmd buffer to traj buff
      if (pthread_create(&intp_, NULL /*&attr_intp */,
                         RobCmdInpThreadEntryFunc, this) == 0 &&
          pthread_create(&execTraj_, NULL /*&attr_exec_traj*/,
                         ExecTrajThreadEntryFunc, this) == 0) {
        initialized_ = true;
      } else {
        strs.str("");
        strs << "Create thread fails in " << __FUNCTION__ << " , at line "
             << __LINE__
             << std::endl;
        LOG_ERROR(strs);
      }
    
  }

  CreateRobot::~CreateRobot() {
    if (initialized_) {
      shutdown_ = true;
      pthread_join(intp_, NULL);
      pthread_join(execTraj_, NULL);
      armMap_.reset();
      armMap_ = nullptr;
      tjBuff_.reset();
      tjBuff_ = nullptr;
    }
    // ros::shutdown();
    // all other std::share_ptr will automaticall cleaned out after here
  }
  
  void CreateRobot::ShutDown() {
    if (initialized_) {
      shutdown_ = true;
      pthread_join(intp_, NULL);
      pthread_join(execTraj_, NULL);
      tjBuff_.reset();
      tjBuff_ = nullptr;
      armMap_.reset();
      armMap_ = nullptr;
      initialized_ = false;
    }
  }

  void *CreateRobot::RobCmdInpThreadEntryFunc(void *myObject) {
    CreateRobot *obj = (CreateRobot *)(myObject); //static_cast<CreateRobot*>(myObject);
    if (obj) {
      obj->RobCmdInpThreadEntry();
    } else {
      std::ostringstream strs;
      strs << "input my object is NULL in " << __FUNCTION__ << " at line "
      << __LINE__ << std::endl;
      LOG_ERROR(strs);
    }
    return NULL;
  }

  void *CreateRobot::ExecTrajThreadEntryFunc(void *myObject) {
    CreateRobot *obj = (CreateRobot *)(myObject); //static_cast<CreateRobot*>(myObject);
    if (obj) {
      obj->ExecTrajThreadEntry();
    } else {
      std::ostringstream strs;
      strs << "input my object is NULL in " << __FUNCTION__ << " at line " 
      << __LINE__ << std::endl;
      LOG_INFO(strs);
    }
    return NULL;
  }

  void CreateRobot::RobCmdInpThreadEntry() {
    system_clock::time_point current_time(system_clock::now());
    system_clock::time_point next_time(current_time);
    //    ROS_INFO("entered robCmdINp Thread");
    while (true && !shutdown_) {
      try{
        current_time = system_clock::now();
        //      ROS_INFO("robot task state %u", rob_task_sm_);
        switch (rob_task_sm_) {
          case 0: // pre_start
            if (start_motion_ && feedback_done_) {                        //util receiving feedback, we will not start processing command
               pause_motion_ = false; // if start motion is called, reset pause_motion and stop_motion flag
               stop_motion_ = false;
               rob_task_sm_ += 1;
            }
           break;
        case 1: // if motion is started
          if (pause_motion_ || stop_motion_) {
            start_motion_ = false; // reset start_motion flag
            rob_task_sm_ = 0;
          } else {
            // if already started motion and no pause/stopping, let tjBuff_ object processing the command buffer
            // and translate that into trajectory buffer
            tjBuff_->ProcessCommandBuffer();
          }
          break;
        }
        next_time = current_time + rob_task_period_;
        std::this_thread::sleep_until(next_time);
      } catch (KinematicsException & e) {
        LOG_ERROR(e.what());
        shutdown_ = true;  //shutdown thread
      } catch (...) { 
        shutdown_ = true;  //shutdown thread
      }
    }
  }

  void CreateRobot::ExecTrajThreadEntry() {
    system_clock::time_point current_time(system_clock::now());
    system_clock::time_point next_time(current_time);
    double scale_v, scale_a; // temparary variable for the vel and acc of the scale variable
    double scale_t;          // time_scale_ is function of scale_t
    while (true && !shutdown_) {
      try {
        current_time = system_clock::now();
        switch (traj_task_sm_) {
          case 0: // pre_start
            if (start_motion_) {
              pause_cycles_ = 0; // reset pause cycle to 0
              traj_task_sm_ += 1;
            }
            break;
          case 1: // if motion is started, but someone call pause or stop motion, then we need gradually decel to 0
            if (pause_motion_ || stop_motion_) {
              scale_t = pause_cycles_ * traj_exec_step_;
              // compute actual time_scale_ at scale_t
              prof_pause_.Trajectory(scale_t, &time_scale_, &scale_v, &scale_a);
              pause_cycles_ += 1;
              if (scale_t > prof_pause_.Duration()) {
                resume_cycles_ = 0;
                traj_task_sm_ += 1;
              }
            }
            break;
          case 2:
            if (start_motion_) { // if resume motion again from pause/stop motion
              scale_t = resume_cycles_ * traj_exec_step_;
              // compute actual time_scale_ at scale_t
              prof_resume_.Trajectory(scale_t, &time_scale_, &scale_v, &scale_a);
              resume_cycles_ += 1;
              if (scale_t > prof_resume_.Duration()) {
                pause_cycles_ = 0; // resume pause cycles, preparing for next pause operation
                traj_task_sm_ = 1;
              }
            }
            break;
          default:
            break;
        }
        if (tjBuff_->ExecuteTrajBuffer(time_scale_ * traj_exec_step_, &jp_d_,
                         &jv_d_, &ja_d_)) {
             
          SendOutputData(jp_d_); // send out control output based upon control_mode_, and control_output_
        }
        GetInputData();               // get feedback (every time, we send out an output commmand)
        next_time = current_time + traj_task_period_;
        std::this_thread::sleep_until(next_time);
      } catch (KinematicsException & e) {
          LOG_ERROR(e.what());
          shutdown_ = true;  //shutdown thread
      } catch (...) { 
          shutdown_ = true;  //shutdown thread
      }
    }
  }
  
  void CreateRobot::GetInputData() {
      return;
  }

  void CreateRobot::SendOutputData(const Eigen::VectorXd& jnt) {
    Pose d_ps;
    
    //rps.getDefaultPose(&d_ps);
    armMap_->JntToCart(jnt, &d_ps); // in simulation, 
    //ROS_INFO("active_joint size=%lu", jnt.size());
      // we always choose canonical model
      // publish Joint angles to rviz or to robot controller
    publishJnt(jnt, d_ps);
     std_msgs::Float64 msgs;
    for (unsigned int i = 0; i < DoF_; i++) {
        msgs.data = jnt(i) * RAD2DEG;
        pub_joint_control_[i].publish(msgs);
    }
  }

  bool CreateRobot::MotionDone() {
    return tjBuff_->IsTrajBufferEmpty() &&
           tjBuff_->IsCommandBufferEmpty();
  }
  bool CreateRobot::StartMotion() {
    if (initialized_ && feedback_done_) {
      start_motion_ = true;
    } else {
      start_motion_ = false;
    }
    return start_motion_;
  }

  bool CreateRobot::StopMotion() {
    if (initialized_ && feedback_done_) {
      stop_motion_ = true;
    } else {
      stop_motion_ = false;
    }
    return stop_motion_;
  }

  bool CreateRobot::PauseMotion() {
    if (initialized_ && feedback_done_) {
      pause_motion_ = true;
    } else {
      pause_motion_ = false;
    }
    return pause_motion_;
  }

  bool CreateRobot::ResumeMotion() {
    if (initialized_ && feedback_done_) {
      start_motion_ = true;
    } else {
      start_motion_ = false;
    }
    return start_motion_;
  }

  bool CreateRobot::SetSpeedScale(const ProfilePercent &perc) {
    current_spd_percent_ = perc;
    return true;
  }

  bool CreateRobot::SetJntFeedback(const EigenDRef<Eigen::VectorXd>& jnt) {
    std::ostringstream strs;
    if (!initialized_ || !armMap_) {
      return false;
    }
    if (jnt.size() != DoF_) {
      return false;
    }
    current_jp_ = jnt;
    last_jp_d_ = jnt;
    // apply FK to obtain the current_pose
    Pose ps;
    if (armMap_->JntToCart(current_jp_, &ps) == 0) {
      current_pose_.setDefaultPose(ps);
      // very last goal is set to current_pose_ here before executing any motion commands
      last_goal_ = current_pose_;
      strs << " initial pose is " <<  current_pose_.ToString(true) << std::endl;
      LOG_INFO(strs);
      feedback_done_ = true;
      return true;
    }
    return false;
  }

  bool CreateRobot::SetJntProfile(const int jnt_index, const JntProfile &jpf) {
      std::ostringstream ss;
      if (jnt_index < 0 && jnt_index >= DoF_) {
          ss.str("");
          ss << " jnt index " <<  jnt_index << " is out of scope" << std::endl;
          LOG_ERROR(ss);
          return false;
      } else {
         jpf_[jnt_index] = jpf;
         return true;
      }
  }
  
  bool CreateRobot::LIN(const LocData &loc, const FrameData &fd,
                       const unsigned int appr_perc) {
    catch_signals();
    std::ostringstream strs;
    // first check if initialized and feedback has been received
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    // first get Frame  g_bt (from base frame b to tool frame t)
    Vec p(loc.x_, loc.y_, loc.z_);
    Rotation r(loc.C_, loc.B_, loc.A_);
    Frame fr(r, p);
    // get reference frame B_ and T_
    if (fd.baseNo_ < 0 || fd.baseNo_ > MAX_BASE_FRAMES) {
      return false;
    }
    Frame B_ = bases_[fd.baseNo_];
    Frame T_ = tools_[fd.toolNo_];
    
    std::vector<int> ikFlag, ikJntTurn;
    
    SingleInt2RobnuxBranch(loc.branch_, ikFlag);
    SingleInt2RobnuxTurn(loc.turns_, DoF_, ikJntTurn); 
    // create the refPose object (recall refPose is a Pose with nondefault base and tool)
    refPose rp(fr, B_, T_, ikFlag, ikJntTurn);

    //refPose rp_usr, rp_usr_last; //rp w.r.t. current base and tool frame
    //rp.getPoseUnderNewRef(current_base_, current_tool_, &rp_usr);

    Pose startPose, goalPose;
    if (!rp.getDefaultPose(&goalPose)) {
      return false;
    }
    if (armMap_->CartToJnt(goalPose, &last_jp_d_) < 0) {
        strs.str("");
        strs <<  " IK of goal pose= " << goalPose.ToString(true)
         << " fails in function " << __FUNCTION__ <<
         ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return false;
    }
    //last_goal_.getPoseUnderNewRef(current_base_, current_tool_, &rp_usr_last);
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    strs.str("");
    strs <<__FUNCTION__ << ":" << __LINE__ 
         << " initial pose is " << startPose.ToString(true)
         <<  " goal pose is " << goalPose.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    ProfileData current_pf = pf_;
    current_pf.max_vel_t_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_t_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_t_ *= current_spd_percent_.jerk_perc_ / 100.0;

    current_pf.max_vel_r_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_r_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_r_ *= current_spd_percent_.jerk_perc_ / 100.0;

    std::shared_ptr<MotionCommand> cmd = std::make_shared<LineMotionCommand>(
           startPose, goalPose, current_pf, appr_perc);
    // update last pose
    last_goal_ = rp; //rp.getDefaultPose(&last_goal_);
    return tjBuff_->AddCommand(cmd);
  }

  bool CreateRobot::ARC(const LocData &loc_1, const LocData &loc_2,
                        const FrameData &fd, const unsigned int appr_perc) {
    catch_signals();
    std::ostringstream strs;
    // first check if initialized and feedback has been received
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        // if bypass the command queue checking, then return rightawaty
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    // first get Frame  g_bt (from base frame b to tool frame t)
    Vec p(loc_1.x_, loc_1.y_, loc_1.z_);
    Rotation r(loc_1.C_, loc_1.B_, loc_1.A_);
    Frame fr(r, p);
    // get reference frame B_ and T_
    if (fd.baseNo_ < 0 || fd.baseNo_ > MAX_BASE_FRAMES) {
      return false;
    }
    Frame B_ = bases_[fd.baseNo_];
    Frame T_ = tools_[fd.toolNo_];
    std::vector<int> ikFlag, ikJntTurn;
    //last_goal_.getBranchFlags(&ikFlag);
    //last_goal_.getJointTurns(&ikJntTurn);
    SingleInt2RobnuxBranch(loc_1.branch_, ikFlag);
    SingleInt2RobnuxTurn(loc_1.turns_, DoF_, ikJntTurn); 
    // create the refPose object based upon loc1
    refPose rp(fr, B_, T_, ikFlag, ikJntTurn);
    
    Pose startPose, midPose, goalPose;
    if (!rp.getDefaultPose(&midPose)) {
      return false;
    }
    
    // generate refPose based upon loc2
    SingleInt2RobnuxBranch(loc_2.branch_, ikFlag);
    SingleInt2RobnuxTurn(loc_2.turns_, DoF_, ikJntTurn); 
    rp.setTranslation(Vec(loc_2.x_, loc_2.y_, loc_2.z_));
    rp.setRotation(Rotation(loc_2.C_, loc_2.B_, loc_2.A_));
    rp.setBranchFlags(ikFlag);
    rp.setJointTurns(ikJntTurn);
    

    if (!rp.getDefaultPose(&goalPose)) {
      return false;
    }

    //last_goal_.getPoseUnderNewRef(current_base_, current_tool_, &rp_usr_last);
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    if (armMap_->CartToJnt(goalPose, &last_jp_d_) < 0) {
        strs.str("");
        strs <<  " IK of goal pose= " << goalPose.ToString(true)
         << " fails in function " << __FUNCTION__ <<
         ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return false;
    }

    ProfileData current_pf = pf_;
    current_pf.max_vel_t_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_t_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_t_ *= current_spd_percent_.jerk_perc_ / 100.0;

    current_pf.max_vel_r_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_r_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_r_ *= current_spd_percent_.jerk_perc_ / 100.0;

    std::shared_ptr<MotionCommand> cmd = 
          std::make_shared<ArcMotionCommand>(startPose, midPose,
                                             goalPose, current_pf, appr_perc);
    // update last pose
    last_goal_ = rp;
    return tjBuff_->AddCommand(cmd);
  }

  bool CreateRobot::PTP(const LocData &dest,
                        const FrameData &fd, const unsigned int appr_perc) {
    catch_signals();
    std::ostringstream strs;
    // first check if initialized and feedback has been received 
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while  (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        // if bypass the command queue checking, then return rightawaty
        return false;
      }
      usleep(100);
    }
    // first get Frame  g_bt (from base frame b to tool frame t)
    Vec p(dest.x_, dest.y_, dest.z_);
    Rotation r(dest.C_, dest.B_, dest.A_);
    Frame fr(r, p);
    // get reference frame B_ and T_
    if (fd.baseNo_ < 0 || fd.baseNo_ > MAX_BASE_FRAMES) {
      return false;
    }
    Frame B_ = bases_[fd.baseNo_];
    Frame T_ = tools_[fd.toolNo_];
    std::vector<int> ikFlag, ikJntTurn;
    //last_goal_.getBranchFlags(&ikFlag);
    //last_goal_.getJointTurns(&ikJntTurn);
    SingleInt2RobnuxBranch(dest.branch_, ikFlag);
    SingleInt2RobnuxTurn(dest.turns_, DoF_, ikJntTurn); 
    // create the refPose object based upon loc1
    refPose rp(fr, B_, T_, ikFlag, ikJntTurn);
    Pose startPose, goalPose;
    if (!rp.getDefaultPose(&goalPose)) {
        return false;
    }
    if (armMap_->CartToJnt(goalPose, &last_jp_d_) < 0) {
        strs.str("");
        strs <<  " IK of goal pose= " << goalPose.ToString(true)
         << " fails in function " << __FUNCTION__ <<
         ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return false;
    }
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    /*
    std::vector<double> g_jnt;
    if (armMap_->CartToJnt(dps, &g_jnt) < 0) {
        return false;
    }
    */
    for (size_t i=0; i < DoF_; i++) {
     jpf_[i].max_vel_ *= (double)current_spd_percent_.vel_perc_ / 100.0;
     jpf_[i].max_acc_ *= (double) current_spd_percent_.acc_perc_ / 100.0;
     jpf_[i].max_jerk_ *= (double) current_spd_percent_.jerk_perc_ / 100.0;
    }

    /*
    std::ostringstream ss;
    ss << " goal jnt is ";
    for (size_t i=0; i < last_jnt_.size(); i++) {
        ss << g_jnt[i] << " ";
    }
    ss << std::endl;
    LOG_INFO(ss);
     */
    strs.str("");
    strs <<__FUNCTION__ << ":" << __LINE__ 
         << " initial pose is " << startPose.ToString(true)
         <<  " goal pose is " << goalPose.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    std::shared_ptr<PTPMotionCommand> cmd =
            std::make_shared<PTPMotionCommand>(startPose, goalPose,
                                              jpf_, armMap_,  appr_perc);
    /* for PTP: no need to set config and turn*/
    /*
    // for this special PTP motion, should have same config
    cmd->SetStartConfig(ikFlag);
    cmd->SetGoalConfig(ikFlag);
    
    // similarly, for this special PTP motion, should have same turn
    cmd->SetStartTurn(ikJntTurn);
    cmd->SetGoalTurn(ikJntTurn);
     */ 
    // update last pose
    last_goal_ = rp;
    return tjBuff_->AddCommand(cmd);
  }

  bool CreateRobot::PTPJ(const EigenDRef<Eigen::VectorXd>& jnt, 
          const unsigned int appr_perc) {
    catch_signals();
    std::ostringstream strs;
    // first check if initialized and feedback has been received 
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while  (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        // if bypass the command queue checking, then return rightawaty
        return false;
      }
      usleep(100);
    }

    for (size_t i=0; i < DoF_; i++) {
     jpf_[i].max_vel_ *= current_spd_percent_.vel_perc_ / 100.0;
     jpf_[i].max_acc_ *= current_spd_percent_.acc_perc_ / 100.0;
     jpf_[i].max_jerk_ *= current_spd_percent_.jerk_perc_ / 100.0;
    }
    
    Pose startPose, goalPose;
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    
    if (armMap_->JntToCart(jnt, &goalPose) < 0) {
        strs.str("");
        strs <<  " FK of jnt= " << jnt
         << " fails in function " << __FUNCTION__ <<
         ", at line " << __LINE__ << std::endl;
        LOG_ERROR(strs);
        return false;
    }
    last_jp_d_ = jnt;
    strs.str("");
    strs <<__FUNCTION__ << ":" << __LINE__ 
         << " initial pose is " << startPose.ToString(true)
         <<  " goal pose is " << goalPose.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    std::shared_ptr<PTPMotionCommand> cmd =
            std::make_shared<PTPMotionCommand>(startPose, goalPose,
                                              jpf_, armMap_,  appr_perc);
    
    refPose rp;
    rp.setDefaultPose(goalPose);
    // update last pose
    last_goal_ = rp;
    return tjBuff_->AddCommand(cmd);
  }

  bool CreateRobot::LIN_REL(const LocData &rel_loc, const FrameData &fd,
                            const unsigned int appr_perc) {
    catch_signals();
    std::ostringstream strs;
    // first check if initialized and feedback has been received
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    // get reference frame B_ and T_
    if (fd.baseNo_ < 0 || fd.baseNo_ > MAX_BASE_FRAMES) {
      return false;
    }
    Frame B_ = bases_[fd.baseNo_];
    Frame T_ = tools_[fd.toolNo_];
 
    refPose goal_ref;
    last_goal_.getPoseUnderNewRef(B_, T_, &goal_ref);

    Pose startPose, goalPose;
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    
    Vec v = goal_ref.getTranslation();
    Vec dv(rel_loc.x_, rel_loc.y_, rel_loc.z_);
    goal_ref.setTranslation(v + dv);
    
    if (!goal_ref.getDefaultPose(&goalPose)) {
      return false;
    }
    

    ProfileData current_pf = pf_;
    current_pf.max_vel_t_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_t_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_t_ *= current_spd_percent_.jerk_perc_ / 100.0;

    current_pf.max_vel_r_ *= current_spd_percent_.vel_perc_ / 100.0;
    current_pf.max_acc_r_ *= current_spd_percent_.acc_perc_ / 100.0;
    current_pf.max_jerk_r_ *= current_spd_percent_.jerk_perc_ / 100.0;

    strs.str("");
    strs <<__FUNCTION__ << ":" << __LINE__ 
         << " initial pose is " << startPose.ToString(true)
         <<  " goal pose is " << goalPose.ToString(true)
         << std::endl;
    LOG_INFO(strs); 
    std::shared_ptr<MotionCommand> cmd = std::make_shared<LineMotionCommand>(
           startPose, goalPose, current_pf, appr_perc);

    last_goal_ = goal_ref;
    return tjBuff_->AddCommand(cmd);
    return true;
  }

  bool CreateRobot::PTP_REL(const LocData &rel_loc,
                            const FrameData &fd,
                            const unsigned int appr_perc) {
    std::ostringstream strs;
    // first check if initialized and feedback has been received 
    if (!initialized_ || !feedback_done_) {
      return false;
    }
    // second check whether command buffer fills enough commands
    while  (tjBuff_->IsCommandBufferFull()) {
      if (bypassCMDQueue_ || shutdown_) {
        // if bypass the command queue checking, then return rightawaty
        return false;
      }
      usleep(100);
    }
  
    // get reference frame B_ and T_
    if (fd.baseNo_ < 0 || fd.baseNo_ > MAX_BASE_FRAMES) {
      return false;
    }
    Frame B_ = bases_[fd.baseNo_];
    Frame T_ = tools_[fd.toolNo_];
   
    refPose goal_ref;
    last_goal_.getPoseUnderNewRef(B_, T_, &goal_ref);

    Pose startPose, goalPose;
    if (!last_goal_.getDefaultPose(&startPose)) {
      return false;
    }
    
    Vec v = goal_ref.getTranslation();
    Vec dv(rel_loc.x_, rel_loc.y_, rel_loc.z_);
    goal_ref.setTranslation(v + dv);
    
    if (!goal_ref.getDefaultPose(&goalPose)) {
      return false;
    }
    for (size_t i=0; i < DoF_; i++) {
     jpf_[i].max_vel_ *= current_spd_percent_.vel_perc_ / 100.0;
     jpf_[i].max_acc_ *= current_spd_percent_.acc_perc_ / 100.0;
     jpf_[i].max_jerk_ *= current_spd_percent_.jerk_perc_ / 100.0;
    }

    strs.str("");
    strs <<__FUNCTION__ << ":" << __LINE__ 
         << " initial pose is " << startPose.ToString(true)
         <<  " goal pose is " << goalPose.ToString(true)
         << std::endl;
    LOG_INFO(strs);
    std::shared_ptr<PTPMotionCommand> cmd =
            std::make_shared<PTPMotionCommand>(startPose, goalPose,
                                              jpf_, armMap_,  appr_perc);

    // update last pose
    last_goal_ = goal_ref;
    return tjBuff_->AddCommand(cmd);
  }

  void CreateRobot::publishJnt(const Eigen::VectorXd &jnt_a, const Pose &pose) {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = "rob_base";
    Vec p = pose.getTranslation();
    Quaternion q = pose.getQuaternion();
    poseMsg.pose.position.x = p.x();
    poseMsg.pose.position.y = p.y();
    poseMsg.pose.position.z = p.z();
    poseMsg.pose.orientation.w = q.w();
    poseMsg.pose.orientation.x = q.x();
    poseMsg.pose.orientation.y = q.y();
    poseMsg.pose.orientation.z = q.z();
    pub_cart_cmd_.publish(poseMsg);

    sensor_msgs::JointState msgs;
    msgs.header.stamp = ros::Time::now();

    msgs.name = *armMap_->GetJntNames();
    // tmp.insert(tmp.end(), jnt_a.begin(), jnt_a.end());
    Eigen::VectorXd jnt_p;
    if (!armMap_->CalcPassive(jnt_a, pose, &jnt_p)) {
      Eigen::VectorXd tmp(jnt_a.size() + jnt_p.size());
      tmp.segment(0, jnt_a.size()) = jnt_a;
      tmp.segment(jnt_a.size(), jnt_p.size()) = jnt_p; 
      //tmp.insert(tmp.end(), jnt_p.begin(), jnt_p.end());
      //msgs.position = tmp;
      EigenVec2StdVec(tmp, &msgs.position);
      // ROS_INFO("name size= %lu, position size= %lu", msgs.name.size(), tmp.size());
      pub_joint_cmd_.publish(msgs);
    }
  }

 bool CreateRobot::GetCartFromJnt(const EigenDRef<Eigen::VectorXd> &jnt,
                       EigenDRef<Eigen::VectorXd> &cart) {
    std::ostringstream strs;
    if (!initialized_) {
       strs.str("");
       strs << "Arm not initialized, API " << __FUNCTION__ << " can not be called"
            << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    refPose rps;
    /*
    std::vector<double> jnt1(DoF_, 0);
    for (size_t i=0; i < DoF_; i++) {
        jnt1[i] = jnt(i);
    }
     */ 
    armMap_->JntToCart(jnt, &rps); // from default world to default tool, even if robot base is not
                                    // identity, we still can use SetCalibMode to incorporate the base_off
    strs.str("");
    strs << "after jtocart, p=" << rps.getTranslation().ToString() << std::endl;
    strs << "after jtocart, q=" << rps.getQuaternion().ToString(false) << ", euler=" <<rps.getQuaternion().ToString(true) <<  std::endl;
    strs << "after jtocart, r=" << rps.getRotation().ToString() << std::endl;
    LOG_INFO(strs);
   
    refPose rps1;
    rps.getPoseUnderNewRef(current_base_, current_tool_, &rps1); // here we get frame from user base to user_tool

    
    strs.str("");
    strs << "after w.new ref, p =" << rps1.getTranslation().ToString() << std::endl;
    strs << "after w.new ref, q =" << rps1.getQuaternion().ToString(false) << ", euler=" <<rps1.getQuaternion().ToString(true) <<  std::endl;
    strs << "after w.new ref, r=" << rps1.getRotation().ToString() << std::endl;
    LOG_INFO(strs);
    
    Vec p = rps1.getTranslation();
    Rotation r = rps1.getRotation();

    double yaw, pitch, roll;
    r.GetEulerZYX(&yaw, &pitch, &roll);
 
    cart.resize(6);
    cart(0) = p.x();
    cart(1) = p.y();
    cart(2) = p.z();
    cart(3) = roll;
    cart(4) = pitch;
    cart(5) = yaw;
    
    
    //cart = tmp;
    return true;
  }
bool CreateRobot::ForwardKin(const EigenDRef<Eigen::VectorXd> &jnt,
                   LocData &pose) {
    std::ostringstream strs;
    if (!initialized_) {
       strs.str("");
       strs << "Arm not initialized, API " << __FUNCTION__ << " can not be called"
            << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    Pose ps;
    /*
    std::vector<double> jnt1(DoF_, 0);
    for (size_t i=0; i < DoF_; i++) {
        jnt1[i] = jnt(i);
    }
     */ 
    if (armMap_->JntToCart(jnt, &ps) <0) {// from default world to default tool, even if robot base is not
                                    // identity, we still can use SetCalibMode to incorporate the base_off
      return false;
    
    }
    
    Vec p = ps.getTranslation();
    Rotation r = ps.getRotation();
    double yaw, pitch, roll;
    r.GetEulerZYX(&yaw, &pitch, &roll);

    pose.x_ = p.x();
    pose.y_ = p.y();
    pose.z_ = p.z();
    pose.A_ = roll;
    pose.B_ = pitch;
    pose.C_ = yaw;
    std::vector<int> branchFlags;
    ps.getBranchFlags(&branchFlags);
    pose.branch_ = RobnuxBranch2SingleInt(branchFlags);

    std::vector<int> turns;
    ps.getJointTurns(&turns);
    pose.turns_ = RobnuxTurn2SingleInt(turns);
    //cart = tmp;
    return true;
}
 
 bool CreateRobot::GetPoseFromJnt(const EigenDRef<Eigen::VectorXd> &jnt,
                       EigenDRef<Eigen::VectorXd> &pose) {
    std::ostringstream strs;
    if (!initialized_) {
       strs.str("");
       strs << "Arm not initialized, API " << __FUNCTION__ << " can not be called"
            << std::endl;
       LOG_ERROR(strs);
       return false;
    }
    refPose rps;
    /*
    std::vector<double> jnt1(DoF_, 0);
    for (size_t i=0; i < DoF_; i++) {
        jnt1[i] = jnt(i);
    }
     */ 
    if (armMap_->JntToCart(jnt, &rps) <0) {// from default world to default tool, even if robot base is not
                                    // identity, we still can use SetCalibMode to incorporate the base_off
      return false;
    
    }
    
    refPose rps1;
    rps.getPoseUnderNewRef(current_base_, current_tool_, &rps1); // here we get frame from user base to user_tool
    
    Vec p = rps1.getTranslation();
    Rotation r = rps1.getRotation();
    double yaw, pitch, roll;
    r.GetEulerZYX(&yaw, &pitch, &roll);
 
    pose.resize(8);   // pose(6) is config, while pose(7) is joint turns
    pose(0) = p.x();
    pose(1) = p.y();
    pose(2) = p.z();
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;
    std::vector<int> branchFlags;
    rps1.getBranchFlags(&branchFlags);
    pose(6) = RobnuxBranch2SingleInt(branchFlags);

    std::vector<int> turns;
    rps1.getJointTurns(&turns);
    pose(7) = RobnuxTurn2SingleInt(turns);
    //cart = tmp;
    return true;
  }
 
 bool CreateRobot::InverseKin(const LocData& pose,
                        EigenDRef<Eigen::VectorXd> &jnt) {
     std::ostringstream strs;
    if (!initialized_){
       strs.str("");
       strs << "Arm not initialized, API " << __FUNCTION__ << " can not be called"
            << std::endl;
       LOG_ERROR(strs);
      return false;
    }

    //std::vector<double> jnt1(DoF_, 0);
    Eigen::VectorXd jnt1(DoF_);
    
    Vec trans(pose.x_, pose.y_, pose.z_);
    Rotation r(pose.C_, pose.B_, pose.A_);
    Frame fr(r, trans);

    int flag = (int) pose.branch_;
    
    std::vector<int> branchFlags;
    SingleInt2RobnuxBranch(flag, branchFlags);

    std::vector<int> ikJointTurns;
    int turn = (int) pose.turns_;
    SingleInt2RobnuxTurn(turn, DoF_, ikJointTurns);
  
    strs.str("");
    strs << "Pose is " << fr.ToString(true) <<  std::endl;
    for (size_t j=0; j< DoF_; j++) {
      strs << "turn " << j << " = " << ikJointTurns[j] << std::endl;
    }
    LOG_INFO(strs);


    Pose ps;
    refPose rps;

    rps.setFrame(fr);
    rps.setBranchFlags(branchFlags);
    rps.setBase(current_base_);
    rps.setTool(current_tool_);
    rps.setJointTurns(ikJointTurns); // so far only default turns, will handle complicated turns in the future
    rps.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
    //strs.str("");
    //strs << "under default base, the pose is " << ps.ToString(true) << std::endl;
    //LOG_INFO(strs);

    if (armMap_->CartToJnt(ps, &jnt1) < 0) {
      return false;
    }
    /*
    jnt.resize(DoF_);
    for (size_t i=0; i < DoF_; i++) {
        jnt(i) = jnt1[i];
    }*/
    jnt = jnt1;
    return true; 
 }

  bool CreateRobot::GetJntFromPose(const EigenDRef<Eigen::VectorXd> &pose,
                       EigenDRef<Eigen::VectorXd> &jnt) {
    std::ostringstream strs;
    if (!initialized_){
       strs.str("");
       strs << "Arm not initialized, API " << __FUNCTION__ << " can not be called"
            << std::endl;
       LOG_ERROR(strs);
      return false;
    }

    //std::vector<double> jnt1(DoF_, 0);
    Eigen::VectorXd jnt1(DoF_);
    
    Vec trans(pose[0], pose[1], pose[2]);
    Rotation r(pose[5], pose[4], pose[3]);
    Frame fr(r, trans);

    int flag = (int) pose[6];
    
    std::vector<int> branchFlags;
    SingleInt2RobnuxBranch(flag, branchFlags);

    std::vector<int> ikJointTurns;
    int turn = (int) pose[7];
    SingleInt2RobnuxTurn(turn, DoF_, ikJointTurns);
  
    strs.str("");
    strs << "Pose is " << pose <<  std::endl;
    for (size_t j=0; j< DoF_; j++) {
      strs << "turn " << j << " = " << ikJointTurns[j] << std::endl;
    }
    LOG_INFO(strs);


    Pose ps;
    refPose rps;

    rps.setFrame(fr);
    rps.setBranchFlags(branchFlags);
    rps.setBase(current_base_);
    rps.setTool(current_tool_);
    rps.setJointTurns(ikJointTurns); // so far only default turns, will handle complicated turns in the future
    rps.getDefaultPose(&ps); //get default pose w.r.t. default base and tool
    //strs.str("");
    //strs << "under default base, the pose is " << ps.ToString(true) << std::endl;
    //LOG_INFO(strs);

    if (armMap_->CartToJnt(ps, &jnt1) < 0) {
      return false;
    }
    /*
    jnt.resize(DoF_);
    for (size_t i=0; i < DoF_; i++) {
        jnt(i) = jnt1[i];
    }*/
    jnt = jnt1;
    return true;
  }


  }
