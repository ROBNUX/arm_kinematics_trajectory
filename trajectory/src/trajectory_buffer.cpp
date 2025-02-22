#include "trajectory/trajectory_buffer.hpp"

#include "simple_motion_logger/Logger.h"

using namespace ROBNUXLogging;
namespace kinematics_lib {
TrajectoryBuffer::TrajectoryBuffer(
    const boost::shared_ptr<BaseKinematicMap>& armMap)
    : cur_cmd_(nullptr),
      cur_traj_(nullptr),
      next_traj_(nullptr),
      state_(0),
      exe_state_(0),
      errorCode_(0),
      exec_errorCode_(0),
      traj_exec_time_(0.0),
      armMap_(armMap) {
  DoF_ = armMap_->GetActDoF();
}

TrajectoryBuffer::~TrajectoryBuffer() { ResetBuffer(); }

bool TrajectoryBuffer::AddCommand(const std::shared_ptr<MotionCommand>& cmd) {
  if (cmd) {
    mtx_cmd_.lock();
    cmdBuffer_.push(cmd);
    mtx_cmd_.unlock();
    return true;
  }
  return false;
}

void TrajectoryBuffer::ProcessCommandBuffer() {
  // start/end condition of current traj segment
  std::ostringstream ss;
  Twist start_vel = Twist::Zero();
  Twist end_vel = Twist::Zero();

  // frames at start and end point of the segment
  Pose sps, eps;  // start and end pose for the current segment

  // cur_cmd_ translate into cart_traj, and push into traj buffer
  std::shared_ptr<CartTrajectory> cart_traj = NULL;
  // cart_traj could be a TrajectorySegment or a PTP traj
  std::shared_ptr<TrajectorySegment> seg_cur = NULL;

  // current translational traj. and rotational traj. scurve profile, as well as
  // joint scurve profile
  std::shared_ptr<SCurveProfile> scurve_t_cur = NULL, scurve_r_cur = NULL,
                                 scurve_j_cur;

  // new Joint profile vector
  std::vector<std::shared_ptr<BaseProfile>> jnt_prof;
  std::vector<JntProfile> jnt_pf_data;  // joint prof data

  // current rotational traj
  std::shared_ptr<ShortestQuatTrajectory> traj_cur_r =
      NULL;                    //--> gets into part of TrajectorySegment
  double cur_approx_dist;      // current approx distance
  Motion_Command_Type cur_ID;  // current and next command id;

  // other temporary variables
  std::chrono::duration<double> difference;
  // two boolean flag for determining if waiting for buffering gets expired
  bool b1, b2;

  // cartesian profile data of current command
  ProfileData pf;
  // temp. variable for creating arc trajectory
  Frame ctFrame;
  Vec center, xAxis, yAxis;
  double radius, start_angle, end_angle;

  // arc/ptp commands
  std::shared_ptr<ArcMotionCommand> arc_cmd = NULL;
  std::shared_ptr<PTPMotionCommand> ptp_cmd = NULL;

  // temp. variables for creating translating trajectory component
  std::shared_ptr<BaseTransTrajectory> traj_cur_t =
      NULL;  //--> gets into part of TrajectorySegment
  std::shared_ptr<PTPTrajectory> ptp_traj = NULL;
  bool isCmdblock = false;
  double lduration = 0;

  mtx_cmd_.lock();
  if (errorCode_ <
      0) {  // once error, need manually reset error, and then can continue
    ss.str("");
    ss << "errorcode = " << errorCode_ << ", in function " << __FUNCTION__
       << " at line " << __LINE__ << std::endl;
    LOG_ERROR(ss);
    return;
  }
  switch (state_) {
    case 0:
      if (cmdBuffer_.size() > 0) {
        stamp_ = std::chrono::system_clock::now();
        state_++;
      }
      break;
    case 1:
      difference = std::chrono::system_clock::now() - stamp_;
      b1 = static_cast<int>(cmdBuffer_.size()) < INIT_CMD_BUFF_SIZE;
      b2 = difference.count() > CMD_BUFF_WAIT_TIME_LIMIT;
      isCmdblock = trajBuffer_.size() >=
                   MAX_TRAJ_BUFF_SIZE;  // if buffer size is > 5, then cmd
                                        // buffer shall be locked
      if (!isCmdblock) {
        if ((b1 && b2) || !b1) {
          state_++;
        }
      }
      break;
    case 2:
      if (cmdBuffer_.size() > 0) {
        // obtain the current command and then set up the default boundary
        // conditions
        cur_cmd_ = cmdBuffer_.front();
        cmdBuffer_.pop();

        // start boundary conditions
        // if starting from current segment, then  sp is the starting point
        // of the current segment, spdot, and spddot are zeros, similarly sq
        // is the starting orientation, and sqdot, sqddot must be 0
        sps = cur_cmd_->GetStartPose();
        eps = cur_cmd_->GetDestPose();

        // step 2: get this segment ID, and approx dist
        cur_approx_dist = cur_cmd_->GetApproxDistance();  // this works for both
                                                          // cart or ptp command
        cur_ID = cur_cmd_->GetCMDID();
        if (cur_ID == ID_PTP) {
          std::shared_ptr<PTPMotionCommand> ptp_cmd =
              dynamic_pointer_cast<PTPMotionCommand>(cur_cmd_);
          if (ptp_cmd) {
            jnt_pf_data = *ptp_cmd->GetJntProfile();
          } else {
            errorCode_ = -2000;  // ptp_cmd dynamic_cast failure
            return;
          }

          for (size_t i = 0; i < DoF_; i++) {
            // compute joint profile vector based on joint profile data
            scurve_j_cur = std::make_shared<SCurveProfile>();
            scurve_j_cur->setConstraints(jnt_pf_data[i].max_vel_,
                                         jnt_pf_data[i].max_acc_,
                                         jnt_pf_data[i].max_jerk_);
            jnt_prof.push_back(scurve_j_cur);
          }
          // for PTP command, we suppose that joint prof doesn't impose
          // any limits on cartesian profile
          pf.max_vel_t_ = std::numeric_limits<double>::max();
          pf.max_acc_t_ = std::numeric_limits<double>::max();
          pf.max_jerk_t_ = std::numeric_limits<double>::max();
          pf.max_vel_r_ = std::numeric_limits<double>::max();
          pf.max_acc_r_ = std::numeric_limits<double>::max();
          pf.max_jerk_r_ = std::numeric_limits<double>::max();
          ptp_traj = std::make_shared<PTPTrajectory>(DoF_);
          ptp_traj->setKinematicMap(armMap_);
          ptp_traj->setProfile(jnt_prof);
          if (!ptp_traj->setBoundaryCond(sps, start_vel, eps, end_vel)) {
            errorCode_ = -ERR_TRAJ_SET_BOUND;
            return;
          }
          cart_traj = ptp_traj;  // recall ptp_traj is also a type of cart_traj
        } else {                 // means cartesian type traj
          // create cur segment translational and rotational profiles
          pf = cur_cmd_->GetProfile();
          scurve_t_cur = std::make_shared<SCurveProfile>();
          scurve_t_cur->setConstraints(pf.max_vel_t_, pf.max_acc_t_,
                                       pf.max_jerk_t_);
          scurve_r_cur = std::make_shared<SCurveProfile>();
          scurve_r_cur->setConstraints(pf.max_vel_r_, pf.max_acc_r_,
                                       pf.max_jerk_r_);

          traj_cur_r = std::make_shared<ShortestQuatTrajectory>(eZYX);
          traj_cur_r->setProfile(scurve_r_cur);

          switch (cur_ID) {
            case ID_LINE:
              traj_cur_t = std::make_shared<LineTrajectory>();
              break;
            case ID_ARC:
              arc_cmd = std::dynamic_pointer_cast<ArcMotionCommand>(cur_cmd_);
              if (!arc_cmd) {
                errorCode_ = -ERR_TRAJ_COMMAND_WRONG_TYPE;
                return;
              }

              ctFrame = arc_cmd->GetArcCenter();
              xAxis = ctFrame.getRotation().UnitX();
              yAxis = ctFrame.getRotation().UnitY();
              center = ctFrame.getTranslation();
              radius = arc_cmd->GetArcRadius();
              start_angle = arc_cmd->GetArcStartAng();
              end_angle = arc_cmd->GetArcEndAng();
              traj_cur_t = std::make_shared<ArcTrajectory>(
                  center, xAxis, yAxis, radius, start_angle, end_angle);
              break;
            default:
              errorCode_ = -ERR_TRAJ_COMMAND_OUT_OF_SCOPE;
              return;
              break;
          }

          if (traj_cur_t) {
            traj_cur_t->setProfile(scurve_t_cur);
          } else {
            errorCode_ = -ERR_TRAJ_COMMAND_GENERATE_TRANSLATION_FAILS;
            return;
          }

          // finally create the entire current trajectory
          seg_cur = std::make_shared<TrajectorySegment>();
          seg_cur->setKinematicMap(armMap_);
          if (!seg_cur->InitializeTrajectoryPlanners(
                  traj_cur_t, traj_cur_r)) {  // if init fails
            errorCode_ = -ERR_TRAJ_COMMAND_GENERATE_ENTIRE_TRAJ_SEG;
            return;
          }

          if (!seg_cur->setBoundCond(sps, start_vel, eps, end_vel)) {
            errorCode_ = -ERR_TRAJ_GENERATE_SYNC_ERROR;
            return;
          }
          cart_traj = seg_cur;
          lduration = cart_traj->Duration();
        }
        cart_traj->SetTailBlendDist(cur_approx_dist);
        mtx_traj_.lock();
        trajBuffer_.push(cart_traj);
        mtx_traj_.unlock();
        motionline_element_t lml_;
        lml_.duration = lduration;
        lml_.line = cur_cmd_->getScriptLine();
        lmlmapper_.addmotionline(lml_);
        if (cur_cmd_) {
          cur_cmd_.reset();
          cur_cmd_ = nullptr;
        }
        state_++;
      }
      break;
    case 3:
      state_ = 0;
      break;
  }
  mtx_cmd_.unlock();
}

void TrajectoryBuffer::ResetBuffer() {
  mtx_cmd_.lock();
  std::queue<std::shared_ptr<MotionCommand>> cmdBuffer_empty;
  std::swap(cmdBuffer_empty, cmdBuffer_);
  lmlmapper_.reset();
  if (cur_cmd_) {
    cur_cmd_.reset();
    cur_cmd_ = nullptr;
  }
  // shall we set execute state to 0?
  state_ = 0;
  mtx_cmd_.unlock();

  mtx_traj_.lock();
  std::queue<std::shared_ptr<CartTrajectory>> trajBuffer_empty;
  std::swap(trajBuffer_empty, trajBuffer_);
  if (cur_traj_) {
    cur_traj_.reset();
    cur_traj_ = nullptr;
  }
  if (next_traj_) {
    next_traj_.reset();
    next_traj_ = nullptr;
  }
  traj_exec_time_ = 0.0;
  exe_state_ = 0;
  mtx_traj_.unlock();
}

bool TrajectoryBuffer::ExecuteTrajBuffer(const double d_time,
                                         Eigen::VectorXd* jp,
                                         Eigen::VectorXd* jv,
                                         Eigen::VectorXd* ja) {
  std::ostringstream ss;
  // other temporary variables
  std::chrono::duration<double> difference;
  // two boolean flag for determining if waiting for buffering gets expired
  bool b1, b2;
  // Eigen::VectorXd first_jp, first_jv, first_ja;
  Eigen::VectorXd rel_jp = Eigen::VectorXd::Zero(DoF_),
                  rel_jv = Eigen::VectorXd::Zero(DoF_),
                  rel_ja = Eigen::VectorXd::Zero(DoF_);
  double blend_start_time, cur_duration;
  double blend_time = 0;

  // the following part are for executing the trajectory
  // if cur_exec_traj_ is not null
  mtx_traj_.lock();
  switch (exe_state_) {
    case 0:
      if (!cur_traj_) {
        if (!trajBuffer_.empty()) {
          cur_traj_ = trajBuffer_.front();
          trajBuffer_.pop();
          exe_state_++;
        }
      }
      break;
    case 1:
      if (cur_traj_->IsTailBlended()) {
        exe_state_++;
      } else {
        exe_state_ = 4;  // go to execute state
      }
      break;
    case 2:
      difference = std::chrono::system_clock::now() - stamp_;
      b1 = trajBuffer_.empty();
      b2 = difference.count() > TRAJ_BUFF_WAIT_TIME_LIMIT;
      if ((b1 && b2) || !b1) {
        exe_state_++;
      }
      break;
    case 3:
      if (!trajBuffer_.empty()) {
        next_traj_ = trajBuffer_.front();
        trajBuffer_.pop();
      }
      exe_state_++;
      break;
    case 4:
      if (cur_traj_) {
        cur_duration = cur_traj_->Duration();
        blend_start_time = cur_traj_->getTailBlendTime();
        blend_time = cur_duration - blend_start_time;
        cur_traj_->Trajectory(traj_exec_time_, jp, jv, ja);

        ss.str("");
        ss << "before blending: first_jp=" << *jp
           << ", cur_duration=" << cur_duration << std::endl;
        LOG_DEBUG(ss);
        if (next_traj_ && traj_exec_time_ >= blend_start_time) {
          next_traj_->RelTrajectory(traj_exec_time_ - blend_start_time, &rel_jp,
                                    &rel_jv, &rel_ja);

          (*jp) += rel_jp;
          (*jv) += rel_jv;
          (*ja) += rel_ja;
        }

        ss.str("");
        ss << "all jp=" << *jp << ", rel_jp=" << rel_jp
           << ", current_exe_time=" << traj_exec_time_
           << ", blend_start_time=" << blend_start_time
           << ", blend_time=" << blend_time << ", d_tim=" << d_time
           << std::endl;
        LOG_DEBUG(ss);

        if (traj_exec_time_ <= cur_duration) {
          traj_exec_time_ += d_time;
          mtx_traj_.unlock();
          return true;  // means out jnt will be
        } else if (next_traj_) {
          cur_traj_ = next_traj_;
          if (cur_traj_->IsTailBlended() && !trajBuffer_.empty()) {
            next_traj_ = trajBuffer_.front();
            trajBuffer_.pop();
          } else {
            next_traj_ = nullptr;
          }
          traj_exec_time_ -= cur_duration;
          traj_exec_time_ +=
              blend_time;  // because if there is blending betwee cur_traj and
                           // next_traj, then when first finishes, the second
                           // already been executed blend time
        } else {
          exe_state_ = 0;
          cur_traj_ = nullptr;
          traj_exec_time_ -= cur_duration;
        }
      }
      break;
  }
  mtx_traj_.unlock();
  return false;
}

void TrajectoryBuffer::resetFault() {
  // fprintf(stdout,"%s::%d",__func__,__LINE__);
  errorCode_ = 0;
  mtx_cmd_.unlock();
}

motionlinemapper* TrajectoryBuffer::getml() { return &this->lmlmapper_; }

}  // namespace kinematics_lib
