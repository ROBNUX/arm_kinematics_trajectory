/*
 * File:   trajectory_buffer.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 14, 2021, 10:33 PM
 */

#ifndef KINEMATICS_LIB_TRAJECTORY_BUFFER_
#define KINEMATICS_LIB_TRAJECTORY_BUFFER_
#include <algorithm>
#include <chrono>
#include <mutex>  // std::mutex
#include <queue>  // std::list

#include "trajectory/PTPTrajectory.hpp"
#include "trajectory/arc_trajectory.hpp"
#include "trajectory/line_trajectory.hpp"
#include "trajectory/motionlinemapper.h"
#include "trajectory/move_command.hpp"
#include "trajectory/s_curve.hpp"
#include "trajectory/shortest_quat_trajectory.hpp"
#include "trajectory/trajectory.hpp"
#include "trajectory/trajectory_segment.hpp"

namespace kinematics_lib {
/*
 * next define command buffer
 */
class TRAJECTORY_API TrajectoryBuffer {
 public:
  //! default constructor
  TrajectoryBuffer(const boost::shared_ptr<BaseKinematicMap> &armMap);

  ~TrajectoryBuffer();

  /*
   * add command into the buffer
   */
  bool AddCommand(const std::shared_ptr<MotionCommand> &cmd);

  /*
   *  Process command buffer and translate them into trajectory, and push them
   * into traj buffer
   */
  void ProcessCommandBuffer();

  /*
   *  check if commandBuffer full
   */
  bool IsCommandBufferFull() { return cmdBuffer_.size() > INIT_CMD_BUFF_SIZE; }

  bool IsCommandBufferEmpty() { return cmdBuffer_.empty(); }
  /*
   * check if trajBuffer_ is empty
   */
  bool IsTrajBufferEmpty() {
    return trajBuffer_.empty() && !cur_traj_ && !next_traj_;
  }

  /*
   * execute Traj Buffer and modify time if needed
   */
  bool ExecuteTrajBuffer(const double d_time, Eigen::VectorXd *jp,
                         Eigen::VectorXd *jv, Eigen::VectorXd *ja);

  /*
   * Get current state
   */
  int GetCurrentState() { return state_; }

  /*
   * reset traj. buffer
   */
  void ResetBuffer();

  /*
   * reset error
   */
  void resetFault();

  /*
   * get code line information
   */
  motionlinemapper *getml();

 private:
  // mutex for locking AddCommand and RetrieveTrajectory
  std::mutex mtx_cmd_, mtx_traj_;
  // input command buffer
  std::queue<std::shared_ptr<MotionCommand> > cmdBuffer_;
  // output trajectory buffer
  std::queue<std::shared_ptr<CartTrajectory> > trajBuffer_;
  // current, and next command
  std::shared_ptr<MotionCommand> cur_cmd_;
  // immediate last trajectory
  std::shared_ptr<CartTrajectory> cur_traj_, next_traj_;

  // state machine states
  int state_, exe_state_;
  // error code
  int errorCode_, exec_errorCode_;

  // Degrees of freedom
  size_t DoF_;

  // time stamp
  std::chrono::time_point<std::chrono::system_clock> stamp_, exec_stamp_;

  // line info obj
  motionlinemapper lmlmapper_;

  // clock for executing the traj buffer
  double traj_exec_time_;

  // set armMap_ so that we can apply IK/FK
  boost::shared_ptr<BaseKinematicMap> armMap_;
};

}  // namespace kinematics_lib
#endif /* KINEMATICS_LIB_TRAJECTORY_BUFFER_ */
