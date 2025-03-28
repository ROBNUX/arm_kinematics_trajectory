/*
 * File:   PTPTrajectory.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on Sept. 4, 2021, 12:20 PM
 *
 * @brief, this file implements a PTP Trajectory object
 */
#ifndef KINEMATICS_LIB_PTP_TRAJECTORY_HPP_
#define KINEMATICS_LIB_PTP_TRAJECTORY_HPP_

#include <vector>

#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/twistWrench.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "robnux_trajectory/base_profile.hpp"
#include "robnux_trajectory/trajectory.hpp"

namespace kinematics_lib {

class PTPTrajectory : public JntTrajectory {
 public:
  PTPTrajectory(const size_t numJnts) : JntTrajectory(numJnts) {
    j_direction_.resize(numJnts_);
  }
  virtual ~PTPTrajectory() {}
  /*
   * @brief set up boundary conditions, given start_pv (pos and vel),
   * and end_pv (end pos and vel)
   */
  bool setBoundaryCond(const Eigen::VectorXd &start_jp,
                       const Eigen::VectorXd &start_jv,
                       const Eigen::VectorXd &end_jp,
                       const Eigen::VectorXd &end_jv) override;

  //! set boundary condition in Cartesian space
  bool setBoundaryCond(const Pose &start_pos, const Twist &start_vel,
                       const Pose &end_pos, const Twist &end_vel);
  //! set boundary condition in Cartesian space with translation and quaternion
  //! inputs
  bool setBoundaryCond(const Pose &start_pos, const Vec &sp_dot,
                       const Quaternion &sq_dot, const Pose &end_pos,
                       const Vec &ep_dot, const Quaternion &eq_dot);

  //! Key API to calculate Jnt (or RelJnt), JntVel, and JntAcc of this
  //! trajectory
  virtual bool Trajectory(const double time, Eigen::VectorXd *jp,
                          Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

  virtual bool RelTrajectory(const double time, Eigen::VectorXd *jp,
                             Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

 private:
  Eigen::VectorXd j_direction_;
};
}  // namespace kinematics_lib
#endif