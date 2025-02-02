/*
 * File:   line_trajectory.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 3, 2021, 12:20 PM
 *
 * @brief, this file implements a line translational motion
 */

#ifndef LINE_TRAJECTORY_HPP
#define LINE_TRAJECTORY_HPP
#include "kdl_common/vec.hpp"
#include "trajectory/base_trans_trajectory.hpp"

namespace kinematics_lib {

class LineTrajectory : public BaseTransTrajectory {
 public:
  /*
   * default constructor
   */
  LineTrajectory();

  /*
   * @brief set up boundary conditions, given start_pv (pos and vel),
   * and end_pv (end pos and vel)
   */
  bool setBoundaryCond(const Vec &start_pos, const Vec &start_vel,
                       const Vec &end_pos, const Vec &end_vel) override;

  /*
   *  compute trajectory= {pos, vel, acc}  at <time>
   */
  bool Trajectory(const double time, Vec *p, Vec *pdot,
                  Vec *pddot) const override;

 private:
  // start and end points and velocity
  Vec start_ps_, start_vel_;
  Vec end_ps_, end_vel_;
  // normal vector from start_ps_ to end_ps_
  Vec direction_;
};
}  // namespace kinematics_lib

#endif /* LINE_TRAJECTORY_HPP */
