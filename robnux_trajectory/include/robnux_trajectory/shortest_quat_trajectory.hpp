/*
 * File:   shortest_euler_trajectory.hpp
 * Author: Leon Liu, ROBNUX LLC (leon@robnux.net)
 *
 * Brief:  this creates a linear interpolation between two given orientations,
 * and the corresponding angular velocity at start and goal orientations Created
 * on April 22, 2021, 5:40 PM
 */

#ifndef SHORTEST_EULER_TRAJECTORY_HPP
#define SHORTEST_EULER_TRAJECTORY_HPP
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_trajectory/base_rot_trajectory.hpp"

namespace kinematics_lib {
class TRAJECTORY_API ShortestQuatTrajectory : public BaseRotTrajectory {
 public:
  ShortestQuatTrajectory();
  ShortestQuatTrajectory(const EulerType &tp);

  /*
   * @brief set up boundary conditions, given start_pv (pos and vel),
   * and end_pv (end pos and vel)
   */
  bool setBoundaryCond(const Quaternion &start_q, const Quaternion &start_qdot,
                       const Quaternion &end_q,
                       const Quaternion &end_qdot) override;

  /*
   * what is the output Euler type
   */
  EulerType getOutputEulerType() const { return tp_; }

  /*
   *  compute orientation trajectory {rot, w, wdot}  at <time>
   */
  bool Trajectory(const double time, Rotation *rot, Vec *w,
                  Vec *wdot) const override;
  /*
   *  compute orientation trajectory in euler angles {euler,
   *  eulerdot, eulerddot}  at <time>
   */
  bool Trajectory(const double time, EulerAngle *euler, Vec *eulerdot,
                  Vec *eulerddot) const override;
  /*
   *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
   */
  bool Trajectory(const double time, Quaternion *q, Quaternion *qdot,
                  Quaternion *qddot) const override;

 private:
  //  start and end orientation,  sqdot_, and eqdot_ are their
  // first derivative
  Quaternion sq_, eq_, sqdot_, eqdot_;
  // a quaternion that represents the direction of interpolation
  Vec direction_;
  // type of Euler angles,  ZYX or ZYZ
  EulerType tp_;
};
}  // namespace kinematics_lib
#endif /* SHORTEST_EULER_TRAJECTORY_HPP */
