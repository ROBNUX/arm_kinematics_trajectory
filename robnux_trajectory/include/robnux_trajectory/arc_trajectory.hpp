/*
 * File:   arc_trajectory.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 3, 2021, 12:19 PM
 */

#ifndef ARC_TRAJECTORY_HPP
#define ARC_TRAJECTORY_HPP
#include "robnux_kdl_common/common_constants.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_trajectory/base_trans_trajectory.hpp"

namespace kinematics_lib {

class ArcTrajectory : public BaseTransTrajectory {
 public:
  // Default constructor
  ArcTrajectory();
  /*
   * @brief constructors 1: using center, the xAxis, yAxis for the circle plane
   *  and radius, start_angle and end_angle (of points on the arc w.r.t. center,
   * xAxis, and yAxis)
   */
  ArcTrajectory(const Vec &center, const Vec &xAxis, const Vec &yAxis,
                const double radius, const double start_angle,
                const double end_angle);

  //! set center, axis, radius, and start/end angle
  bool SetCenterRadius(const Vec &center, const Vec &xAxis, const Vec &yAxis,
                       const double radius, const double start_angle,
                       const double end_angle);

  /*
   * @brief constuctors 2: using 3 points, default starting angle
   * being 0 (i.e. from  center to p1),
   * and default ending angle ( as defined by the vector from center to p3)
   */
  ArcTrajectory(const Vec &p1, const Vec &p2, const Vec &p3);

  //! set three points
  bool SetThreePoints(const Vec &p1, const Vec &p2, const Vec &p3);

  /*
   * @brief set up boundary conditions, given start_pv (pos and vel),
   * and end_pv (end pos and vel)
   */
  bool setBoundaryCond(const Vec &start_pos, const Vec &start_vel,
                       const Vec &end_pos, const Vec &end_vel) override;

  /*
   *  compute trajectory {Pos, Vel,  Acc} at <time>
   */
  bool Trajectory(const double time, Vec *p, Vec *pdot,
                  Vec *ppdot) const override;

  /*
   *  properties
   */
  //! center of the circle
  Vec getCenter() { return center_; }
  //! xAxis of the circle plane
  Vec getXAxis() { return xAxis_; }
  //! yAxis of the circle plane
  Vec getYAxis() { return yAxis_; }
  //! get radius
  double getRadius() { return radius_; }
  //! start angle of the arc start point
  double getStartAngle() { return start_angle_; }
  //! end angle of the arc end point
  double getEndAngle() { return end_angle_; }

 private:
  // arc center
  Vec center_, xAxis_, yAxis_;
  // start arc angle, end arc angle
  double start_angle_, end_angle_;
  // moving spd at start and end angle along the arc tangent
  double start_spd_, end_spd_;
  // arc radius
  double radius_;
};

}  // namespace kinematics_lib
#endif /* ARC_TRAJECTORY_HPP */
