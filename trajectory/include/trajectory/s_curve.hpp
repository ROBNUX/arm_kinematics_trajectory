/*
 * File:   s_curve.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 14, 2021, 10:33 PM
 */

#ifndef S_CURVE_HPP
#define S_CURVE_HPP
#include <limits>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "kdl_common/vec.hpp"
#include "scurve_lib/piecewise_function.hpp"
#include "scurve_lib/segment_planning.hpp"
#include "trajectory/base_profile.hpp"

namespace kinematics_lib {
class SCurveProfile : public BaseProfile {
 public:
  SCurveProfile();
  /*
   * @brief: set boundary conditions needed to find a time optimal profile
   */
  bool setBoundaryCondition(const double pos_start, const double pos_end,
                            const double vel_start,
                            const double vel_end) override;

  bool Trajectory(const double time, double *X, double *Xdot,
                  double *Xddot) const override;

  /*
   * @brief get time at given distance after the starting pos, i.e.,
   * inverse function of X(time), Note this applies canonical inverse
   * which means assume scale_ = 1.0
   */
  double TimeAtDist(const double dist) const override;

 private:
  /*
   *  Initial start/end pos, vel and acc
   */
  double start_pos_, start_vel_;
  double end_pos_, end_vel_;

  double lower_t_bound_, up_t_bound_;

  /*
   * piecewise functions for p(t), v(t)， a(t), and j(t)
   */
  std::vector<PiecewiseFunction> trajs_;
};

}  // namespace kinematics_lib

#endif /* S_CURVE_HPP */
