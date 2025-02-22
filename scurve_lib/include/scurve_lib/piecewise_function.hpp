#ifndef KINEMATICS_LIB_PIECEWISE_FUNCTION_HPP_
#define KINEMATICS_LIB_PIECEWISE_FUNCTION_HPP_
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include "simple_motion_logger/Logger.h"
#include "scurve_lib/computeRootsUtil.hpp"

using namespace ROBNUXLogging;

namespace kinematics_lib {
/*
 * object to represent polynomials  a0 + a1t + a2t^2 +...+ an t^n
 */
class SCURVE_API PolyNomials {
 public:
  PolyNomials();
  PolyNomials(const int max_degree);
  PolyNomials(const std::vector<double> coef);

  //! Assignment operator. The normal copy by value semantics.
  PolyNomials& operator=(const PolyNomials& arg);
  PolyNomials Integrate(
      const double inte_constants);  // int_constans + \int (a0 + a1t + ... ) dt
  double evaluate(const double in_value) const;
  bool TimeAtDist(const double in_value, double& time) const;
  std::vector<double> GetCoef() const;
  int GetMaxDegree() const;

 private:
  std::vector<double> coef_;
  int max_degree_;
};

/*
 * object to produce piecewise curve based upon defined functions on each
 * interval and interval boundaries
 */

class PiecewiseFunction {
 public:
  PiecewiseFunction(const std::vector<double>& boundaries,
                    const std::vector<PolyNomials>& functions);

  double evaluate(const double in_value) const;

  //! Assignment operator. The normal copy by value semantics.
  PiecewiseFunction& operator=(const PiecewiseFunction& arg);

  bool TimeAtDist(const double in_value, double& time) const;

  bool extend(const std::vector<double>& boundaries,
              const std::vector<PolyNomials>& functions);

  bool sample(const int npoints, std::vector<double>& values);

  PiecewiseFunction Integrate(const double inte_constants);

  bool duration(double& duration_time);

  bool lowest_t_bound(double& low_bound);

  bool largest_t_bound(double& up_bound);

  std::vector<double> GetBoundary() const { return boundaries_; }

  std::vector<PolyNomials> GetFunction() const { return functions_; }

 private:
  std::vector<double> boundaries_;
  std::vector<PolyNomials> functions_;
  double duration_, dist_, lowest_bound_, largest_bound_;
  double sign_;
  bool initialized_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_PIECEWISE_FUNCTION_HPP_ */
