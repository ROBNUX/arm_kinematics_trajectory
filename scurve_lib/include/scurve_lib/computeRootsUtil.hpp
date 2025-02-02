#ifndef KINEMATICS_LIB_COMPUTE_ROOTS_UTIL_HPP_
#define KINEMATICS_LIB_COMPUTE_ROOTS_UTIL_HPP_
#include <limits>
#include <sstream>
#include <vector>

#include "Logger/Logger.h"
#include "scurve_lib/scurve_exportdecl.h"

using namespace ROBNUXLogging;

namespace kinematics_lib {

class SCURVE_API Compute_Roots_Utility {
 public:
  /*
   * solve quadratic equation ax^2 + bx +c =0, output roots vector and number of
   * roots return true if finding real roots, and false if no real roots
   */
  static bool QuadEqRealRoots(const double a, const double b, const double c,
                              std::vector<double> &roots);

  /*
   * find minimal positive root given two roots of a quadratic equation
   */
  static double min_positive_root2(const double r1, const double r2);

  /*
   * find minimal positive root given 3 roots of cubic equation
   */
  static double min_positive_root3(const double r1, const double r2,
                                   const double r3);

  /*
   * solve cubit equation
   */
  static bool CubicEqRealRoots(const double a, const double b, const double c,
                               const double d, std::vector<double> &roots);
};

}  // namespace kinematics_lib
#endif