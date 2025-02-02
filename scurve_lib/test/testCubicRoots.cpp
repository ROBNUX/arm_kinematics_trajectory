#include "scurve_lib/computeRootsUtil.hpp"
#include <cmath>
#include <cassert>
#include <sstream>

using namespace kinematics_lib;
using namespace ROBNUXLogging;
bool check_min_positive_real_root_for_cubic_eq(double a, double b,
                                               double c, double d,
                                               double root_value) {
  std::vector<double> roots;
  if (!Compute_Roots_Utility::CubicEqRealRoots(a,  b, c, d, roots)) {
      return false;
  }
  size_t n_rts = roots.size();
  double time;
  if (n_rts == 1) {
    time = roots[0];
  } else if (n_rts ==2) {
    time =  Compute_Roots_Utility::min_positive_root2(roots[0], roots[1]);
  } else if (n_rts == 3) {
    time =  Compute_Roots_Utility::min_positive_root3(roots[0], roots[1], roots[2]);
  }
  return fabs(root_value - time) < 1e-5;
}


int main() {
    std::ostringstream strs;
    bool ret = check_min_positive_real_root_for_cubic_eq(1, -6, 11, -6, 1);
    assert(ret==true);
    strs.str("");
    strs << " first examples: ret=" << ret << std::endl;
    LOG_INFO(strs);
    ret = check_min_positive_real_root_for_cubic_eq(0, 1, -3, 2, 1);
    assert(ret==true);
    strs.str("");
    strs << " second examples: ret=" << ret << std::endl;
    LOG_INFO(strs);
    ret = check_min_positive_real_root_for_cubic_eq(1, -3, 2, 0, 1);
    assert(ret==true);
    strs.str("");
    strs << " 3rd examples: ret=" << ret << std::endl;
    LOG_INFO(strs);
    ret = check_min_positive_real_root_for_cubic_eq(1, 1, 1, -3, 1);
    assert(ret==true);
    strs.str("");
    strs << " 4th examples: ret=" << ret << std::endl;
    LOG_INFO(strs);
    ret = check_min_positive_real_root_for_cubic_eq(1, 6, 11, 6,  0);
    assert(ret==false);
    strs.str("");
    strs << " 5th examples: ret=" << ret << std::endl;
    LOG_INFO(strs);
}