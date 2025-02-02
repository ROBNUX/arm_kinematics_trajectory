#include "scurve_lib/computeRootsUtil.hpp"

#include <cmath>

namespace kinematics_lib {

bool Compute_Roots_Utility::QuadEqRealRoots(const double a, const double b,
                                            const double c,
                                            std::vector<double> &roots) {
  const double dist = b * b - 4 * a * c;
  if (fabs(a) < std::numeric_limits<double>::epsilon()) {
    if (fabs(b) < std::numeric_limits<double>::epsilon()) {
      // no solution
      return false;
    } else {
      roots.push_back(-c / b);
    }
  } else if (dist > 0) {
    roots.push_back((-b - sqrt(dist)) / (2 * a));
    roots.push_back((-b + sqrt(dist)) / (2 * a));
  } else {
    return false;
  }
  return true;
}

double Compute_Roots_Utility::min_positive_root2(const double r1,
                                                 const double r2) {
  double min_rt = -1.0;
  if (r1 > 0 && r2 > 0) {
    if (r1 < r2) {
      min_rt = r1;
    } else {
      min_rt = r2;
    }
  } else if (r1 > 0) {
    min_rt = r1;
  } else if (r2 > 0) {
    min_rt = r2;
  }
  return min_rt;  // could return negative value if both r1 and r2 < 0
}

double Compute_Roots_Utility::min_positive_root3(const double r1,
                                                 const double r2,
                                                 const double r3) {
  double min_rt = -1.0;
  if (r1 > 0 && r2 > 0 && r3 > 0) {
    if (r1 < r2) {
      min_rt = min_positive_root2(r1, r3);
    } else {
      min_rt = min_positive_root2(r2, r3);
    }
  } else if (r1 > 0 && r2 > 0) {
    min_rt = min_positive_root2(r1, r2);
  } else if (r1 > 0 && r3 > 0) {
    min_rt = min_positive_root2(r1, r3);
  } else if (r2 > 0 && r3 > 0) {
    min_rt = min_positive_root2(r2, r3);
  } else if (r1 > 0) {
    min_rt = r1;
  } else if (r2 > 0) {
    min_rt = r2;
  } else if (r3 > 0) {
    min_rt = r3;
  }
  return min_rt;
}

bool Compute_Roots_Utility::CubicEqRealRoots(const double a, const double b,
                                             const double c, const double d,
                                             std::vector<double> &roots) {
  std::ostringstream strs;
  strs << __FUNCTION__ << ":" << __LINE__ << ": eq_info: a=" << a << ", b=" << b
       << ", c=" << c << ", d=" << d << std::endl;
  LOG_INFO(strs);
  roots.clear();
  if (fabs(a) < std::numeric_limits<double>::epsilon()) {
    strs.str("");
    strs << "The coefficient of the cube of x is 0. Please use the utility for "
            "a SECOND degree quadratic. No further action taken."
         << std::endl;
    LOG_INFO(strs);
    return QuadEqRealRoots(b, c, d, roots);
  }

  if (fabs(d) < std::numeric_limits<double>::epsilon()) {
    strs.str("");
    strs << "One root is 0. Now divide through by x and use the utility for a "
            "SECOND degree quadratic to solve the "
         << "resulting equation for the other two roots. No further action "
            "taken."
         << std::endl;
    LOG_INFO(strs);
    std::vector<double> subRots;
    if (!QuadEqRealRoots(a, b, c, subRots)) {
      return false;
    }

    roots.insert(roots.end(), subRots.begin(), subRots.end());
    roots.push_back(0);
    return true;
  }

  double bb = b / a;
  double cc = c / a;
  double dd = d / a;

  double q = (3.0 * cc - (bb * bb)) / 9.0;
  double r = -(27.0 * dd) + bb * (9.0 * cc - 2.0 * (bb * bb));
  r /= 54.0;
  double disc = q * q * q + r * r;
  strs.str("");
  strs << "disc= " << disc << std::endl;
  LOG_INFO(strs);
  double term1 = b / 3.0;
  if (disc > std::numeric_limits<double>::epsilon()) {  // one root real, two
                                                        // are complex
    double s = r + sqrt(disc);
    if (s < 0) {
      double mins_s = -s;
      s = -std::pow(mins_s, 1.0 / 3.0);
    } else {
      s = std::pow(s, 1.0 / 3.0);
    }
    double t = r - sqrt(disc);
    if (t < 0) {
      double mins_t = -t;
      t = -std::pow(mins_t, 1.0 / 3.0);
    } else {
      t = std::pow(t, 1.0 / 3.0);
    }
    double x1r = -term1 + s + t;
    roots.push_back(x1r);
    // term1 += (s + t) / 2.0;
    // double x3r = -term1;
    // double x2r = -term1;
    // term1 = sqrt(3.0) * (-t + s) / 2.0;
    // x2r = term1;
    // rt1 = x1r
    // rt2 = -100.0
    // rt3 = -100.0
    // return rt1, rt2, rt3, 1
    //  End if (disc > 0)
    // The remaining options are all real
  } else if (fabs(disc) <=
             std::numeric_limits<double>::epsilon()) {  // All roots real, at
                                                        // least two are equal.
    disc = 0;
    double mins_r, r13, x1r, x2r, x3r;
    if (r < 0) {
      mins_r = -r;
      r13 = -std::pow(mins_r, 1.0 / 3.0);
    } else {
      r13 = std::pow(r, 1.0 / 3.0);
    }

    x1r = -term1 + 2.0 * r13;
    x3r = -(r13 + term1);
    x2r = -(r13 + term1);
    roots.push_back(x1r);
    roots.push_back(x2r);
    roots.push_back(x3r);
    // End if (disc == 0)
  } else {
    // Only one option left is that all roots are real and unequal (to get here,
    // q < 0) print "last case: disc < 0 , disc= " print disc
    q = -q;
    double dum1 = q * q * q;
    dum1 = acos(r / sqrt(dum1));
    double r13 = 2.0 * sqrt(q);
    double x1r, x2r, x3r;
    x1r = -term1 + r13 * cos(dum1 / 3.0);
    x2r = -term1 + r13 * cos((dum1 + 2.0 * M_PI) / 3.0);
    x3r = -term1 + r13 * cos((dum1 + 4.0 * M_PI) / 3.0);
    roots.push_back(x1r);
    roots.push_back(x2r);
    roots.push_back(x3r);
  }
  return true;
}

}  // namespace kinematics_lib