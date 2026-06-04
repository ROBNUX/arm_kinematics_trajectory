#ifndef ROBNUX_KINEMATICS_MAP_POE_SUBPROBLEMS_HPP_
#define ROBNUX_KINEMATICS_MAP_POE_SUBPROBLEMS_HPP_

#include <array>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Core>

namespace kinematics_lib {
namespace poe_sp {

// Default tolerances matching ssik defaults
static constexpr double SP_FEASIBILITY_TOL = 1e-6;
static constexpr double SP_DEGENERACY_TOL  = 1e-7;
static constexpr double SP_NUMERICAL_TOL   = 1e-10;

struct Sp1Result {
    double theta;
    bool   is_ls;
};

struct Sp34Result {
    std::vector<double> angles;
    bool                is_ls;
};

struct Sp2Result {
    std::vector<std::pair<double, double>> pairs;
    bool                                   is_ls;
};

struct Sp6Result {
    std::vector<std::pair<double, double>> pairs;
    bool                                   is_ls;
};

// Rodrigues 3x3 rotation matrix for unit axis and angle
Eigen::Matrix3d rotMat(const Eigen::Vector3d& axis, double angle);

// Rotate vector v about unit axis k by angle theta (Rodrigues)
Eigen::Vector3d rotate(const Eigen::Vector3d& k, double angle, const Eigen::Vector3d& v);

// Wrap angle to (-pi, pi]
double wrapToPi(double a);

// SP1: Rot(k, theta) @ p == q  (exact) or least-squares minimum
Sp1Result sp1(const Eigen::Vector3d& k,
              const Eigen::Vector3d& p,
              const Eigen::Vector3d& q,
              double feas_tol = SP_FEASIBILITY_TOL);

// SP2: Rot(k1, t1) @ p == Rot(k2, t2) @ q  (up to 2 solutions)
Sp2Result sp2(const Eigen::Vector3d& k1,
              const Eigen::Vector3d& k2,
              const Eigen::Vector3d& p,
              const Eigen::Vector3d& q,
              double deg_tol  = SP_DEGENERACY_TOL,
              double feas_tol = SP_FEASIBILITY_TOL);

// SP4: h . (Rot(k, theta) @ p) == d  (up to 2 solutions)
Sp34Result sp4(const Eigen::Vector3d& h,
               const Eigen::Vector3d& k,
               const Eigen::Vector3d& p,
               double d,
               double feas_tol = SP_FEASIBILITY_TOL,
               double deg_tol  = SP_DEGENERACY_TOL);

// SP3: |Rot(k, theta) @ p - q| == d  (delegates to SP4)
Sp34Result sp3(const Eigen::Vector3d& k,
               const Eigen::Vector3d& p,
               const Eigen::Vector3d& q,
               double d,
               double feas_tol = SP_FEASIBILITY_TOL,
               double deg_tol  = SP_DEGENERACY_TOL);

// SP6: two coupled SP4-like equations in two unknowns (up to 4 solutions)
//   h[0].dot(Rot(k[0],t1)*p[0]) + h[1].dot(Rot(k[1],t2)*p[1]) == d1
//   h[2].dot(Rot(k[2],t1)*p[2]) + h[3].dot(Rot(k[3],t2)*p[3]) == d2
Sp6Result sp6(const std::array<Eigen::Vector3d, 4>& h,
              const std::array<Eigen::Vector3d, 4>& k,
              const std::array<Eigen::Vector3d, 4>& p,
              double d1,
              double d2,
              double deg_tol = SP_DEGENERACY_TOL,
              double num_tol = SP_NUMERICAL_TOL);

}  // namespace poe_sp
}  // namespace kinematics_lib

#endif  // ROBNUX_KINEMATICS_MAP_POE_SUBPROBLEMS_HPP_
