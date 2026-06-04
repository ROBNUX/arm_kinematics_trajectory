#include "robnux_kinematics_map/poe_subproblems.hpp"

#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

namespace kinematics_lib {
namespace poe_sp {

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

Eigen::Matrix3d rotMat(const Eigen::Vector3d& axis, double angle) {
    const double c  = std::cos(angle);
    const double s  = std::sin(angle);
    const double oc = 1.0 - c;
    const double x  = axis.x(), y = axis.y(), z = axis.z();
    Eigen::Matrix3d R;
    R << c + x*x*oc,     x*y*oc - z*s,  x*z*oc + y*s,
         y*x*oc + z*s,   c + y*y*oc,    y*z*oc - x*s,
         z*x*oc - y*s,   z*y*oc + x*s,  c + z*z*oc;
    return R;
}

Eigen::Vector3d rotate(const Eigen::Vector3d& k, double angle, const Eigen::Vector3d& v) {
    const double c  = std::cos(angle);
    const double s  = std::sin(angle);
    const double kv = k.dot(v);
    return v * c + k.cross(v) * s + k * (kv * (1.0 - c));
}

double wrapToPi(double a) {
    return std::fmod(a + M_PI, 2.0 * M_PI) - M_PI;
}

// ---------------------------------------------------------------------------
// SP1: Rot(k, theta) @ p == q
// ---------------------------------------------------------------------------

Sp1Result sp1(const Eigen::Vector3d& k,
              const Eigen::Vector3d& p,
              const Eigen::Vector3d& q,
              double feas_tol) {
    const double kp    = k.dot(p);
    const double kq    = k.dot(q);
    const double theta = std::atan2(k.cross(p).dot(q), p.dot(q) - kp * kq);

    const double p_perp_sq = p.squaredNorm() - kp * kp;
    const double q_perp_sq = q.squaredNorm() - kq * kq;
    const bool   is_ls     = (std::abs(p_perp_sq - q_perp_sq) > feas_tol ||
                               std::abs(kp - kq) > feas_tol);
    return {theta, is_ls};
}

// ---------------------------------------------------------------------------
// SP2: Rot(k1, t1) @ p == Rot(k2, t2) @ q  (up to 2 solutions)
// ---------------------------------------------------------------------------

Sp2Result sp2(const Eigen::Vector3d& k1,
              const Eigen::Vector3d& k2,
              const Eigen::Vector3d& p,
              const Eigen::Vector3d& q,
              double deg_tol,
              double feas_tol) {
    const double c    = k1.dot(k2);
    const double s_sq = 1.0 - c * c;

    if (s_sq < deg_tol) {
        // Parallel axes: only their sum/diff is determined; canonical: t2 = 0
        auto r1 = sp1(k1, p, q, feas_tol);
        return {{{r1.theta, 0.0}}, true};
    }

    const double d1    = k1.dot(p);
    const double d2    = k2.dot(q);
    const double alpha = (d1 - c * d2) / s_sq;
    const double beta  = (d2 - c * d1) / s_sq;
    const Eigen::Vector3d kxk = k1.cross(k2);

    const double pp            = p.squaredNorm();
    const double qq            = q.squaredNorm();
    const double z_sq_target   = 0.5 * (pp + qq);
    const double gamma_sq_sc   = z_sq_target - alpha*alpha - beta*beta - 2.0*alpha*beta*c;

    const bool mag_mismatch     = std::abs(pp - qq) > feas_tol;
    const bool infeasible_sph   = gamma_sq_sc < -feas_tol;
    const bool is_ls            = mag_mismatch || infeasible_sph;

    if (gamma_sq_sc <= 0.0) {
        Eigen::Vector3d z = alpha * k1 + beta * k2;
        double t1 = sp1(k1, p, z, feas_tol).theta;
        double t2 = sp1(k2, q, z, feas_tol).theta;
        return {{{t1, t2}}, is_ls};
    }

    const double gamma = std::sqrt(gamma_sq_sc / s_sq);
    Eigen::Vector3d z_a = alpha * k1 + beta * k2 + gamma * kxk;
    Eigen::Vector3d z_b = alpha * k1 + beta * k2 - gamma * kxk;

    double t1a = sp1(k1, p, z_a, feas_tol).theta;
    double t2a = sp1(k2, q, z_a, feas_tol).theta;
    double t1b = sp1(k1, p, z_b, feas_tol).theta;
    double t2b = sp1(k2, q, z_b, feas_tol).theta;
    return {{{t1a, t2a}, {t1b, t2b}}, is_ls};
}

// ---------------------------------------------------------------------------
// SP4: h . (Rot(k, theta) @ p) == d  (up to 2 solutions)
// ---------------------------------------------------------------------------

Sp34Result sp4(const Eigen::Vector3d& h,
               const Eigen::Vector3d& k,
               const Eigen::Vector3d& p,
               double d,
               double feas_tol,
               double deg_tol) {
    const double hp    = h.dot(p);
    const double kp    = k.dot(p);
    const double hk    = h.dot(k);
    const double coef_a = hp - kp * hk;
    const double coef_b = h.dot(k.cross(p));
    const double coef_c = kp * hk;
    const double r_sq   = coef_a * coef_a + coef_b * coef_b;

    const double deg_sq = deg_tol * deg_tol;
    if (r_sq < deg_sq) {
        return {{0.0}, std::abs(coef_c - d) > feas_tol};
    }

    const double r   = std::sqrt(r_sq);
    const double rhs = d - coef_c;
    const double phi = std::atan2(coef_b, coef_a);

    if (std::abs(rhs) - r > feas_tol) {
        double theta = (rhs > 0.0) ? phi : phi + M_PI;
        return {{theta}, true};
    }

    const double rhs_over_r  = rhs / r;
    const double rhs_clipped = std::max(-1.0, std::min(1.0, rhs_over_r));
    const double delta        = std::acos(rhs_clipped);
    if (delta < feas_tol) {
        return {{phi}, false};
    }
    return {{phi + delta, phi - delta}, false};
}

// ---------------------------------------------------------------------------
// SP3: |Rot(k, theta) @ p - q| == d  (delegates to SP4)
// ---------------------------------------------------------------------------

Sp34Result sp3(const Eigen::Vector3d& k,
               const Eigen::Vector3d& p,
               const Eigen::Vector3d& q,
               double d,
               double feas_tol,
               double deg_tol) {
    const double target = 0.5 * (p.squaredNorm() + q.squaredNorm() - d * d);
    return sp4(q, k, p, target, feas_tol, deg_tol);
}

// ---------------------------------------------------------------------------
// SP6 helpers: Bezout quartic conic intersection
// ---------------------------------------------------------------------------

static std::vector<double> quarticRealRoots(
    double z4, double z3, double z2, double z1, double z0,
    double imag_tol)
{
    if (std::abs(z4) < 1e-15) return {};

    const double a = z3 / z4;
    const double b = z2 / z4;
    const double c = z1 / z4;
    const double d = z0 / z4;

    // Companion matrix of the monic quartic x^4 + a x^3 + b x^2 + c x + d
    Eigen::Matrix4d C;
    C << 0, 0, 0, -d,
         1, 0, 0, -c,
         0, 1, 0, -b,
         0, 0, 1, -a;

    Eigen::EigenSolver<Eigen::Matrix4d> es(C, false);
    const auto& ev = es.eigenvalues();

    std::vector<double> roots;
    roots.reserve(4);
    for (int i = 0; i < 4; ++i) {
        if (std::abs(ev[i].imag()) < imag_tol) {
            roots.push_back(ev[i].real());
        }
    }
    return roots;
}

static std::vector<std::pair<double, double>> solveDecoupled(
    double c_coef, double e_coef, double f_coef,
    double a2, double b2, double c2, double d2, double e2, double f2,
    double epsilon,
    bool primary_is_y)
{
    // primary equation: c y^2 + e y + f = 0  (y-primary) or  a x^2 + d x + f = 0 (x-primary)
    std::vector<std::pair<double, double>> sols;

    auto quadRoots = [&](double A, double B, double C) -> std::vector<double> {
        std::vector<double> r;
        if (std::abs(A) < epsilon) {
            if (std::abs(B) < epsilon) return r;
            r.push_back(-C / B);
            return r;
        }
        double disc = B * B - 4.0 * A * C;
        if (disc < -epsilon) return r;
        double rd = std::sqrt(std::max(0.0, disc));
        r.push_back((-B + rd) / (2.0 * A));
        r.push_back((-B - rd) / (2.0 * A));
        return r;
    };

    if (primary_is_y) {
        auto primary_roots = quadRoots(c_coef, e_coef, f_coef);
        for (double y : primary_roots) {
            // a2 x^2 + (b2*y + d2) x + (c2*y^2 + e2*y + f2) = 0
            auto xr = quadRoots(a2, b2 * y + d2, c2 * y * y + e2 * y + f2);
            for (double x : xr) sols.push_back({x, y});
        }
    } else {
        // primary: c_coef=a, e_coef=d (coef of x), f_coef=f
        auto primary_roots = quadRoots(c_coef, e_coef, f_coef);
        for (double x : primary_roots) {
            // c2 y^2 + (b2*x + e2) y + (a2*x^2 + d2*x + f2) = 0
            auto yr = quadRoots(c2, b2 * x + e2, a2 * x * x + d2 * x + f2);
            for (double y : yr) sols.push_back({x, y});
        }
    }
    return sols;
}

static std::vector<std::pair<double, double>> solveTwoEllipse(
    const Eigen::Vector2d& xm1, const Eigen::Matrix<double, 2, 2>& xn1,
    const Eigen::Vector2d& xm2, const Eigen::Matrix<double, 2, 2>& xn2,
    double epsilon)
{
    // Conic 1: |xm1 + xn1 @ [xi, eta]|^2 = 1
    //          → a xi^2 + b xi eta + c eta^2 + d xi + e eta + f = 0
    Eigen::Matrix2d A1 = xn1.transpose() * xn1;
    double a  = A1(0, 0);
    double b  = 2.0 * A1(1, 0);
    double c  = A1(1, 1);
    Eigen::Vector2d B1 = 2.0 * (xm1.transpose() * xn1);
    double d  = B1(0);
    double e  = B1(1);
    double f  = xm1.squaredNorm() - 1.0;

    // Conic 2
    Eigen::Matrix2d A2 = xn2.transpose() * xn2;
    double a1 = A2(0, 0);
    double b1 = 2.0 * A2(1, 0);
    double c1 = A2(1, 1);
    Eigen::Vector2d B2 = 2.0 * (xm2.transpose() * xn2);
    double d1 = B2(0);
    double e1 = B2(1);
    double f1 = xm2.squaredNorm() - 1.0;

    // Degenerate-conic fallbacks
    if (std::abs(a) < epsilon && std::abs(b) < epsilon && std::abs(d) < epsilon) {
        return solveDecoupled(c, e, f, a1, b1, c1, d1, e1, f1, epsilon, true);
    }
    if (std::abs(c) < epsilon && std::abs(b) < epsilon && std::abs(e) < epsilon) {
        return solveDecoupled(a, d, f, a1, b1, c1, d1, e1, f1, epsilon, false);
    }
    if (std::abs(a1) < epsilon && std::abs(b1) < epsilon && std::abs(d1) < epsilon) {
        auto s = solveDecoupled(c1, e1, f1, a, b, c, d, e, f, epsilon, true);
        // outputs are (x, y) where the primary y was solved, swap to (x, y)
        return s;
    }
    if (std::abs(c1) < epsilon && std::abs(b1) < epsilon && std::abs(e1) < epsilon) {
        return solveDecoupled(a1, d1, f1, a, b, c, d, e, f, epsilon, false);
    }

    // General case: Bezout resultant → quartic in eta (y)
    double z0 = f  * a * d1*d1 + a*a * f1*f1 - d*a*d1*f1 + a1*a1*f*f - 2.0*a*f1*a1*f
               - d*d1*a1*f + a1*d*d*f1;
    double z1 = e1*d*d*a1 - f1*d1*a*b - 2.0*a*f1*a1*e - f*a1*b1*d
               + 2.0*d1*b1*a*f + 2.0*e1*f1*a*a + d1*d1*a*e - e1*d1*a*d
               - 2.0*a*e1*a1*f - f*a1*d1*b + 2.0*f*e*a1*a1 - f1*b1*a*d
               - e*a1*d1*d + 2.0*f1*b*a1*d;
    double z2 = e1*e1*a*a + 2.0*c1*f1*a*a - e*a1*d1*b + f1*a1*b*b - e*a1*b1*d
               - f1*b1*a*b - 2.0*a*e1*a1*e + 2.0*d1*b1*a*e - c1*d1*a*d
               - 2.0*a*c1*a1*f + b1*b1*a*f + 2.0*e1*b*a1*d + e*e*a1*a1
               - c*a1*d1*d - e1*b1*a*d + 2.0*f*c*a1*a1 - f*a1*b1*b
               + c1*d*d*a1 + d1*d1*a*c - e1*d1*a*b - 2.0*a*f1*a1*c;
    double z3 = -2.0*a*a1*c*e1 + e1*a1*b*b + 2.0*c1*b*a1*d - c*a1*b1*d
               + b1*b1*a*e - e1*b1*a*b - 2.0*a*c1*a1*e - e*a1*b1*b
               - c1*b1*a*d + 2.0*e1*c1*a*a + 2.0*e*c*a1*a1 - c*a1*d1*b
               + 2.0*d1*b1*a*c - c1*d1*a*b;
    double z4 = a*a*c1*c1 - 2.0*a*c1*a1*c + a1*a1*c*c - b*a*b1*c1
               - b*b1*a1*c + b*b*a1*c1 + c*a*b1*b1;

    auto y_roots = quarticRealRoots(z4, z3, z2, z1, z0, epsilon);

    std::vector<std::pair<double, double>> solutions;
    for (double y : y_roots) {
        // Back-substitute x linearly from conic 1
        double num = -((a * c1 * y*y + a * f1) - a1 * c * y*y
                       + a * e1 * y - a1 * e * y - a1 * f);
        double den = (a * b1 * y + a * d1) - a1 * b * y - a1 * d;
        if (std::abs(den) < epsilon) continue;
        double x = num / den;
        solutions.push_back({x, y});
    }
    return solutions;
}

// ---------------------------------------------------------------------------
// SP6: two coupled equations in two angles
// ---------------------------------------------------------------------------

static double sp6Residual(
    double t1, double t2,
    const std::array<Eigen::Vector3d, 4>& h,
    const std::array<Eigen::Vector3d, 4>& k,
    const std::array<Eigen::Vector3d, 4>& pv,
    double d1, double d2)
{
    double lhs1 = h[0].dot(rotate(k[0], t1, pv[0])) + h[1].dot(rotate(k[1], t2, pv[1]));
    double lhs2 = h[2].dot(rotate(k[2], t1, pv[2])) + h[3].dot(rotate(k[3], t2, pv[3]));
    return std::max(std::abs(lhs1 - d1), std::abs(lhs2 - d2));
}

static std::pair<double, double> refineGN(
    double t1, double t2,
    const std::array<Eigen::Vector3d, 4>& h,
    const std::array<Eigen::Vector3d, 4>& k,
    const std::array<Eigen::Vector3d, 4>& pv,
    double d1, double d2,
    int max_iter = 20)
{
    const double max_step = M_PI / 4.0;
    for (int it = 0; it < max_iter; ++it) {
        Eigen::Vector3d r0 = rotate(k[0], t1, pv[0]);
        Eigen::Vector3d r1 = rotate(k[1], t2, pv[1]);
        Eigen::Vector3d r2 = rotate(k[2], t1, pv[2]);
        Eigen::Vector3d r3 = rotate(k[3], t2, pv[3]);

        double f0 = h[0].dot(r0) + h[1].dot(r1) - d1;
        double f1 = h[2].dot(r2) + h[3].dot(r3) - d2;

        double j00 = h[0].dot(k[0].cross(r0));
        double j01 = h[1].dot(k[1].cross(r1));
        double j10 = h[2].dot(k[2].cross(r2));
        double j11 = h[3].dot(k[3].cross(r3));

        double det = j00 * j11 - j01 * j10;
        if (std::abs(det) < 1e-15) break;

        double inv = 1.0 / det;
        double delta0 = -(j11 * f0 - j01 * f1) * inv;
        double delta1 = -(-j10 * f0 + j00 * f1) * inv;

        double step = std::sqrt(delta0*delta0 + delta1*delta1);
        if (step > max_step) {
            double sc = max_step / step;
            delta0 *= sc;
            delta1 *= sc;
        }
        t1 = wrapToPi(t1 + delta0);
        t2 = wrapToPi(t2 + delta1);
        if (step < 1e-15) break;
    }
    return {t1, t2};
}

Sp6Result sp6(const std::array<Eigen::Vector3d, 4>& h,
              const std::array<Eigen::Vector3d, 4>& k,
              const std::array<Eigen::Vector3d, 4>& pv,
              double d1, double d2,
              double deg_tol,
              double num_tol)
{
    // Check if all p_i are collinear with k_i (fully degenerate)
    bool all_collinear = true;
    for (int i = 0; i < 4; ++i) {
        double p_perp_sq = pv[i].squaredNorm() - std::pow(k[i].dot(pv[i]), 2);
        if (p_perp_sq >= deg_tol) { all_collinear = false; break; }
    }
    if (all_collinear) return {{}, true};

    // Build A (2x4) and b (2x1)
    std::array<Eigen::Matrix<double, 3, 2>, 4> a_cols;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d kxp = k[i].cross(pv[i]);
        a_cols[i].col(0) = kxp;
        a_cols[i].col(1) = -k[i].cross(kxp);
    }

    Eigen::Matrix<double, 2, 4> A;
    A.row(0).segment<2>(0) = (h[0].transpose() * a_cols[0]).transpose();
    A.row(0).segment<2>(2) = (h[1].transpose() * a_cols[1]).transpose();
    A.row(1).segment<2>(0) = (h[2].transpose() * a_cols[2]).transpose();
    A.row(1).segment<2>(2) = (h[3].transpose() * a_cols[3]).transpose();

    Eigen::Vector2d bvec;
    bvec(0) = d1 - h[0].dot(k[0]) * k[0].dot(pv[0]) - h[1].dot(k[1]) * k[1].dot(pv[1]);
    bvec(1) = d2 - h[2].dot(k[2]) * k[2].dot(pv[2]) - h[3].dot(k[3]) * k[3].dot(pv[3]);

    // Full QR of A^T (4x2)
    Eigen::HouseholderQR<Eigen::Matrix<double, 4, 2>> qr(A.transpose());
    Eigen::Matrix4d Q_full = Eigen::Matrix4d::Identity();
    Q_full = qr.householderQ() * Q_full;

    // Range: first 2 cols, Null: last 2 cols
    Eigen::Vector4d x_null_1 = Q_full.col(2);
    Eigen::Vector4d x_null_2 = Q_full.col(3);
    Eigen::Matrix<double, 4, 2> Q_range = Q_full.leftCols<2>();

    // Upper triangular R (2x2) from the QR
    Eigen::Matrix2d R_upper = qr.matrixQR().topLeftCorner<2, 2>().triangularView<Eigen::Upper>();

    if (std::abs(R_upper(0, 0)) < deg_tol || std::abs(R_upper(1, 1)) < deg_tol) {
        return {{}, true};
    }

    // Solve R^T x = b (lower triangular)
    Eigen::Matrix2d R_lower = R_upper.transpose();
    Eigen::Vector2d x_min_coefs;
    x_min_coefs(0) = bvec(0) / R_lower(0, 0);
    x_min_coefs(1) = (bvec(1) - R_lower(1, 0) * x_min_coefs(0)) / R_lower(1, 1);
    Eigen::Vector4d x_min = Q_range * x_min_coefs;

    // Form conic constraints: |x_min[:2] + xn1 @ [xi0, xi1]|^2 = 1
    //                          |x_min[2:] + xn2 @ [xi0, xi1]|^2 = 1
    Eigen::Vector2d     xm1 = x_min.head<2>();
    Eigen::Matrix2d     xn1;
    xn1.col(0) = x_null_1.head<2>();
    xn1.col(1) = x_null_2.head<2>();

    Eigen::Vector2d     xm2 = x_min.tail<2>();
    Eigen::Matrix2d     xn2;
    xn2.col(0) = x_null_1.tail<2>();
    xn2.col(1) = x_null_2.tail<2>();

    auto xi_sols = solveTwoEllipse(xm1, xn1, xm2, xn2, deg_tol);
    if (xi_sols.empty()) return {{}, true};

    // Reconstruct angle pairs, sort by residual, refine via GN
    std::vector<std::pair<double, double>> candidates;
    for (auto& [xi0, xi1] : xi_sols) {
        Eigen::Vector4d x = x_min + xi0 * x_null_1 + xi1 * x_null_2;
        double n1 = x.head<2>().norm();
        double n2 = x.tail<2>().norm();
        if (std::abs(n1 - 1.0) > num_tol || std::abs(n2 - 1.0) > num_tol) continue;
        double t1 = std::atan2(x(0), x(1));
        double t2 = std::atan2(x(2), x(3));
        candidates.push_back({t1, t2});
    }

    // Sort by pre-GN residual (cleanest Bezout representative first)
    std::sort(candidates.begin(), candidates.end(),
              [&](const std::pair<double,double>& ca, const std::pair<double,double>& cb) {
                  return sp6Residual(ca.first, ca.second, h, k, pv, d1, d2)
                       < sp6Residual(cb.first, cb.second, h, k, pv, d1, d2);
              });

    // Refine via GN
    std::vector<std::pair<double, double>> refined;
    for (auto& cand : candidates) {
        auto [rt1, rt2] = refineGN(cand.first, cand.second, h, k, pv, d1, d2);
        refined.push_back({rt1, rt2});
    }

    // Separate exact from LS
    std::vector<std::pair<double, double>> exact;
    for (auto& r : refined) {
        if (sp6Residual(r.first, r.second, h, k, pv, d1, d2) < num_tol) {
            exact.push_back(r);
        }
    }
    if (!exact.empty()) return {exact, false};
    if (!refined.empty()) {
        auto best = *std::min_element(refined.begin(), refined.end(),
            [&](const std::pair<double,double>& ca, const std::pair<double,double>& cb) {
                return sp6Residual(ca.first, ca.second, h, k, pv, d1, d2)
                     < sp6Residual(cb.first, cb.second, h, k, pv, d1, d2);
            });
        return {{best}, true};
    }
    return {{}, true};
}

}  // namespace poe_sp
}  // namespace kinematics_lib
