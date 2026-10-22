#include "robnux_kinematics_map/serialArmPOE.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>

#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "simple_motion_logger/Logger.h"

// register plugins
PLUGINLIB_EXPORT_CLASS(kinematics_lib::serialArmPOE_6R, kinematics_lib::BaseKinematicMap)
PLUGINLIB_EXPORT_CLASS(kinematics_lib::serialArmPOE_7R, kinematics_lib::BaseKinematicMap)

namespace kinematics_lib {

// ===========================================================================
// Constructor
// ===========================================================================

serialArmPOE::serialArmPOE(size_t dof) : serialArm(dof) {}

// ===========================================================================
// SetGeometry
// kine_para = [axes 3N | T_left 16N row-major | T_right 16N row-major]
// ===========================================================================

void serialArmPOE::SetGeometry(const Eigen::VectorXd& kine_para) {
    std::ostringstream s;
    if (initialized_) {
        s << GetName() << ": already initialized\n"; LOG_ERROR(s); return;
    }
    const size_t N = DoF_;
    if (static_cast<size_t>(kine_para.size()) < 35 * N) {
        s << GetName() << ": kine_para too short (" << kine_para.size()
          << " < " << 35*N << ")\n"; LOG_ERROR(s); return;
    }
    axes_.resize(N); T_left_.resize(N); T_right_.resize(N);
    for (size_t i = 0; i < N; ++i) {
        axes_[i] = kine_para.segment<3>(3*i); axes_[i].normalize();
    }
    const size_t o1 = 3*N, o2 = o1 + 16*N;
    for (size_t i = 0; i < N; ++i) {
        T_left_[i]  = Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(kine_para.data()+o1+16*i);
        T_right_[i] = Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(kine_para.data()+o2+16*i);
    }
    initialized_ = true;
    topology_    = detectTopology();
}

std::string serialArmPOE::GetName() const { return "serialArmPOE"; }

// ===========================================================================
// Frame / Eigen utilities
// ===========================================================================

static Frame eigenToFrame(const Eigen::Matrix4d& T) {
    Rotation R(T(0,0),T(0,1),T(0,2), T(1,0),T(1,1),T(1,2), T(2,0),T(2,1),T(2,2));
    return Frame(R, Vec(T(0,3), T(1,3), T(2,3)));
}
static Eigen::Matrix4d frameToEigen(const Frame& f) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Rotation R = f.getRotation(); Vec P = f.getTranslation();
    for (int r=0;r<3;r++) for (int c=0;c<3;c++) T(r,c)=R(r,c);
    T(0,3)=P.x(); T(1,3)=P.y(); T(2,3)=P.z(); return T;
}
static Eigen::VectorXd buildKineParams(const std::vector<Eigen::Vector3d>& ax,
                                       const std::vector<Eigen::Matrix4d>& TL,
                                       const std::vector<Eigen::Matrix4d>& TR) {
    size_t N = ax.size();
    Eigen::VectorXd p(35*N);
    for (size_t i=0;i<N;i++) p.segment<3>(3*i) = ax[i];
    for (size_t i=0;i<N;i++) Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>>(p.data()+3*N+16*i) = TL[i];
    for (size_t i=0;i<N;i++) Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>>(p.data()+3*N+16*N+16*i) = TR[i];
    return p;
}

// ===========================================================================
// POE FK
// ===========================================================================

Eigen::Matrix4d serialArmPOE::poeFk(const Eigen::VectorXd& q) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    {   // Apply base offset
        Rotation R=defaultBaseOff_.getRotation(); Vec P=defaultBaseOff_.getTranslation();
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) T(r,c)=R(r,c);
        T(0,3)=P.x(); T(1,3)=P.y(); T(2,3)=P.z();
    }
    for (size_t i=0; i<DoF_; ++i) {
        double qi=q(i), c=std::cos(qi), s=std::sin(qi), oc=1-c;
        double ax=axes_[i].x(), ay=axes_[i].y(), az=axes_[i].z();
        Eigen::Matrix4d Rot=Eigen::Matrix4d::Identity();
        Rot(0,0)=c+ax*ax*oc; Rot(0,1)=ax*ay*oc-az*s; Rot(0,2)=ax*az*oc+ay*s;
        Rot(1,0)=ay*ax*oc+az*s; Rot(1,1)=c+ay*ay*oc; Rot(1,2)=ay*az*oc-ax*s;
        Rot(2,0)=az*ax*oc-ay*s; Rot(2,1)=az*ay*oc+ax*s; Rot(2,2)=c+az*az*oc;
        T = T * T_left_[i] * Rot * T_right_[i];
    }
    return T;
}

// ===========================================================================
// Frame at joint_idx BEFORE that joint's rotation
// ===========================================================================

std::pair<Eigen::Matrix3d, Eigen::Vector3d>
serialArmPOE::frameAtJoint(const Eigen::VectorXd& q, int joint_idx) const {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (int i=0; i<(int)DoF_; ++i) {
        p += R * T_left_[i].block<3,1>(0,3);
        if (i == joint_idx) return {R, p};
        double qi=q(i), c=std::cos(qi), s=std::sin(qi), oc=1-c;
        double ax=axes_[i].x(), ay=axes_[i].y(), az=axes_[i].z();
        Eigen::Matrix3d Ri;
        Ri(0,0)=c+ax*ax*oc; Ri(0,1)=ax*ay*oc-az*s; Ri(0,2)=ax*az*oc+ay*s;
        Ri(1,0)=ay*ax*oc+az*s; Ri(1,1)=c+ay*ay*oc; Ri(1,2)=ay*az*oc-ax*s;
        Ri(2,0)=az*ax*oc-ay*s; Ri(2,1)=az*ay*oc+ax*s; Ri(2,2)=c+az*az*oc;
        R = R * Ri;
        Eigen::Matrix3d Rt = T_right_[i].block<3,3>(0,0);
        if (!Rt.isIdentity(1e-10)) R = R * Rt;
        p += R * T_right_[i].block<3,1>(0,3);
    }
    return {R, p};
}

// ===========================================================================
// Geometric Jacobian
// ===========================================================================

int serialArmPOE::calcGeomJacobian(const Eigen::VectorXd& q,
                                    Eigen::MatrixXd& Jt, Eigen::MatrixXd& Jr,
                                    Eigen::Vector3d& p_ee) const {
    const int N = DoF_;
    Jt.resize(3,N); Jr.resize(3,N);
    p_ee = poeFk(q).block<3,1>(0,3);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i=0; i<N; ++i) {
        Eigen::Matrix4d Tb = T * T_left_[i];
        Eigen::Vector3d z = Tb.block<3,3>(0,0) * axes_[i];
        Eigen::Vector3d pi = Tb.block<3,1>(0,3);
        Jr.col(i)=z; Jt.col(i)=z.cross(p_ee-pi);
        double qi=q(i),c=std::cos(qi),s=std::sin(qi),oc=1-c;
        double ax=axes_[i].x(),ay=axes_[i].y(),az=axes_[i].z();
        Eigen::Matrix4d Rot=Eigen::Matrix4d::Identity();
        Rot(0,0)=c+ax*ax*oc; Rot(0,1)=ax*ay*oc-az*s; Rot(0,2)=ax*az*oc+ay*s;
        Rot(1,0)=ay*ax*oc+az*s; Rot(1,1)=c+ay*ay*oc; Rot(1,2)=ay*az*oc-ax*s;
        Rot(2,0)=az*ax*oc-ay*s; Rot(2,1)=az*ay*oc+ax*s; Rot(2,2)=c+az*az*oc;
        T = Tb * Rot * T_right_[i];
    }
    return 0;
}

// ===========================================================================
// FK verifier
// ===========================================================================

bool serialArmPOE::verifyCandidate(const Eigen::VectorXd& q,
                                    const Eigen::Matrix4d& T, double pt, double rt) const {
    Eigen::Matrix4d Tf = poeFk(q);
    if ((Tf.block<3,1>(0,3)-T.block<3,1>(0,3)).norm() > pt) return false;
    Eigen::AngleAxisd aa(Tf.block<3,3>(0,0).transpose()*T.block<3,3>(0,0));
    return std::abs(aa.angle()) < rt;
}

// ===========================================================================
// Joint origins
// ===========================================================================

std::vector<Eigen::Vector3d> serialArmPOE::jointOrigins() const {
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> o; o.reserve(DoF_);
    for (const auto& Tl : T_left_) { c += Tl.block<3,1>(0,3); o.push_back(c); }
    return o;
}

// ===========================================================================
// Geometric predicates
// ===========================================================================

bool serialArmPOE::axisParallel(const Eigen::Vector3d& a,
                                 const Eigen::Vector3d& b, double tol) {
    return a.cross(b).norm() < tol;
}

bool serialArmPOE::axisIntersect(const Eigen::Vector3d& a, const Eigen::Vector3d& oa,
                                  const Eigen::Vector3d& b, const Eigen::Vector3d& ob,
                                  double par_tol, double dist_tol) {
    Eigen::Vector3d cr = a.cross(b);
    double cn = cr.norm();
    Eigen::Vector3d d = ob - oa;
    if (cn < par_tol) return (d - d.dot(a)*a).norm() < dist_tol;
    return std::abs(cr.dot(d)) / cn < dist_tol;
}

int serialArmPOE::findThreeConsecutiveParallel(double tol) const {
    for (size_t i=0; i+2<DoF_; ++i)
        if (axisParallel(axes_[i],axes_[i+1],tol) &&
            axisParallel(axes_[i+1],axes_[i+2],tol) &&
            axisParallel(axes_[i],axes_[i+2],tol)) return (int)i;
    return -1;
}

int serialArmPOE::findThreeConsecutiveIntersecting(
    const std::vector<Eigen::Vector3d>& origs, double pt, double dt) const {
    for (size_t i=0; i+2<DoF_; ++i) {
        const auto &a=axes_[i],&b=axes_[i+1],&c=axes_[i+2];
        const auto &oa=origs[i],&ob=origs[i+1],&oc=origs[i+2];
        if (axisParallel(a,b,pt)||axisParallel(b,c,pt)) continue;
        if (!axisIntersect(a,oa,b,ob,pt,dt)) continue;
        if (!axisIntersect(b,ob,c,oc,pt,dt)) continue;
        if (!axisIntersect(a,oa,c,oc,pt,dt)) continue;
        Eigen::Matrix<double,3,2> M; M.col(0)=a; M.col(1)=-b;
        Eigen::Vector2d sol=M.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).solve(ob-oa);
        Eigen::Vector3d p=oa+sol(0)*a;
        Eigen::Vector3d dc=p-oc;
        if ((dc-dc.dot(c)*c).norm()>=dt) continue;
        if ((ob-p).norm()>=dt) continue;
        if ((oc-p).norm()>=dt) continue;
        return (int)i;
    }
    return -1;
}

// Common intersection point of joint axes at the given indices
// (relaxed check — axis lines only, not origin coincidence).
std::optional<Eigen::Vector3d>
serialArmPOE::findCommonPoint(const std::vector<int>& idx,
                               const std::vector<Eigen::Vector3d>& origs,
                               double par_tol, double dist_tol) const {
    if (idx.size() < 2) return std::nullopt;
    for (size_t k=0; k+1<idx.size(); ++k)
        if (axisParallel(axes_[idx[k]], axes_[idx[k+1]], par_tol)) return std::nullopt;
    // Intersection of first two axes
    Eigen::Matrix<double,3,2> M;
    M.col(0)=axes_[idx[0]]; M.col(1)=-axes_[idx[1]];
    Eigen::Vector2d sol=M.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV)
                         .solve(origs[idx[1]]-origs[idx[0]]);
    Eigen::Vector3d common = origs[idx[0]] + sol(0)*axes_[idx[0]];
    // Verify all axes pass through common
    for (int i : idx) {
        Eigen::Vector3d d = common - origs[i];
        if ((d - d.dot(axes_[i])*axes_[i]).norm() >= dist_tol) return std::nullopt;
    }
    return common;
}

// ===========================================================================
// SRS 7R detection
// ===========================================================================

std::optional<SrsData> serialArmPOE::findSRS7R() const {
    if (DoF_ != 7) return std::nullopt;
    auto origs = jointOrigins();
    auto S = findCommonPoint({0,1,2}, origs);
    if (!S) return std::nullopt;
    auto W = findCommonPoint({4,5,6}, origs);
    if (!W) return std::nullopt;

    // Arm lengths
    double L_se = (origs[3] - *S).norm();
    double L_ew = (origs[3] - *W).norm();
    if (L_se < 1e-6 || L_ew < 1e-6) return std::nullopt;

    // EE at q=0 minus wrist pivot
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
    Eigen::Vector3d ee_home = poeFk(q0).block<3,1>(0,3);
    Eigen::Matrix3d R_post  = T_right_.back().block<3,3>(0,0);

    return SrsData{*S, *W, L_se, L_ew, ee_home - *W, R_post};
}

// ===========================================================================
// Topology dispatcher (called once in SetGeometry)
// ===========================================================================

ArmTopology serialArmPOE::detectTopology() const {
    constexpr double PT=1e-6, DT=1e-6;

    if (DoF_ == 6) {
        auto origs = jointOrigins();
        int par = findThreeConsecutiveParallel(PT);
        if (par == 1) return ArmTopology::ThreeParallel;
        int sph = findThreeConsecutiveIntersecting(origs, PT, DT);
        if (sph == 3) {
            if (axisParallel(axes_[1], axes_[2], PT))
                return ArmTopology::SphericalTwoParallel;
            if (T_left_[1].block<3,1>(0,3).norm() < DT)
                return ArmTopology::SphericalTwoIntersecting;
        }
        return ArmTopology::NoMatch;
    }

    if (DoF_ == 7) {
        auto srs = findSRS7R();
        if (srs) {
            srs_data_ = *srs;
            return ArmTopology::SevenRSRS;
        }
        lock_idx_ = chooseLockJoint();
        return ArmTopology::SevenRJointLock;
    }

    return ArmTopology::Unknown;
}

// ===========================================================================
// Shared spherical-wrist step (6R)
// ===========================================================================

void serialArmPOE::solveSphericalWrist(const Eigen::Matrix3d& r36,
                                        std::vector<Eigen::Vector3d>& q456) const {
    using namespace poe_sp;
    auto res = sp4(axes_[3], axes_[4], axes_[5], axes_[3].dot(r36*axes_[5]));
    for (double q5 : res.angles) {
        double q4 = sp1(axes_[3], rotMat( axes_[4], q5)*axes_[5], r36*axes_[5]).theta;
        double q6 = sp1(-axes_[5], rotMat(-axes_[4], q5)*axes_[3], r36.transpose()*axes_[3]).theta;
        q456.push_back({q4,q5,q6});
    }
}

// ===========================================================================
// Candidate selection helper
// ===========================================================================

namespace {
// Max FK residual (position [m] + rotation angle [rad]) for an IK candidate
// to count as an exact solution.
static constexpr double IK_RESIDUAL_TOL = 1e-4;
// Joint indices used by the 6R branch flags.
static constexpr int SIXR_SHOULDER_JOINT = 1;   // q2 (first arm-plane joint)
static constexpr int SIXR_ELBOW_JOINT    = 2;   // q3
static constexpr int SIXR_WRIST1_JOINT   = 3;   // q4
static constexpr int SIXR_WRIST2_JOINT   = 4;   // q5
static constexpr int SIXR_WRIST3_JOINT   = 5;   // q6
// SRS 7R joints whose signs select its discrete branches (see cartToJntSRS).
static constexpr int SRS_SHOULDER_JOINT = 1;    // q1 = q1_sign * acos(..)
static constexpr int SRS_WRIST_JOINT    = 5;    // q5 = q5_sign * acos(..)
// SRS branch flags honored by cartToJntSRS: {q1 sign, q5 sign}.
static constexpr size_t SRS_NUM_BRANCH_FLAGS = 2;
// 6R branch flags: {shoulder, elbow, wrist}.
static constexpr size_t SIXR_NUM_BRANCH_FLAGS = 3;
// Numeric (damped least squares) IK fallback.
static constexpr int    IK_NUM_MAX_ITER  = 200;    // iterations per seed
static constexpr double IK_NUM_DAMPING   = 1e-3;   // LM damping lambda
static constexpr double IK_NUM_CONV_TOL  = 1e-12;  // |pose error| to stop
static constexpr double IK_NUM_MAX_STEP  = 0.5;    // max joint step [rad]
static constexpr int    IK_NUM_RESTARTS  = 64;     // extra seeds when cold
static constexpr unsigned IK_NUM_SEED    = 12345u; // fixed RNG seed -> deterministic
// Branch flag for a signed quantity: 0 for >= 0, 1 for < 0.
static int signFlag(double v) { return v >= 0.0 ? 0 : 1; }
struct Cand { Eigen::VectorXd q; double res; int oi, ii; };
static double fkRes(const Eigen::VectorXd& q, const Eigen::Matrix4d& T,
             const std::function<Eigen::Matrix4d(const Eigen::VectorXd&)>& fk) {
    Eigen::Matrix4d Tf=fk(q);
    double pe=(Tf.block<3,1>(0,3)-T.block<3,1>(0,3)).norm();
    Eigen::AngleAxisd aa(Tf.block<3,3>(0,0).transpose()*T.block<3,3>(0,0));
    return pe+std::abs(aa.angle());
}
// Sort candidates by residual; return the exact solution (residual within
// IK_RESIDUAL_TOL) whose geometric branch matches br, else the best exact one.
// Returns false when no candidate is exact.
static bool pickByBranch(std::vector<Cand>& cv, const std::vector<int>& br,
                         const std::function<std::vector<int>(const Eigen::VectorXd&)>& branchOf,
                         Eigen::VectorXd& qo) {
    if (cv.empty()) return false;
    std::sort(cv.begin(),cv.end(),[](const Cand&a,const Cand&b){return a.res<b.res;});
    if (cv.front().res>IK_RESIDUAL_TOL) return false;
    if (br.size()>=SIXR_NUM_BRANCH_FLAGS) {
        for (auto& c:cv) {
            if (c.res>IK_RESIDUAL_TOL) break;
            auto cb=branchOf(c.q);
            if (std::equal(cb.begin(),cb.end(),br.begin())) { qo=c.q; return true; }
        }
    }
    qo=cv.front().q; return true;
}
// Wrap angle to (-pi, pi]
static double wrapPi(double a) { return poe_sp::wrapToPi(a); }
// Joint-space distance that treats each joint as periodic (mod 2*pi) --
// atan2-derived angles wrap at +-pi, so a joint that physically moved by a
// tiny amount across that boundary (e.g. +3.14 -> -3.13) has a *raw*
// difference of nearly 2*pi. Using the raw norm as a continuity metric
// makes that joint look like it jumped almost all the way around, which
// corrupts both the discrete tie-break and the continuous refinement's
// search direction right at the wrap.
static double wrappedDist(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    double s = 0.0;
    for (int i=0; i<a.size(); ++i) { double d = wrapPi(a(i)-b(i)); s += d*d; }
    return std::sqrt(s);
}
// Check if two 7-angle solutions are within tol of each other
static bool nearDup(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double tol=1e-3) {
    for (int i=0; i<a.size(); ++i)
        if (std::abs(wrapPi(a(i)-b(i)))>tol) return false;
    return true;
}
static void dedup(std::vector<Eigen::VectorXd>& v, double tol=1e-3) {
    std::vector<Eigen::VectorXd> u;
    for (auto& q : v) {
        bool dup=false;
        for (auto& r : u) if (nearDup(q,r,tol)){dup=true;break;}
        if (!dup) u.push_back(q);
    }
    v=std::move(u);
}
}  // namespace

// ===========================================================================
// 6R IK: three-parallel  (UR family)
// ===========================================================================

int serialArmPOE::cartToJntThreeParallel(const Eigen::Matrix4d& T,
                                          const std::vector<int>& br,
                                          Eigen::VectorXd& qo) const {
    using namespace poe_sp;
    std::array<Eigen::Vector3d,7> p;
    for(int i=0;i<6;i++) p[i]=T_left_[i].block<3,1>(0,3);
    p[6]=T_right_[5].block<3,1>(0,3);
    Eigen::Matrix3d rh=T_right_[5].block<3,3>(0,0);
    Eigen::Matrix3d r06=T.block<3,3>(0,0)*rh.transpose();
    Eigen::Vector3d p0t=T.block<3,1>(0,3);
    Eigen::Vector3d p16=p0t-p[0]-r06*p[6];
    std::array<Eigen::Vector3d,4> hs={axes_[1],axes_[1],axes_[1],axes_[1]};
    std::array<Eigen::Vector3d,4> ks={-axes_[0],axes_[4],-axes_[0],axes_[4]};
    std::array<Eigen::Vector3d,4> ps={p16,-p[5],r06*axes_[5],-axes_[5]};
    auto s6=sp6(hs,ks,ps,axes_[1].dot(p[1]+p[2]+p[3]+p[4]),0.0);
    auto fk=[this](const Eigen::VectorXd& q){return poeFk(q);};
    std::vector<Cand> cv;
    int oi=0;
    for(auto&[q1,q5]:s6.pairs){
        Eigen::Matrix3d r01=rotMat(axes_[0],q1), r45=rotMat(axes_[4],q5);
        double t14=sp1(axes_[1],r45*axes_[5],r01.transpose()*r06*axes_[5]).theta;
        double q6=sp1(-axes_[5],r45.transpose()*axes_[1],r06.transpose()*r01*axes_[1]).theta;
        Eigen::Matrix3d r14=rotMat(axes_[1],t14);
        Eigen::Vector3d di=r01.transpose()*p16-p[1]-r14*r45*p[5]-r14*p[4];
        auto s3=sp3(axes_[1],-p[3],p[2],di.norm());
        int ii=0;
        for(double q3:s3.angles){
            Eigen::Vector3d p2r=p[2]+rotate(axes_[1],q3,p[3]);
            double q2=sp1(axes_[1],p2r,di).theta;
            double q4=wrapToPi(t14-q2-q3);
            Eigen::VectorXd qc(6); qc<<wrapToPi(q1),wrapToPi(q2),wrapToPi(q3),q4,wrapToPi(q5),wrapToPi(q6);
            cv.push_back({qc,fkRes(qc,T,fk),oi,ii++});
        }
        ++oi;
    }
    // Select by geometric branch, not by (oi, ii): sp6/sp3 return their
    // solutions in an order that changes as the target moves, so a fixed
    // index pair maps to different physical configurations along a path
    // (MoveLine flicker) and does not even round-trip FK -> IK.
    auto branchOf=[this](const Eigen::VectorXd& q){return sixRBranch(q);};
    if(!pickByBranch(cv,br,branchOf,qo)) return -ERR_ROB_IK_OUTOF_REACH;
    return 0;
}

std::vector<int> serialArmPOE::sixRBranch(const Eigen::VectorXd& q) const {
    // shoulder: wrist point in front of / behind joint 1's axis, measured along
    //           the arm plane's reach direction (plane normal = J2 axis after
    //           q1); 0 when the wrist point is on that axis (shoulder singularity)
    // elbow:    a3 . (upper x R(a3,q3) forearm), 0 when upper arm and forearm
    //           are aligned (stretched/folded elbow singularity)
    // wrist:    a5 . (a4 x R(a5,q5) a6), 0 when wrist axes 4 and 6 are
    //           parallel (wrist singularity)
    // For the UR (ThreeParallel) geometry these reduce to sin(q3) and sin(q5).
    const int last=static_cast<int>(DoF_)-1;
    Eigen::Matrix4d T=poeFk(q);
    Eigen::Matrix3d r06=T.block<3,3>(0,0)*T_right_[last].block<3,3>(0,0).transpose();
    Eigen::Vector3d p16=T.block<3,1>(0,3)-T_left_[0].block<3,1>(0,3)-r06*T_right_[last].block<3,1>(0,3);
    Eigen::Vector3d reach=(poe_sp::rotMat(axes_[0],q(0))*axes_[SIXR_SHOULDER_JOINT]).cross(axes_[0]);

    // forearm: elbow -> next joint (ThreeParallel) or -> wrist center (spherical wrist)
    Eigen::Vector3d upper=T_left_[SIXR_ELBOW_JOINT].block<3,1>(0,3);
    Eigen::Vector3d fore=T_left_[SIXR_WRIST1_JOINT].block<3,1>(0,3);
    if (topology_!=ArmTopology::ThreeParallel)
        fore+=T_left_[SIXR_WRIST2_JOINT].block<3,1>(0,3)+T_left_[SIXR_WRIST3_JOINT].block<3,1>(0,3);
    const Eigen::Vector3d& a3=axes_[SIXR_ELBOW_JOINT];
    double elbow=a3.dot(upper.cross(poe_sp::rotMat(a3,q(SIXR_ELBOW_JOINT))*fore));

    const Eigen::Vector3d& a5=axes_[SIXR_WRIST2_JOINT];
    double wrist=a5.dot(axes_[SIXR_WRIST1_JOINT].cross(
        poe_sp::rotMat(a5,q(SIXR_WRIST2_JOINT))*axes_[SIXR_WRIST3_JOINT]));
    return {signFlag(reach.dot(p16)), signFlag(elbow), signFlag(wrist)};
}

// ===========================================================================
// 6R IK: spherical-wrist + two-parallel shoulder  (Puma/Fanuc/KUKA KR)
// ===========================================================================

int serialArmPOE::cartToJntSphericalTwoParallel(const Eigen::Matrix4d& T,
                                                  const std::vector<int>& br,
                                                  Eigen::VectorXd& qo) const {
    using namespace poe_sp;
    std::array<Eigen::Vector3d,7> p;
    for(int i=0;i<6;i++) p[i]=T_left_[i].block<3,1>(0,3);
    p[3]=T_left_[3].block<3,1>(0,3)+T_left_[4].block<3,1>(0,3)+T_left_[5].block<3,1>(0,3);
    p[4]=p[5]=Eigen::Vector3d::Zero();
    p[6]=T_right_[5].block<3,1>(0,3);
    Eigen::Matrix3d rh=T_right_[5].block<3,3>(0,0);
    Eigen::Matrix3d r06=T.block<3,3>(0,0)*rh.transpose();
    Eigen::Vector3d p0t=T.block<3,1>(0,3);
    auto fk=[this](const Eigen::VectorXd& q){return poeFk(q);};
    auto s4=sp4(axes_[1],-axes_[0],p0t-r06*p[6]-p[0],axes_[1].dot(p[1]+p[2]+p[3]));
    std::vector<Cand> cv;
    int oi=0;
    for(double q1:s4.angles){
        Eigen::Vector3d sh=rotMat(-axes_[0],q1)*(-p0t+r06*p[6]+p[0])+p[1];
        auto s3=sp3(axes_[2],-p[3],p[2],sh.norm());
        int ii=0;
        for(double q3:s3.angles){
            double q2=sp1(axes_[1],-p[2]-rotMat(axes_[2],q3)*p[3],sh).theta;
            Eigen::Matrix3d r36=rotMat(-axes_[2],q3)*rotMat(-axes_[1],q2)*rotMat(-axes_[0],q1)*r06;
            std::vector<Eigen::Vector3d> w; solveSphericalWrist(r36,w);
            for(auto&wv:w){
                Eigen::VectorXd qc(6); qc<<q1,q2,q3,wv(0),wv(1),wv(2);
                qc=qc.unaryExpr([](double a){return wrapToPi(a);});
                cv.push_back({qc,fkRes(qc,T,fk),oi,ii});
            }
            ++ii;
        }
        ++oi;
    }
    auto branchOf=[this](const Eigen::VectorXd& q){return sixRBranch(q);};
    if(!pickByBranch(cv,br,branchOf,qo)) return -ERR_ROB_IK_OUTOF_REACH;
    return 0;
}

// ===========================================================================
// 6R IK: spherical-wrist + intersecting shoulder  (ABB IRB120/xArm6/Lite6)
// ===========================================================================

int serialArmPOE::cartToJntSphericalTwoIntersecting(const Eigen::Matrix4d& T,
                                                      const std::vector<int>& br,
                                                      Eigen::VectorXd& qo) const {
    using namespace poe_sp;
    std::array<Eigen::Vector3d,7> p;
    for(int i=0;i<6;i++) p[i]=T_left_[i].block<3,1>(0,3);
    p[3]=T_left_[3].block<3,1>(0,3)+T_left_[4].block<3,1>(0,3)+T_left_[5].block<3,1>(0,3);
    p[4]=p[5]=Eigen::Vector3d::Zero();
    p[6]=T_right_[5].block<3,1>(0,3);
    Eigen::Matrix3d rh=T_right_[5].block<3,3>(0,0);
    Eigen::Matrix3d r06=T.block<3,3>(0,0)*rh.transpose();
    Eigen::Vector3d p0t=T.block<3,1>(0,3);
    Eigen::Vector3d p16=p0t-r06*p[6]-p[0];
    auto fk=[this](const Eigen::VectorXd& q){return poeFk(q);};
    auto s3=sp3(axes_[2],p[3],-p[2],p16.norm());
    std::vector<Cand> cv;
    int oi=0;
    for(double q3:s3.angles){
        auto s2=sp2(-axes_[0],axes_[1],p16,p[2]+rotMat(axes_[2],q3)*p[3]);
        int ii=0;
        for(auto&[q1,q2]:s2.pairs){
            Eigen::Matrix3d r36=rotMat(-axes_[2],q3)*rotMat(-axes_[1],q2)*rotMat(-axes_[0],q1)*r06;
            std::vector<Eigen::Vector3d> w; solveSphericalWrist(r36,w);
            for(auto&wv:w){
                Eigen::VectorXd qc(6); qc<<q1,q2,q3,wv(0),wv(1),wv(2);
                qc=qc.unaryExpr([](double a){return wrapToPi(a);});
                cv.push_back({qc,fkRes(qc,T,fk),oi,ii});
            }
            ++ii;
        }
        ++oi;
    }
    auto branchOf=[this](const Eigen::VectorXd& q){return sixRBranch(q);};
    if(!pickByBranch(cv,br,branchOf,qo)) return -ERR_ROB_IK_OUTOF_REACH;
    return 0;
}

// ===========================================================================
// 7R IK: SRS Singh-Kreutz  (iiwa / Rizon / Kassow)
// ===========================================================================

int serialArmPOE::cartToJntSRS(const Eigen::Matrix4d& T,
                                 const std::vector<int>& branch,
                                 Eigen::VectorXd& qo) const {
    const SrsData& d = srs_data_;
    Eigen::Matrix3d R_target = T.block<3,3>(0,0);
    Eigen::Vector3d p_target = T.block<3,1>(0,3);

    Eigen::Vector3d W_t = p_target - R_target * d.ee_offset_local;
    Eigen::Vector3d SW  = W_t - d.shoulder_pivot;
    double dsw = SW.norm();

    if (dsw < 1e-12 || dsw > d.L_se + d.L_ew || dsw < std::abs(d.L_se - d.L_ew))
        return -ERR_ROB_IK_OUTOF_REACH;

    Eigen::Vector3d u_sw = SW / dsw;

    // Swivel-circle geometry
    double x_c     = (d.L_se*d.L_se - d.L_ew*d.L_ew + dsw*dsw) / (2.0*dsw);
    double r_sq    = std::max(d.L_se*d.L_se - x_c*x_c, 0.0);
    double r_circle = std::sqrt(r_sq);

    // Basis for swivel plane
    Eigen::Vector3d ref = (std::abs(u_sw.z()) < 0.99)
        ? Eigen::Vector3d(0,0,1) : Eigen::Vector3d(1,0,0);
    Eigen::Vector3d u1 = ref - ref.dot(u_sw)*u_sw; u1.normalize();
    Eigen::Vector3d u2 = u_sw.cross(u1);

    // Cosine rule → q3 (2 branches)
    double cos_int = std::max(-1.0, std::min(1.0,
        (d.L_se*d.L_se + d.L_ew*d.L_ew - dsw*dsw) / (2.0*d.L_se*d.L_ew)));
    double base_q3 = M_PI - std::acos(cos_int);

    constexpr int N_SWIVEL = 16;

    // Closed-form map from a continuous swivel angle theta (plus the
    // discrete elbow/shoulder/wrist branch signs) to the full 7-vector
    // joint solution. Reused both for the initial N_SWIVEL-point discrete
    // search below (to locate the right branch/region) and for a local
    // continuous refinement of theta around the previous tick's solution --
    // the discrete grid alone can only track the arm's continuous
    // self-motion to within one grid step (2*pi/N_SWIVEL), which shows up
    // as an abrupt joint-space jump whenever the true continuous optimum
    // crosses a grid boundary between ticks of a smoothly moving Cartesian
    // target.
    auto qFromTheta = [&](double theta, double q3_signed, int q1_sign,
                           int q5_sign) -> Eigen::VectorXd {
        Eigen::Vector3d E_t = d.shoulder_pivot
            + x_c*u_sw + r_circle*(std::cos(theta)*u1 + std::sin(theta)*u2);
        Eigen::Vector3d dvec = (E_t - d.shoulder_pivot) / d.L_se;
        double cos_q1 = std::max(-1.0, std::min(1.0, dvec.z()));

        double q1     = q1_sign * std::acos(cos_q1);
        double sin_q1 = std::sin(q1);
        double q0     = (std::abs(sin_q1) > 1e-9)
            ? std::atan2(dvec.y()/sin_q1, dvec.x()/sin_q1) : 0.0;

        // Frame at joint 5 with q2=0 → wrist position at q2=0
        Eigen::VectorXd qp = Eigen::VectorXd::Zero(7);
        qp(0)=q0; qp(1)=q1; qp(3)=q3_signed;
        auto [R5, W0] = frameAtJoint(qp, 5);

        // SP1: q2 rotates W0 to W_t around upper-arm axis
        double q2 = poe_sp::sp1(dvec, W0-E_t, W_t-E_t).theta;

        // Frame before joint 4 (wrist) with full (q0,q1,q2,q3)
        Eigen::VectorXd qf = qp;
        qf(2) = q2;
        auto [R_pre, p_pre] = frameAtJoint(qf, 4);

        // ZYZ Euler decomposition of residual rotation
        Eigen::Matrix3d Rres = R_pre.transpose() * R_target * d.R_post_wrist.transpose();
        double cos_q5 = std::max(-1.0, std::min(1.0, Rres(2,2)));

        double q5    = q5_sign * std::acos(cos_q5);
        double sinq5 = std::sin(q5);
        double q4, q6;
        if (std::abs(sinq5) > 1e-9) {
            q4 = std::atan2(q5_sign*Rres(1,2),  q5_sign*Rres(0,2));
            q6 = std::atan2(q5_sign*Rres(2,1), -q5_sign*Rres(2,0));
        } else {
            q4 = 0.0;
            q6 = (cos_q5 > 0)
                ? std::atan2(-Rres(0,1), Rres(0,0))
                : std::atan2( Rres(0,1),-Rres(0,0));
        }

        Eigen::VectorXd qc(7);
        qc << q0, q1, q2, q3_signed, q4, q5, q6;
        return qc;
    };

    struct SrsCand { Eigen::VectorXd q; double theta, q3s; int q1s, q5s; };
    std::vector<SrsCand> cands;

    // Both signs of q3 are generated. q3=-base_q3 is not a second physical
    // elbow configuration -- for every theta it reproduces the identical arm
    // pose via q2->q2+pi, q3->-q3, q4->q4+pi -- but it is a different joint
    // *labeling*, and the arm can be at either one (e.g. after a PTP move to
    // a ready pose with q3<0). Offering only one labeling forces a pi jump in
    // q2/q4 the first time IK runs from such a pose; with both, the
    // closest-to-previous selection below keeps the arm's current labeling.
    for (int swivel_i = 0; swivel_i < N_SWIVEL; ++swivel_i) {
        double theta = -M_PI + swivel_i * (2.0*M_PI / N_SWIVEL);
        for (double q3s : {base_q3, -base_q3}) {
            for (int q1_sign : {+1, -1}) {
                for (int q5_sign : {+1, -1}) {
                    Eigen::VectorXd qc = qFromTheta(theta, q3s, q1_sign, q5_sign);
                    if (!qc.allFinite()) continue;
                    if (verifyCandidate(qc, T))
                        cands.push_back({qc, theta, q3s, q1_sign, q5_sign});
                }
            }
        }
    }

    if (cands.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    {   // dedup near-identical joint-space solutions, keeping the first
        std::vector<SrsCand> u;
        for (auto& c : cands) {
            bool dup=false;
            for (auto& r : u) if (nearDup(c.q, r.q)) { dup=true; break; }
            if (!dup) u.push_back(c);
        }
        cands = std::move(u);
    }

    // First pass: find the best achievable residual across all candidates.
    auto fk = [this](const Eigen::VectorXd& q){return poeFk(q);};
    double best_res = std::numeric_limits<double>::max();
    for (auto& c : cands) {
        double r = fkRes(c.q, T, fk);
        if (r < best_res) best_res = r;
    }
    if (best_res > IK_RESIDUAL_TOL) return -ERR_ROB_IK_OUTOF_REACH;

    // Second pass: among candidates that reach the target essentially as
    // well as the best one (this topology's redundancy means several
    // distinct joint-space solutions routinely do), pick whichever is
    // closest in joint space to the last solution returned -- keeping the
    // arm's actual, continuous self-motion instead of jumping between
    // discrete N_SWIVEL samples. Falls back to the lowest-residual
    // candidate when there's no previous solution to stay close to (e.g.
    // the very first IK call).
    //
    // Per swivel angle there are up to 4 *physically distinct* branches
    // tied in residual (elbow-left/right via q1_sign, wrist-flip/non-flip
    // via q5_sign) -- not just parametrization noise. An unrestricted
    // joint-space nearest-neighbor search across all of them can
    // occasionally jump to a different branch when it happens to look
    // close on one joint, producing large (multi-radian) single-tick
    // jumps. So first try to stay on the previous tick's branch, and only
    // fall back to a full cross-branch search if that branch has no tied
    // candidate this tick (e.g. it became infeasible) or there's no
    // previous branch yet (first call).
    constexpr double kResTol = 1e-6;
    const bool have_seed = (srs_last_q_.size() == static_cast<int>(DoF_)) && srs_have_branch_;
    // Branch to lock: the requested flags {q1 sign, q5 sign} (as JntToCart
    // reports them) when given, else the previous solution's branch.
    const bool have_request = branch.size() >= SRS_NUM_BRANCH_FLAGS;
    const int want_q1s = have_request ? (branch[0] == 0 ? 1 : -1) : srs_last_q1_sign_;
    const int want_q5s = have_request ? (branch[1] == 0 ? 1 : -1) : srs_last_q5_sign_;
    bool branch_available = false;
    if (have_seed || have_request) {
        for (auto& c : cands) {
            if (c.q1s == want_q1s && c.q5s == want_q5s &&
                fkRes(c.q, T, fk) <= best_res + kResTol) { branch_available = true; break; }
        }
    }
    SrsCand best = cands.front();
    double best_score = std::numeric_limits<double>::max();
    for (auto& c : cands) {
        double r = fkRes(c.q, T, fk);
        if (r > best_res + kResTol) continue;
        if (branch_available &&
            !(c.q1s == want_q1s && c.q5s == want_q5s)) continue;
        double score = have_seed ? wrappedDist(c.q, srs_last_q_) : r;
        if (score < best_score) { best_score = score; best = c; }
    }

    // Local continuous refinement: the discrete grid above only locates
    // *which* swivel branch (q3/q1/q5 signs) and which grid cell is
    // closest to the previous solution -- it can't represent the
    // continuously-varying true optimum in between grid points. Ternary-
    // search theta in a one-grid-step window around the winning cell
    // (dist-to-previous-solution is unimodal there) to track the arm's
    // self-motion continuously instead of snapping between N_SWIVEL
    // discrete positions. This is what removes the occasional large joint
    // jump (up to one full grid step, ~0.28 rad measured for N_SWIVEL=16)
    // that otherwise shows up as visible shaking when a smoothly moving
    // Cartesian target's true optimum crosses a grid boundary between ticks.
    // The distance metric wraps each joint mod 2*pi (wrappedDist) so a
    // joint crossing the atan2 +-pi seam doesn't look like a near-2*pi
    // jump to the search.
    if (have_seed) {
        double lo = best.theta - (2.0*M_PI / N_SWIVEL);
        double hi = best.theta + (2.0*M_PI / N_SWIVEL);
        auto dist = [&](double th) {
            Eigen::VectorXd qc = qFromTheta(th, best.q3s, best.q1s, best.q5s);
            if (!qc.allFinite()) return std::numeric_limits<double>::max();
            return wrappedDist(qc, srs_last_q_);
        };
        for (int iter = 0; iter < 40; ++iter) {
            double m1 = lo + (hi-lo)/3.0;
            double m2 = hi - (hi-lo)/3.0;
            if (dist(m1) < dist(m2)) hi = m2; else lo = m1;
        }
        double theta_ref = 0.5*(lo+hi);
        Eigen::VectorXd q_ref = qFromTheta(theta_ref, best.q3s, best.q1s, best.q5s);
        if (q_ref.allFinite() && fkRes(q_ref, T, fk) <= best_res + kResTol &&
            wrappedDist(q_ref, srs_last_q_) < best_score) {
            best.q = q_ref;
        }
    }

    qo = best.q;
    srs_last_q_ = best.q;
    srs_last_q1_sign_ = best.q1s;
    srs_last_q5_sign_ = best.q5s;
    srs_have_branch_ = true;
    return 0;
}

// ===========================================================================
// 7R IK: joint-lock + inner 6R sweep  (Franka/xArm7/any 7R)
// ===========================================================================

Eigen::VectorXd serialArmPOE::buildLockedSubChain(int ki, double q_lock) const {
    Eigen::Matrix3d R_lock = poe_sp::rotMat(axes_[ki], q_lock);
    int last = (int)DoF_ - 1;
    std::vector<Eigen::Vector3d> ax6;
    std::vector<Eigen::Matrix4d> TL6, TR6;

    for (int i=0; i<(int)DoF_; ++i) {
        if (i == ki) continue;
        if (i < ki) {
            ax6.push_back(axes_[i]);
            TL6.push_back(T_left_[i]);
            TR6.push_back(T_right_[i]);
            continue;
        }
        // Post-lock: rotate by R_lock
        Eigen::Vector3d nax = R_lock * axes_[i];
        Eigen::Matrix4d nl = Eigen::Matrix4d::Identity();
        nl.block<3,1>(0,3) = R_lock * T_left_[i].block<3,1>(0,3);
        Eigen::Matrix4d nr = Eigen::Matrix4d::Identity();
        nr.block<3,1>(0,3) = R_lock * T_right_[i].block<3,1>(0,3);

        if (i == last)
            nr.block<3,3>(0,0) = R_lock * T_right_[i].block<3,3>(0,0);

        if (i == ki+1) {
            // Absorb locked joint's T_left + T_right into next joint's T_left
            Eigen::Vector3d tc = T_left_[ki].block<3,1>(0,3)
                + R_lock*(T_right_[ki].block<3,1>(0,3) + T_left_[i].block<3,1>(0,3));
            nl.block<3,1>(0,3) = tc;
        }
        ax6.push_back(nax); TL6.push_back(nl); TR6.push_back(nr);
    }
    return buildKineParams(ax6, TL6, TR6);
}

int serialArmPOE::chooseLockJoint() const {
    // Rank by topology quality of resulting 6R sub-chain (at q_lock=0)
    auto rank = [&](ArmTopology t) -> int {
        switch(t) {
            case ArmTopology::ThreeParallel:            return 0;
            case ArmTopology::SphericalTwoParallel:     return 1;
            case ArmTopology::SphericalTwoIntersecting: return 2;
            case ArmTopology::NoMatch: default:          return 9;
        }
    };
    int best_k=0, best_r=99;
    for (int k=0; k<(int)DoF_; ++k) {
        serialArmPOE sub(6);
        sub.SetGeometry(buildLockedSubChain(k, 0.0));
        int r = rank(sub.topology_);
        if (r < best_r) { best_r=r; best_k=k; }
    }
    return best_k;
}

int serialArmPOE::cartToJntJointLock(const Eigen::Matrix4d& T,
                                       const std::vector<int>& branch,
                                       Eigen::VectorXd& qo) const {
    constexpr int N_SAMPLES = 16;
    std::vector<Eigen::VectorXd> cands;

    // Locking a joint whose axis the entire downstream chain rotates
    // rigidly about as q_lock sweeps (e.g. a waist Z axis with nothing
    // else sharing its frame) never changes the *relative* geometry
    // (parallelism/concurrency) the topology detector looks at -- the
    // sub-chain's topology is identical at every q_lock sample. So if
    // lock_idx_ (chooseLockJoint()'s single best-ranked pick, which
    // defaults to joint 0 when every candidate ties at "no good
    // sub-topology" -- exactly what happens for a geometry deliberately
    // built so no 6-joint subset is SRS/parallel/intersecting, e.g. this
    // Panda-like arm) gives NoMatch once, the *entire* N_SAMPLES sweep is
    // NoMatch and IK fails for every reachable pose, every time. Try the
    // other lock joints before giving up -- unlike joint 0 here, they can
    // have a genuinely q_lock-dependent sub-chain geometry.
    std::vector<int> lock_candidates{lock_idx_};
    for (int k = 0; k < static_cast<int>(DoF_); ++k) {
        if (k != lock_idx_) lock_candidates.push_back(k);
    }

    bool any_sub_chain = false;
    for (int ki : lock_candidates) {
        if (jl_no_analytic_) break;
        for (int si=0; si<N_SAMPLES; ++si) {
            double q_lock = -M_PI + si * (2.0*M_PI / N_SAMPLES);

            serialArmPOE sub(6);
            sub.SetGeometry(buildLockedSubChain(ki, q_lock));
            if (sub.topology_ == ArmTopology::NoMatch || sub.topology_ == ArmTopology::Unknown)
                continue;
            any_sub_chain = true;

            // Build a Pose from T_target (no branch flags — sub-arm picks best)
            Frame fr = eigenToFrame(T);
            Pose  tp(fr);

            Eigen::VectorXd q6(6);
            if (sub.CartToJnt(tp, q6) < 0) continue;

            // Reconstruct 7D solution
            Eigen::VectorXd q7(7);
            for (int i=0; i<ki; ++i) q7(i)      = q6(i);
            q7(ki) = q_lock;
            for (int i=ki; i<6; ++i) q7(i+1)   = q6(i);

            if (verifyCandidate(q7, T)) cands.push_back(q7);
        }
        if (!cands.empty()) break;  // this lock joint produced workable solutions
    }
    if (!any_sub_chain) jl_no_analytic_ = true;

    if (cands.empty()) {
        // No lock joint gives an analytic 6R sub-chain (e.g. Panda: the J4/J7
        // offsets break every spherical/parallel group). Solve numerically:
        // first from the previous solution (continuous tracking), then from
        // zero and a fixed set of restart seeds.
        std::vector<Eigen::VectorXd> seeds;
        if (jl_last_q_.size() == static_cast<int>(DoF_)) seeds.push_back(jl_last_q_);
        seeds.push_back(Eigen::VectorXd::Zero(DoF_));
        std::mt19937 rng(IK_NUM_SEED);
        std::uniform_real_distribution<double> uni(-M_PI, M_PI);
        for (int r=0; r<IK_NUM_RESTARTS; ++r) {
            Eigen::VectorXd sd(DoF_);
            for (int i=0; i<static_cast<int>(DoF_); ++i) sd(i)=uni(rng);
            seeds.push_back(sd);
        }
        for (auto& sd : seeds) {
            Eigen::VectorXd qn;
            if (numericIK(T, sd, qn)) { qo = qn; jl_last_q_ = qn; return 0; }
        }
        return -ERR_ROB_IK_OUTOF_REACH;
    }
    dedup(cands);

    auto fk = [this](const Eigen::VectorXd& q){return poeFk(q);};
    Eigen::VectorXd best = cands.front();
    double best_res = fkRes(best, T, fk);
    for (auto& qc : cands) {
        double r = fkRes(qc, T, fk);
        if (r < best_res) { best_res=r; best=qc; }
    }
    if (best_res > IK_RESIDUAL_TOL) return -ERR_ROB_IK_OUTOF_REACH;
    qo = best; return 0;
}

bool serialArmPOE::numericIK(const Eigen::Matrix4d& T, const Eigen::VectorXd& seed,
                              Eigen::VectorXd& qo) const {
    const int N = static_cast<int>(DoF_);
    Eigen::VectorXd q = seed;
    Eigen::MatrixXd Jt, Jr, J(6, N);
    Eigen::Vector3d pe;
    Eigen::VectorXd e(6);
    for (int it=0; it<IK_NUM_MAX_ITER; ++it) {
        Eigen::Matrix4d Tc = poeFk(q);
        e.head<3>() = T.block<3,1>(0,3) - Tc.block<3,1>(0,3);
        Eigen::AngleAxisd aa(T.block<3,3>(0,0) * Tc.block<3,3>(0,0).transpose());
        e.tail<3>() = aa.angle() * aa.axis();
        if (e.norm() < IK_NUM_CONV_TOL) break;
        calcGeomJacobian(q, Jt, Jr, pe);
        J.topRows<3>() = Jt; J.bottomRows<3>() = Jr;
        Eigen::Matrix<double,6,6> A = J * J.transpose()
            + IK_NUM_DAMPING * IK_NUM_DAMPING * Eigen::Matrix<double,6,6>::Identity();
        Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(e);
        double m = dq.cwiseAbs().maxCoeff();
        if (m > IK_NUM_MAX_STEP) dq *= IK_NUM_MAX_STEP / m;
        q += dq;
    }
    q = q.unaryExpr([](double a){ return poe_sp::wrapToPi(a); });
    auto fk = [this](const Eigen::VectorXd& x){ return poeFk(x); };
    if (fkRes(q, T, fk) > IK_RESIDUAL_TOL) return false;
    qo = q;
    return true;
}

// ===========================================================================
// Public FK overrides
// ===========================================================================

int serialArmPOE::JntToCart(const Eigen::VectorXd& q, Pose& p) {
    std::ostringstream s;
    if (!initialized_) { s<<GetName()<<": not initialized\n"; LOG_ERROR(s); return -ERR_ROB_PARAM_NOT_INITIALIZED; }
    p.setFrame(eigenToFrame(poeFk(q)));

    // Joint turns: floor(q[i] / 2π), adjusted so remainder stays in (-π, π]
    std::vector<int> jt(static_cast<int>(DoF_), 0);
    for (int i = 0; i < static_cast<int>(DoF_); i++) {
        double t = std::floor(q[i] / (2.0 * M_PI));
        if (q[i] - t * (2.0 * M_PI) > M_PI) t += 1.0;
        jt[i] = static_cast<int>(t);
    }
    p.setJointTurns(jt);

    if (topology_ == ArmTopology::SevenRSRS) {
        // The {q1 sign, q5 sign} branch cartToJntSRS honors. FK deliberately
        // does not touch the IK's continuity state (srs_last_q_): planner
        // look-ahead and user FK calls on other joint values would otherwise
        // yank the interpolator's next IK tick away from the arm's path.
        p.setBranchFlags({signFlag(q(SRS_SHOULDER_JOINT)), signFlag(q(SRS_WRIST_JOINT)), 0});
        return 0;
    }

    if (topology_ == ArmTopology::ThreeParallel ||
        topology_ == ArmTopology::SphericalTwoParallel ||
        topology_ == ArmTopology::SphericalTwoIntersecting) {
        p.setBranchFlags(sixRBranch(q));
        return 0;
    }

    // Branch (oi, ii): try each outer×inner combination {0,1}×{0,1} via IK on
    // the FK result, then keep the candidate whose joint solution is closest to q.
    // This mirrors the serialArm/sixaxis_1 pattern of computing config from joints.
    Eigen::Matrix4d T = frameToEigen(defaultBaseOff_.Inverse() * eigenToFrame(poeFk(q)));
    int best_oi = 0, best_ii = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int oi = 0; oi <= 1; oi++) {
        for (int ii = 0; ii <= 1; ii++) {
            std::vector<int> br{oi, ii};
            Eigen::VectorXd qs(DoF_);
            int ret = -1;
            switch (topology_) {
                case ArmTopology::ThreeParallel:
                    ret = cartToJntThreeParallel(T, br, qs); break;
                case ArmTopology::SphericalTwoParallel:
                    ret = cartToJntSphericalTwoParallel(T, br, qs); break;
                case ArmTopology::SphericalTwoIntersecting:
                    ret = cartToJntSphericalTwoIntersecting(T, br, qs); break;
                case ArmTopology::SevenRSRS:
                    ret = cartToJntSRS(T, br, qs); break;
                case ArmTopology::SevenRJointLock:
                    ret = cartToJntJointLock(T, br, qs); break;
                default: break;
            }
            if (ret == 0) {
                double d = (qs - q).norm();
                if (d < best_dist) { best_dist = d; best_oi = oi; best_ii = ii; }
            }
        }
    }
    // Pad to 3 elements (a trailing, unused 0) to match the legacy
    // int<->vector branch encoding used throughout the rest of the
    // pipeline: SingleInt2RobnuxBranch (pose.cpp) always resizes to 3
    // (the old DH left/right,up/down,flip/nonflip convention), so a
    // Cartesian target built in Python from this pose's branch int (e.g.
    // LocData(..., cfg=pose.G, ...)) always decodes back to 3 elements.
    // Left at 2, TrajectorySegment::setBoundCond's config-match check
    // (CompareRobotConfigTurn, which rejects on size alone) would reject
    // every such target as "config not matching", aborting the segment.
    // pickBest() only ever reads br[0]/br[1], so appending -- not
    // prepending -- leaves that lookup unaffected.
    p.setBranchFlags({best_oi, best_ii, 0});
    return 0;
}

int serialArmPOE::JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                              Pose& p, Twist& v) {
    std::ostringstream s;
    if (!initialized_) { s<<GetName()<<": not initialized\n"; LOG_ERROR(s); return -ERR_ROB_PARAM_NOT_INITIALIZED; }
    Eigen::MatrixXd Jt,Jr; Eigen::Vector3d pe;
    calcGeomJacobian(q,Jt,Jr,pe);
    p.setFrame(eigenToFrame(poeFk(q)));
    Eigen::Vector3d vt=Jt*qdot, vr=Jr*qdot;
    v.setLinearVel(Vec(vt.x(),vt.y(),vt.z())); v.setAngularVel(Vec(vr.x(),vr.y(),vr.z()));
    return 0;
}

int serialArmPOE::JntToCart(const Eigen::VectorXd&, const Eigen::VectorXd&,
                              const Eigen::VectorXd&, Pose&, Twist&, Twist&) {
    std::ostringstream s; s<<GetName()<<": acceleration FK not implemented\n"; LOG_ALARM(s); return -1;
}

// ===========================================================================
// Jacobian
// ===========================================================================

int serialArmPOE::CalcJacobian(const Eigen::VectorXd&, Pose& p,
                                Eigen::MatrixXd& Jpt, Eigen::MatrixXd& Jpr, bool wj) {
    std::ostringstream s;
    if (!initialized_) { s<<GetName()<<": not initialized\n"; LOG_ERROR(s); return -ERR_ROB_PARAM_NOT_INITIALIZED; }
    Eigen::VectorXd q = theta_.head(DoF_);
    Eigen::MatrixXd Jt,Jr; Eigen::Vector3d pe;
    calcGeomJacobian(q,Jt,Jr,pe);
    if (wj) {
        Rotation Rb=defaultBaseOff_.getRotation(); Eigen::Matrix3d R;
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) R(r,c)=Rb(r,c);
        Jt=R*Jt; Jr=R*Jr;
    }
    Jpt=Jt; Jpr=Jr; p.setFrame(eigenToFrame(poeFk(q))); return 0;
}

// ===========================================================================
// CartToJnt (position) — topology dispatcher
// ===========================================================================

int serialArmPOE::CartToJnt(const Pose& p, Eigen::VectorXd& q) {
    std::ostringstream s;
    if (!initialized_) { s<<GetName()<<": not initialized\n"; LOG_ERROR(s); return -ERR_ROB_PARAM_NOT_INITIALIZED; }
    if (useCalibrated_) { s<<GetName()<<": requires canonical params\n"; LOG_ERROR(s); return -ERR_ROB_DEFAULT_IK_NOT_CANO; }

    Frame tip; p.getFrame(&tip);
    Eigen::Matrix4d T = frameToEigen(defaultBaseOff_.Inverse() * tip);
    std::vector<int> br; p.getBranchFlags(&br);
    if (q.size() != (int)DoF_) q.resize(DoF_);

    switch (topology_) {
        case ArmTopology::ThreeParallel:
            return cartToJntThreeParallel(T, br, q);
        case ArmTopology::SphericalTwoParallel:
            return cartToJntSphericalTwoParallel(T, br, q);
        case ArmTopology::SphericalTwoIntersecting:
            return cartToJntSphericalTwoIntersecting(T, br, q);
        case ArmTopology::SevenRSRS:
            return cartToJntSRS(T, br, q);
        case ArmTopology::SevenRJointLock:
            return cartToJntJointLock(T, br, q);
        default:
            s<<GetName()<<": no IK for topology "<<(int)topology_<<"\n"; LOG_ALARM(s);
            return -ERR_ROB_IK_OUTOF_REACH;
    }
}

int serialArmPOE::CartToJnt(const Pose& p, const Twist& v,
                              Eigen::VectorXd& q, Eigen::VectorXd& qdot) {
    return serialArm::CartToJnt(p, v, q, qdot);
}

int serialArmPOE::CartToJnt(const Pose&, const Twist&, const Twist&,
                              Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&) {
    std::ostringstream s; s<<GetName()<<": acceleration IK not implemented\n"; LOG_ALARM(s); return -1;
}

// ===========================================================================
// Misc overrides
// ===========================================================================

int serialArmPOE::CalcPassive(const Eigen::VectorXd&, const Pose&, Eigen::VectorXd& qp) {
    qp.resize(0); return 0;
}
bool serialArmPOE::PickSubJacobian(const Eigen::MatrixXd& Jt, const Eigen::MatrixXd& Jr,
                                    Eigen::MatrixXd& Jst, Eigen::MatrixXd& Jsr, bool) {
    Jst=Jt; Jsr=Jr; return true;
}
double serialArmPOE::PickCartErr(const Eigen::Vector3d& eT, const Eigen::Vector3d& eR,
                                  Eigen::VectorXd& b, bool) {
    b.resize(6); b.head<3>()=eT; b.tail<3>()=eR;
    return std::sqrt(eT.squaredNorm()+eR.squaredNorm());
}
void serialArmPOE::UpdateDH(const Eigen::VectorXd& q, Eigen::VectorXd& th, Eigen::VectorXd&) const {
    th=q; const_cast<Eigen::VectorXd&>(theta_)=q;
}
void serialArmPOE::UpdateDH(const Eigen::VectorXd&, const Eigen::VectorXd& j, Eigen::VectorXd& nd) const {
    nd=j;
}
void serialArmPOE::UpdateConfigTurn(const Eigen::VectorXd&, const Eigen::VectorXd&,
                                     std::vector<int>&, std::vector<int>&) const {
    // POE arms don't use DH theta/d parameters; branch/turns are computed
    // directly from joint angles in JntToCart via IK back-classification.
}

}  // namespace kinematics_lib
