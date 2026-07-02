#include "robnux_kinematics_map/serialArmPOE.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
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
struct Cand { Eigen::VectorXd q; double res; int oi, ii; };
static double fkRes(const Eigen::VectorXd& q, const Eigen::Matrix4d& T,
             const std::function<Eigen::Matrix4d(const Eigen::VectorXd&)>& fk) {
    Eigen::Matrix4d Tf=fk(q);
    double pe=(Tf.block<3,1>(0,3)-T.block<3,1>(0,3)).norm();
    Eigen::AngleAxisd aa(Tf.block<3,3>(0,0).transpose()*T.block<3,3>(0,0));
    return pe+std::abs(aa.angle());
}
static Cand pickBest(std::vector<Cand>& cv, const std::vector<int>& br) {
    std::sort(cv.begin(),cv.end(),[](const Cand&a,const Cand&b){return a.res<b.res;});
    if (br.size()>=2) {
        int w0=br[0],w1=br[1];
        for (auto& c:cv) if(c.oi==w0&&c.ii==w1) return c;
    }
    return cv.front();
}
// Wrap angle to (-pi, pi]
static double wrapPi(double a) { return std::fmod(a+M_PI,2*M_PI)-M_PI; }
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
            Eigen::VectorXd qc(6); qc<<q1,q2,q3,q4,q5,q6;
            cv.push_back({qc,fkRes(qc,T,fk),oi,ii++});
        }
        ++oi;
    }
    if(cv.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    auto best=pickBest(cv,br);
    if(best.res>1e-4) return -ERR_ROB_IK_OUTOF_REACH;
    qo=best.q; return 0;
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
                cv.push_back({qc,fkRes(qc,T,fk),oi,ii});
            }
            ++ii;
        }
        ++oi;
    }
    if(cv.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    auto best=pickBest(cv,br);
    if(best.res>1e-4) return -ERR_ROB_IK_OUTOF_REACH;
    qo=best.q; return 0;
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
                cv.push_back({qc,fkRes(qc,T,fk),oi,ii});
            }
            ++ii;
        }
        ++oi;
    }
    if(cv.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    auto best=pickBest(cv,br);
    if(best.res>1e-4) return -ERR_ROB_IK_OUTOF_REACH;
    qo=best.q; return 0;
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
    std::vector<Eigen::VectorXd> cands;

    for (double q3_signed : {base_q3, -base_q3}) {
        for (int swivel_i = 0; swivel_i < N_SWIVEL; ++swivel_i) {
            double theta = -M_PI + swivel_i * (2.0*M_PI / N_SWIVEL);
            Eigen::Vector3d E_t = d.shoulder_pivot
                + x_c*u_sw + r_circle*(std::cos(theta)*u1 + std::sin(theta)*u2);
            Eigen::Vector3d dvec = (E_t - d.shoulder_pivot) / d.L_se;
            double cos_q1 = std::max(-1.0, std::min(1.0, dvec.z()));

            for (int q1_sign : {+1, -1}) {
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

                for (int q5_sign : {+1, -1}) {
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
                    if (!qc.allFinite()) continue;
                    if (verifyCandidate(qc, T)) cands.push_back(qc);
                }
            }
        }
    }

    if (cands.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    dedup(cands);

    // Select by branch flag or best FK
    auto fk = [this](const Eigen::VectorXd& q){return poeFk(q);};
    Eigen::VectorXd best = cands.front();
    double best_res = fkRes(best, T, fk);
    for (auto& qc : cands) {
        double r = fkRes(qc, T, fk);
        if (r < best_res) { best_res = r; best = qc; }
    }

    if (best_res > 1e-4) return -ERR_ROB_IK_OUTOF_REACH;
    qo = best; return 0;
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

    for (int ki : lock_candidates) {
        for (int si=0; si<N_SAMPLES; ++si) {
            double q_lock = -M_PI + si * (2.0*M_PI / N_SAMPLES);

            serialArmPOE sub(6);
            sub.SetGeometry(buildLockedSubChain(ki, q_lock));
            if (sub.topology_ == ArmTopology::NoMatch || sub.topology_ == ArmTopology::Unknown)
                continue;

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

    if (cands.empty()) return -ERR_ROB_IK_OUTOF_REACH;
    dedup(cands);

    auto fk = [this](const Eigen::VectorXd& q){return poeFk(q);};
    Eigen::VectorXd best = cands.front();
    double best_res = fkRes(best, T, fk);
    for (auto& qc : cands) {
        double r = fkRes(qc, T, fk);
        if (r < best_res) { best_res=r; best=qc; }
    }
    if (best_res > 1e-4) return -ERR_ROB_IK_OUTOF_REACH;
    qo = best; return 0;
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
    p.setBranchFlags({best_oi, best_ii});
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
