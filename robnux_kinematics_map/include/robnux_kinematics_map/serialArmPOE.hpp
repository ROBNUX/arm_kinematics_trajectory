#ifndef ROBNUX_KINEMATICS_MAP_SERIALARMPOE_HPP_
#define ROBNUX_KINEMATICS_MAP_SERIALARMPOE_HPP_

#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "robnux_kinematics_map/serialArm.hpp"
#include "robnux_kinematics_map/poe_subproblems.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace kinematics_lib {

/**
 * @brief Kinematic family detected from POE geometry.
 *
 * 6R dispatch order (ssik dispatcher.py):
 *   1. ThreeParallel            axes[1] ∥ axes[2] ∥ axes[3]          (UR3/5/10)
 *   2. SphericalTwoParallel     spherical wrist (3,4,5)+axes[1]∥axes[2] (Puma,Fanuc,KUKA KR)
 *   3. SphericalTwoIntersecting spherical wrist + ||p[1]|| ≈ 0         (IRB120,xArm6,Lite6)
 *
 * 7R dispatch order (ssik dispatcher.py):
 *   4. SevenRSRS      axes[0,1,2] concurrent + axes[4,5,6] concurrent  (iiwa,Rizon,Kassow)
 *   5. SevenRJointLock any other 7R (sweep one locked joint over inner 6R) (Franka,xArm7)
 */
enum class ArmTopology : int {
    Unknown                  = 0,
    ThreeParallel            = 1,
    SphericalTwoParallel     = 2,
    SphericalTwoIntersecting = 3,
    SevenRSRS                = 4,
    SevenRJointLock          = 5,
    NoMatch                  = 6,
};

/** Precomputed SRS arm constants (cached once in SetGeometry). */
struct SrsData {
    Eigen::Vector3d shoulder_pivot;   ///< common point of axes 0,1,2
    Eigen::Vector3d wrist_pivot;      ///< common point of axes 4,5,6
    double          L_se;             ///< shoulder-to-elbow distance at q=0
    double          L_ew;             ///< elbow-to-wrist distance at q=0
    Eigen::Vector3d ee_offset_local;  ///< FK(0)[:3,3] - wrist_pivot
    Eigen::Matrix3d R_post_wrist;     ///< T_right[6][:3,:3] home rotation
};

/**
 * @brief Serial arm kinematics using Product-of-Exponentials (POE) representation.
 *
 * Extends serialArm with POE-based FK and closed-form analytical IK for:
 *
 * **6R arms**
 *   - ThreeParallel:             UR3, UR5, UR10 and any arm with axes[1‥3] parallel
 *   - SphericalTwoParallel:      Puma 560, Fanuc LR/CR, KUKA KR
 *   - SphericalTwoIntersecting:  ABB IRB120, uFactory xArm6, Lite6
 *
 * **7R arms**
 *   - SevenRSRS:       KUKA iiwa LBR 7/14, Flexiv Rizon 4/10, Kassow KR810  (Singh-Kreutz)
 *   - SevenRJointLock: Franka Panda/FR3, uFactory xArm7, any other 7R        (joint-lock sweep)
 *
 * SetGeometry parameter layout (kine_para length = 35 * dof):
 *   [ax_0, ay_0, az_0, …, ax_{N-1}, ay_{N-1}, az_{N-1},   (3*N joint axes, base frame)
 *    T_left_0  row-major 4×4, …, T_left_{N-1}  row-major,  (16*N)
 *    T_right_0 row-major 4×4, …, T_right_{N-1} row-major]  (16*N)
 *
 * CartToJnt branch-flag encoding:
 *   branch[0] = outer loop index, branch[1] = inner loop index.
 *   Falls back to lowest-FK-residual solution when absent or out of range.
 */
class KINEMATICS_API serialArmPOE : public serialArm {
 public:
  explicit serialArmPOE(size_t dof = 6);

  // --- Configuration ---
  void SetGeometry(const Eigen::VectorXd& kine_para) override;
  std::string GetName() const override;

  // --- Forward Kinematics ---
  int JntToCart(const Eigen::VectorXd& q, Pose& p) override;
  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                Pose& p, Twist& v) override;
  int JntToCart(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                const Eigen::VectorXd& qddot, Pose& p, Twist& v, Twist& a) override;

  // --- Inverse Kinematics ---
  int CartToJnt(const Pose& p, Eigen::VectorXd& q) override;
  int CartToJnt(const Pose& p, const Twist& v,
                Eigen::VectorXd& q, Eigen::VectorXd& qdot) override;
  int CartToJnt(const Pose& p, const Twist& v, const Twist& a,
                Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                Eigen::VectorXd& qddot) override;

  // --- Jacobian (3×DoF geometric) ---
  int CalcJacobian(const Eigen::VectorXd& kine_para, Pose& p,
                   Eigen::MatrixXd& Jp_t, Eigen::MatrixXd& Jp_r,
                   bool world_jac = false) override;
  int CalcPassive(const Eigen::VectorXd& q, const Pose& p,
                  Eigen::VectorXd& qpassive) override;
  bool PickSubJacobian(const Eigen::MatrixXd& Jp_t, const Eigen::MatrixXd& Jp_r,
                       Eigen::MatrixXd& Js_t, Eigen::MatrixXd& Js_r,
                       bool reduction = false) override;
  double PickCartErr(const Eigen::Vector3d& errT, const Eigen::Vector3d& errR,
                     Eigen::VectorXd& b, bool reduction = false) override;

  // --- DH stubs (no-ops in POE path) ---
  void UpdateDH(const Eigen::VectorXd& q,
                Eigen::VectorXd& theta, Eigen::VectorXd& d) const override;
  void UpdateDH(const Eigen::VectorXd& orig_dh, const Eigen::VectorXd& jnt,
                Eigen::VectorXd& new_dh) const override;
  void UpdateConfigTurn(const Eigen::VectorXd& theta, const Eigen::VectorXd& d,
                        std::vector<int>& branchFlags,
                        std::vector<int>& jointTurns) const override;

 protected:
  // ---- POE primitives ----

  Eigen::Matrix4d poeFk(const Eigen::VectorXd& q) const;

  int calcGeomJacobian(const Eigen::VectorXd& q,
                       Eigen::MatrixXd& Jt, Eigen::MatrixXd& Jr,
                       Eigen::Vector3d& p_ee) const;

  bool verifyCandidate(const Eigen::VectorXd& q, const Eigen::Matrix4d& T_target,
                       double pos_tol = 1e-4, double rot_tol = 1e-4) const;

  // Frame (R, p) at joint joint_idx BEFORE that joint's rotation applies.
  // Uses the stored axes_ / T_left_ / T_right_.
  std::pair<Eigen::Matrix3d, Eigen::Vector3d>
      frameAtJoint(const Eigen::VectorXd& q, int joint_idx) const;

  // Cumulative sum of T_left[:3,3] — joint origins in base frame at q=0.
  std::vector<Eigen::Vector3d> jointOrigins() const;

  // ---- Topology detection ----

  ArmTopology detectTopology() const;

  // Geometric predicates (port of ssik/kinematics/predicates.py).
  static bool axisParallel(const Eigen::Vector3d& a,
                           const Eigen::Vector3d& b, double tol = 1e-6);
  static bool axisIntersect(const Eigen::Vector3d& a, const Eigen::Vector3d& oa,
                            const Eigen::Vector3d& b, const Eigen::Vector3d& ob,
                            double par_tol = 1e-6, double dist_tol = 1e-6);

  int findThreeConsecutiveParallel(double par_tol = 1e-6) const;
  int findThreeConsecutiveIntersecting(const std::vector<Eigen::Vector3d>& origins,
                                       double par_tol = 1e-6,
                                       double dist_tol = 1e-6) const;

  // Common intersection point of a subset of joint axes. Returns nullopt when
  // axes are parallel or don't share a common point within dist_tol.
  std::optional<Eigen::Vector3d>
      findCommonPoint(const std::vector<int>& indices,
                      const std::vector<Eigen::Vector3d>& origins,
                      double par_tol = 1e-3,
                      double dist_tol = 1e-3) const;

  // SRS-class check for 7R arm. Returns SrsData when topology matches.
  std::optional<SrsData> findSRS7R() const;

  // ---- 6R analytical IK solvers ----

  // Shared wrist step for all spherical-wrist solvers.
  void solveSphericalWrist(const Eigen::Matrix3d& r_36,
                           std::vector<Eigen::Vector3d>& q456) const;

  int cartToJntThreeParallel(const Eigen::Matrix4d& T_target,
                             const std::vector<int>& branch,
                             Eigen::VectorXd& q_out) const;
  int cartToJntSphericalTwoParallel(const Eigen::Matrix4d& T_target,
                                    const std::vector<int>& branch,
                                    Eigen::VectorXd& q_out) const;
  int cartToJntSphericalTwoIntersecting(const Eigen::Matrix4d& T_target,
                                        const std::vector<int>& branch,
                                        Eigen::VectorXd& q_out) const;

  // ---- 7R analytical IK solvers ----

  // Singh-Kreutz SRS solver.
  int cartToJntSRS(const Eigen::Matrix4d& T_target,
                   const std::vector<int>& branch,
                   Eigen::VectorXd& q_out) const;

  // Joint-lock + inner 6R sweep.
  int cartToJntJointLock(const Eigen::Matrix4d& T_target,
                          const std::vector<int>& branch,
                          Eigen::VectorXd& q_out) const;

  // Build sub-chain geometry with joint lock_idx locked at q_lock.
  // Returns (axes6, T_left6, T_right6) as flat kine_para for a new serialArmPOE(6).
  Eigen::VectorXd buildLockedSubChain(int lock_idx, double q_lock) const;

  // Choose which joint to lock for the joint-lock solver (topology-rank based).
  int chooseLockJoint() const;

  // ---- Storage ----

  std::vector<Eigen::Vector3d> axes_;
  std::vector<Eigen::Matrix4d> T_left_;
  std::vector<Eigen::Matrix4d> T_right_;

  ArmTopology                  topology_{ArmTopology::Unknown};
  mutable SrsData              srs_data_{};          ///< valid when topology_ == SevenRSRS
  mutable int                  lock_idx_{0};          ///< valid when topology_ == SevenRJointLock
};

// ---------------------------------------------------------------------------
// Thin wrappers so pluginlib can instantiate serialArmPOE without arguments.
// DoF must be fixed at construction time; SetGeometry validates parameter
// count against DoF_, so 6R and 7R need separate plugin classes.
// ---------------------------------------------------------------------------

class serialArmPOE_6R : public serialArmPOE {
 public:
  serialArmPOE_6R() : serialArmPOE(6) {}
  std::string GetName() const override { return "serialArmPOE_6R"; }
};

class serialArmPOE_7R : public serialArmPOE {
 public:
  serialArmPOE_7R() : serialArmPOE(7) {}
  std::string GetName() const override { return "serialArmPOE_7R"; }
};

}  // namespace kinematics_lib

#endif  // ROBNUX_KINEMATICS_MAP_SERIALARMPOE_HPP_
