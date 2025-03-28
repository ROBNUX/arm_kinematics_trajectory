/*
 * File:   trajectory.hpp
 * Author: Leon Liu ROBNUX LLC (leon@robnux.net)
 *
 * Created on Jun 4, 2021, 12:20 PM
 *
 * @brief, this file implements an abstract Cartesian/Joint Trajectory class
 */
#ifndef KINEMATICS_LIB_TRAJECTORY_HPP_
#define KINEMATICS_LIB_TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/twistWrench.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "robnux_trajectory/base_profile.hpp"

namespace kinematics_lib {
/*
 *  Interface of Abstract Trajectory.
 *  Note: trajectroy_segment represents cartesian traj
 */

enum Traj_Type {
  eCart = 0,  // cartesian trajectory
  eJnt = 1,   // Jnt trajectory
};

class CartTrajectory {
 public:
  virtual ~CartTrajectory() {}
  //! duration of this traj
  virtual double Duration() const = 0;
  //! length of this traj
  virtual double Dist() const = 0;

  //! Key API to calculate Frame, Twist, and Twist Acc of this trajectory
  virtual bool Trajectory(const double time, Pose *ps, Twist *tw,
                          Twist *acctw) const = 0;

  //! For convenience, we directly compute translational
  //! and rotational components
  virtual bool TranslationalTrajectory(const double time, Vec *p, Vec *pdot,
                                       Vec *ppdot) const = 0;

  virtual bool RotationalTrajectory(const double time, Rotation *rot, Vec *w,
                                    Vec *wdot) const = 0;
  /*
   *  compute orientation trajectory in euler angles {euler,
   *eulerdot, eulerddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, EulerAngle *euler,
                                    Vec *eulerdot, Vec *eulerddot) const = 0;
  /*
   *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, Quaternion *q,
                                    Quaternion *qdot,
                                    Quaternion *qddot) const = 0;

  //! Key API to calculate Jnt, JntVel, and JntAcc of this trajectory as a
  // function of time
  virtual bool Trajectory(const double time, Eigen::VectorXd *jp,
                          Eigen::VectorXd *jv, Eigen::VectorXd *ja) const = 0;

  //! Key API to calculate real movement of Jnt, JntSpd, and JntAcc
  // of this trajectory as a function of time
  virtual bool RelTrajectory(const double time, Eigen::VectorXd *jp,
                             Eigen::VectorXd *jv,
                             Eigen::VectorXd *ja) const = 0;

  virtual Traj_Type getTrajType() { return eCart; }

  virtual bool IsTailBlended() { return isTailBlended_; }

  // set tail blend dist, note should be less than dist_ of the traj itself
  // 0 means no blend, dist_ menas full blending from starting point
  virtual void SetTailBlendDist(const double b_dist) = 0;
  // set tail blend percentage (0% no blend, 100% blend from starting point)
  virtual void SetTailBlendPercent(const double b_perc) = 0;

  virtual double getTailBlendDist() { return tailBlendDist_; }

  virtual double getTailBlendTime() {  // when the blending begins on this seg.
    return tailBlendTime_;
  }

  /*
   *  set kinematics map. Recall one robot only has one unique kinematics map,
   * so here we use shared pointer
   */
  virtual void setKinematicMap(
      const boost::shared_ptr<BaseKinematicMap> &kMap) {
    armMap_ = kMap;
  }

 protected:
  // means whether a traj is blended from the head, i.e. it is
  // blending with the immediate previous trajectory segment
  // bool isHeadBlended_;
  // means whether a traj is blended at the end, i.t., it is blending with
  // the immediate next trajectory segment
  bool isTailBlended_;

  // tail blend distance
  double tailBlendDist_ = 0;

  // tail blend time
  double tailBlendTime_ = 0;  // the time from the end of this current traj
                              // to trigger next trajectory segment

  // robot kinematics map, will only be deleted outside of the class
  boost::shared_ptr<BaseKinematicMap> armMap_;
};

class JntTrajectory : public CartTrajectory {
 public:
  JntTrajectory(const size_t numJnts);

  virtual ~JntTrajectory() {}
  //! duration of this traj
  virtual double Duration() const { return duration_; }
  //! length of this traj
  virtual double Dist() const { return dist_; }
  //! set boundary condition in joint space
  virtual bool setBoundaryCond(const Eigen::VectorXd &start_jp,
                               const Eigen::VectorXd &start_jv,
                               const Eigen::VectorXd &end_jp,
                               const Eigen::VectorXd &end_jv);

  //! set boundary condition in Cartesian space
  virtual bool setBoundaryCond(const Pose &start_pos, const Twist &start_vel,
                               const Pose &end_pos, const Twist &end_vel);
  //! set boundary condition in Cartesian space with translation and quaternion
  //! inputs
  virtual bool setBoundaryCond(const Pose &start_pos, const Vec &sp_dot,
                               const Quaternion &sq_dot, const Pose &end_pos,
                               const Vec &ep_dot, const Quaternion &eq_dot);

  /*
   *  set Profile; recall different joints might have different profile
   */
  virtual void setProfile(
      const std::vector<std::shared_ptr<BaseProfile> > &jnt_prof) {
    if (jnt_prof.size() == numJnts_) {
      jnt_prof_ = jnt_prof;
      this->isProfSet_ = true;
    }
  }

  //! For convenience, we directly compute translational
  //! and rotational components
  virtual bool TranslationalTrajectory(const double time, Vec *p, Vec *pdot,
                                       Vec *ppdot) const;

  virtual bool RotationalTrajectory(const double time, Rotation *rot, Vec *w,
                                    Vec *wdot) const;
  /*
   *  compute orientation trajectory in euler angles {euler,
   *eulerdot, eulerddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, EulerAngle *euler,
                                    Vec *eulerdot, Vec *eulerddot) const;
  /*
   *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, Quaternion *q,
                                    Quaternion *qdot, Quaternion *qddot) const;

  // set tail blend dist, note should be less than dist_ of the traj itself
  // 0 means no blend, dist_ menas full blending from starting point
  virtual void SetTailBlendDist(const double b_dist);
  // set tail blend percentage (0% no blend, 100% blend from starting point)
  virtual void SetTailBlendPercent(const double b_perc);

  //! Key API to calculate Jnt (or RelJnt), JntVel, and JntAcc of this
  //! trajectory
  virtual bool Trajectory(const double time, Eigen::VectorXd *jp,
                          Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

  virtual bool RelTrajectory(const double time, Eigen::VectorXd *jp,
                             Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

  virtual bool Trajectory(const double time, Pose *p, Twist *v,
                          Twist *a) const override;

  Traj_Type getTrajType() override { return eJnt; }

 protected:
  // number of joints
  size_t numJnts_;

  // slowest channel
  size_t slowest_jnt_index_ = 0;  // slowest joint index

  // duration of this joint trajectory
  double duration_;
  // distance traveled of this trajectory
  double dist_;
  //! motion profiles: recall each BaseProfile gives pos(t),
  // vel(t), acc(t) of 1-D motion
  // sometimes, we wish to have each DoF of the joints to
  // use different profile, so
  // we need a vector of BaseProfile
  std::vector<std::shared_ptr<BaseProfile> > jnt_prof_;

  // whether the required profile has been set
  bool isProfSet_;

  // whether planning has been completed
  bool planDone_;

  // boundary conditions
  Eigen::VectorXd sq_, eq_, sqdot_, eqdot_;
  Eigen::VectorXd jntVec_;
};

}  // namespace kinematics_lib

#endif /* KINEMATICS_LIB_TRAJECTORY_HPP_ */
