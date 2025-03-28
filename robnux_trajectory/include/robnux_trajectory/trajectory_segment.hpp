/*
 * File:   trajectory_segment.hpp
 * Author: Leon Liu, ROBNUX LLC, leon@robnux.net
 *
 * Created on April 3, 2021, 2:07 PM
 */

#ifndef TRAJECTORY_SEGMENT_HPP
#define TRAJECTORY_SEGMENT_HPP
#include "robnux_kdl_common/pose.hpp"
#include "robnux_kdl_common/rotation.hpp"
#include "robnux_kdl_common/twistWrench.hpp"
#include "robnux_kdl_common/vec.hpp"
#include "robnux_trajectory/base_profile.hpp"
#include "robnux_trajectory/base_rot_trajectory.hpp"
#include "robnux_trajectory/base_trans_trajectory.hpp"
#include "robnux_trajectory/trajectory.hpp"

namespace kinematics_lib {
/*
 * set up the most basic template for a trajectory segment,
 * but has some basic implementation of the APIs
 */
class TRAJECTORY_API TrajectorySegment : public CartTrajectory {
 public:
  //  robot char_length to compare which motion is
  // significant? rotation or translation?
  // then we will plan that part first, and the remaining part will
  //
  TrajectorySegment();  // default to free

  //! destructor
  ~TrajectorySegment();

  /*
   * @brief  time durtion of this trajectory segment
   */
  double Duration() const override;

  /*
   * @brief  translational dist (meters) if the translation time >
   * rotation time, else, return rotational angles,
   * this distance will be used in continuous trajectory blending,
   * because the starting blending is usually
   * described by the proportion (or percentage) of the traveled distance.
   */
  double Dist() const override;

  /*
   * Initialize internal translational and
   * rotational trajectory planner
   */
  bool InitializeTrajectoryPlanners(
      const std::shared_ptr<BaseTransTrajectory> &transPlanner,
      const std::shared_ptr<BaseRotTrajectory> &rotPlanner);

  bool setBoundCond(const Pose &start_pos, const Twist &start_vel,
                    const Pose &end_pos, const Twist &end_vel);

  /*
   *  compute current Frame, Generalized Vel, and Generalized Acc
   * at <time> , return false if there is any error
   */
  virtual bool Trajectory(const double time, Pose *ps, Twist *tw,
                          Twist *acctw) const override;

  /*
   *  translational component
   */
  virtual bool TranslationalTrajectory(const double time, Vec *p, Vec *pdot,
                                       Vec *ppdot) const override;

  /*
   * rotational component in Rotation matrix
   */
  virtual bool RotationalTrajectory(const double time, Rotation *rot, Vec *w,
                                    Vec *wdot) const override;
  /*
   *  compute orientation trajectory in euler angles {euler,
   *eulerdot, eulerddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, EulerAngle *euler,
                                    Vec *eulerdot,
                                    Vec *eulerddot) const override;
  /*
   *  compute orientation trajectory in quaternion {q, qdot, qddot}  at <time>
   */
  virtual bool RotationalTrajectory(const double time, Quaternion *q,
                                    Quaternion *qdot,
                                    Quaternion *qddot) const override;

  //! Key API to calculate Jnt (or RelJnt), JntVel, and JntAcc of this
  //! trajectory
  virtual bool Trajectory(const double time, Eigen::VectorXd *jp,
                          Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

  virtual bool RelTrajectory(const double time, Eigen::VectorXd *jp,
                             Eigen::VectorXd *jv, Eigen::VectorXd *ja) const;

  // set tail blend dist, note should be less than dist_ of the traj itself
  // 0 means no blend, dist_ menas full blending from starting point
  virtual void SetTailBlendDist(const double b_dist);
  // set tail blend percentage (0% no blend, 100% blend from starting point)
  virtual void SetTailBlendPercent(const double b_perc);

 protected:
  //  two trajectory components
  std::shared_ptr<BaseTransTrajectory> trans_T_;
  std::shared_ptr<BaseRotTrajectory> rot_T_;

  // duration of this trajectory segment
  double duration_;

  // major-distance (could be translation meter or rotation angles)
  // traveled by this trajectory
  double dist_;

  // whether planning has been done, can start retrieving trajectory data
  bool planDone_;

  // whether or not translation dominated
  bool transDominated_;

  // branchFlags for this segment (robot config)
  std::vector<int> branchFlags_;

  // jointTurns for this segment (joint turns)
  std::vector<int> jointTurns_;

  // starting joint vector
  Eigen::VectorXd sq_;
};

}  // namespace kinematics_lib
#endif /* TRAJECTORY_SEGMENT_HPP */
