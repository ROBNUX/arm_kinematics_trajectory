#include "robnux_trajectory/arc_trajectory.hpp"

#include <cmath>

namespace kinematics_lib {
ArcTrajectory::ArcTrajectory()
    : center_(Vec(0, 0, 0)),
      xAxis_(Vec(1, 0, 0)),
      yAxis_(Vec(0, 1, 0)),
      radius_(0),
      start_angle_(0),
      end_angle_(0),
      start_spd_(0),
      end_spd_(0),
      BaseTransTrajectory() {}

ArcTrajectory::ArcTrajectory(const Vec &center, const Vec &xAxis,
                             const Vec &yAxis, const double radius,
                             const double start_angle, const double end_angle)
    : center_(center),
      xAxis_(xAxis),
      yAxis_(yAxis),
      radius_(radius),
      start_angle_(start_angle),
      end_angle_(end_angle),
      start_spd_(0),
      end_spd_(0),
      BaseTransTrajectory() {
  // here we always require end_angle >= start_angle, i.e. we
  // always pick
}

bool ArcTrajectory::SetCenterRadius(const Vec &center, const Vec &xAxis,
                                    const Vec &yAxis, const double radius,
                                    const double start_angle,
                                    const double end_angle) {
  center_ = center;
  xAxis_ = xAxis;
  yAxis_ = yAxis;
  radius_ = radius;
  start_angle_ = start_angle;
  end_angle_ = end_angle;
  start_spd_ = 0;
  end_spd_ = 0;
  return true;
}

ArcTrajectory::ArcTrajectory(const Vec &p1, const Vec &p2, const Vec &p3)
    : BaseTransTrajectory() {
  Vec v1 = p2 - p1;
  Vec v2 = p3 - p1;
  double v1v1, v2v2, v1v2;
  v1v1 = v1.dot(v1);
  v2v2 = v2.dot(v2);
  v1v2 = v1.dot(v2);
  double det = v1v1 * v2v2 - v1v2 * v1v2;
  if (det < EPSILON_CIRC_3P) {
    //        std::ostringstream ss;
    //        ss << "three points p1, p2, p3 are singular "
    //                  << " impossible to find a circle passing through them"
    //                  << std::endl;
    //        LOG_ERROR(ss);
    //        throw KinematicsException("ArcTrajectory constructor exception:
    //        three "
    //                "input points are singular ",
    //                                 -12);
  }

  double base = 0.5 / det;
  double k1 = base * v2v2 * (v1v1 - v1v2);
  double k2 = base * v1v1 * (v2v2 - v1v2);
  this->center_ = p1 + v1 * k1 + v2 * k2;  // center
  this->xAxis_ = p1 - this->center_;
  this->radius_ = this->xAxis_.Norm();

  if (this->radius_ < EPSILON_MIN_RADIUS) {
    //        std::ostringstream ss;
    //        ss << "the circle passing through them "
    //                  << "has too small radius" << std::endl;
    //        LOG_ERROR(ss);
    //        throw KinematicsException("ArcTrajectory constructor exception:
    //        the "
    //                "arc radius is too small",
    //                                 -13);
  }
  this->xAxis_ = this->xAxis_ / this->radius_;
  // determine the y-axis is quite tricky, need to consider 3 cases
  Vec v3 = (p2 - this->center_) / this->radius_;
  Vec v4 = (p3 - this->center_) / this->radius_;
  double cosv3 = v3.dot(this->xAxis_);
  double cosv4 = v4.dot(this->xAxis_);

  Vec dy1 = v3 - cosv3 * this->xAxis_;
  double norm_dy1 = dy1.Norm();

  Vec dy2 = v4 - cosv4 * this->xAxis_;
  double norm_dy2 = dy2.Norm();

  //  case 1: p1,p2 lines on a diameter
  if (norm_dy1 < EPSILON_TWOPOINTS_DIAMETER) {
    this->yAxis_ = -dy2 / norm_dy2;
  } else if (norm_dy2 < EPSILON_TWOPOINTS_DIAMETER) {
    // case 2: p1, p3 lines on a diameter
    this->yAxis_ = dy1 / norm_dy1;
  } else if (dy1.dot(dy2) < 0) {
    // case 3:  p2, p3 lines two sides of x-axis
    this->yAxis_ = dy1 / norm_dy1;
  } else if (cosv3 >= cosv4) {
    this->yAxis_ = dy1 / norm_dy1;
  } else {
    this->yAxis_ = -dy2 / norm_dy2;
  }
  this->start_angle_ = 0;
  this->end_angle_ = std::atan2(v4.dot(this->yAxis_), cosv4);
  this->start_spd_ = 0;
  this->end_spd_ = 0;
}

bool ArcTrajectory::SetThreePoints(const Vec &p1, const Vec &p2,
                                   const Vec &p3) {
  Vec v1 = p2 - p1;
  Vec v2 = p3 - p1;
  double v1v1, v2v2, v1v2;
  v1v1 = v1.dot(v1);
  v2v2 = v2.dot(v2);
  v1v2 = v1.dot(v2);
  double det = v1v1 * v2v2 - v1v2 * v1v2;
  if (det < EPSILON_CIRC_3P) {
    //        std::ostringstream ss;
    //        ss << "three points p1, p2, p3 are singular "
    //                  << " impossible to find a circle passing through them"
    //                  << std::endl;
    //        LOG_ERROR(ss);
    return false;
  }

  double base = 0.5 / det;
  double k1 = base * v2v2 * (v1v1 - v1v2);
  double k2 = base * v1v1 * (v2v2 - v1v2);
  this->center_ = p1 + v1 * k1 + v2 * k2;  // center
  this->xAxis_ = p1 - this->center_;
  this->radius_ = this->xAxis_.Norm();

  if (this->radius_ < EPSILON_MIN_RADIUS) {
    //        std::ostringstream ss;
    //        ss << "the circle passing through them "
    //                  << "has too small radius" << std::endl;
    //        LOG_ERROR(ss);
    return false;
  }
  this->xAxis_ = this->xAxis_ / this->radius_;
  // determine the y-axis is quite tricky, need to consider 3 cases
  Vec v3 = (p2 - this->center_) / this->radius_;
  Vec v4 = (p3 - this->center_) / this->radius_;
  double cosv3 = v3.dot(this->xAxis_);
  double cosv4 = v4.dot(this->xAxis_);

  Vec dy1 = v3 - cosv3 * this->xAxis_;
  double norm_dy1 = dy1.Norm();

  Vec dy2 = v4 - cosv4 * this->xAxis_;
  double norm_dy2 = dy2.Norm();

  //  case 1: p1,p2 lines on a diameter
  if (norm_dy1 < EPSILON_TWOPOINTS_DIAMETER) {
    this->yAxis_ = -dy2 / norm_dy2;
  } else if (norm_dy2 < EPSILON_TWOPOINTS_DIAMETER) {
    // case 2: p1, p3 lines on a diameter
    this->yAxis_ = dy1 / norm_dy1;
  } else if (dy1.dot(dy2) < 0) {
    // case 3:  p2, p3 lines two sides of x-axis
    this->yAxis_ = dy1 / norm_dy1;
  } else if (cosv3 >= cosv4) {
    this->yAxis_ = dy1 / norm_dy1;
  } else {
    this->yAxis_ = -dy2 / norm_dy2;
  }
  this->start_angle_ = 0;
  this->end_angle_ = std::atan2(v4.dot(this->yAxis_), cosv4);
  this->start_spd_ = 0;
  this->end_spd_ = 0;
  return true;
}

bool ArcTrajectory::setBoundaryCond(const Vec &start_pos, const Vec &start_vel,
                                    const Vec &end_pos, const Vec &end_vel) {
  if (!isProfSet_ || !trans_prof_) {
    //        std::ostringstream ss;
    //        ss << "profile is not set yet, in function "
    //        << __FUNCTION__ << ", line " << __LINE__ << std::endl;
    //        LOG_ERROR(ss);
    return false;
  }
  Vec st = start_pos - center_;
  start_angle_ = std::atan2(st.dot(this->yAxis_), st.dot(this->xAxis_));
  Vec ed = end_pos - center_;
  end_angle_ = std::atan2(ed.dot(this->yAxis_), ed.dot(this->xAxis_));
  // tangents at start and end angle
  Vec st_tangent = -sin(start_angle_) * xAxis_ + cos(start_angle_) * yAxis_;
  Vec ed_tangent = -sin(end_angle_) * xAxis_ + cos(end_angle_) * yAxis_;
  if (end_angle_ < start_angle_) {
    end_angle_ += 2 * M_PI;
  }
  // distance traveled
  dist_ = radius_ * (end_angle_ - start_angle_);
  if (dist_ < MIN_TRANS_DIST) {
    duration_ = 0;
  } else {
    // start and end velocity
    start_spd_ = start_vel.dot(st_tangent);
    end_spd_ = end_vel.dot(ed_tangent);
    this->trans_prof_->setBoundaryCondition(0, dist_, start_spd_, end_spd_);
    duration_ = this->trans_prof_->Duration();
  }
  planDone_ = true;
  return true;
}

bool ArcTrajectory::Trajectory(const double time, Vec *p, Vec *pdot,
                               Vec *pddot) const {
  if (!this->planDone_ || !p || !pdot || !pddot) {
    //     std::ostringstream ss;
    //     ss << "The trajectory is not planned or input pointers to "
    //               << __FUNCTION__ << " is null" << std::endl;
    //     LOG_ERROR(ss);
    return false;
  } else {
    // default values
    *p = center_ +
         radius_ * (cos(start_angle_) * xAxis_ + sin(start_angle_) * yAxis_);
    *pdot =
        start_spd_ * (-sin(start_angle_) * xAxis_ + cos(start_angle_) * yAxis_);
    *pddot = (start_spd_ * start_spd_ / radius_) *
             (-cos(start_angle_) * xAxis_ - sin(start_angle_) * yAxis_);
    if (duration_ > MIN_TRAJ_DURATION) {
      double X, Xdot, Xddot;
      this->trans_prof_->Trajectory(time, &X, &Xdot, &Xddot);
      double theta = start_angle_ + (X / radius_);
      *p = center_ + radius_ * (cos(theta) * xAxis_ + sin(theta) * yAxis_);
      *pdot = Xdot * (-sin(theta) * xAxis_ + cos(theta) * yAxis_);
      *pddot = Xddot * (-sin(theta) * xAxis_ + cos(theta) * yAxis_) +
               (Xdot * Xdot / radius_) *
                   (-cos(theta) * xAxis_ - sin(theta) * yAxis_);
    }
    return true;
  }
}

}  // namespace kinematics_lib
