/*
 * File:   twistWrench.hpp
 * Author: Leon Liu, ROBNUX LLC (leon@robnux.net)
 *
 * Created on April 12, 2021, 12:22 PM
 */

#ifndef TWISTWRENCH_HPP
#define TWISTWRENCH_HPP
#include "kdl_common/vec.hpp"
namespace kinematics_lib {
/**
 * \brief represents the combination of a force and a torque, or called
 * generalized force
 */
class COMMON_API Wrench {
 private:
  Vec force;   //!< Force that is applied at the origin of the current ref frame
  Vec torque;  //!< Torque that is applied at the origin of the current ref
               //!< frame
 public:
  // mainly for print out the content
  std::string ToString() {
    std::ostringstream strs;
    strs << "force " << force.ToString() << " torque " << torque.ToString();
    std::string str = strs.str();
    return str;
  }

  //! Does  initialise force and torque to zero via the underlying
  // constructor of Vector
  Wrench() : force(), torque(){};
  Wrench(const Vec& _force, const Vec& _torque)
      : force(_force), torque(_torque){};

  //! Operators
  Wrench& operator-=(const Wrench& arg);
  Wrench& operator+=(const Wrench& arg);

  //! index-based access to components, first force(0..2), then torque(3..5)
  double& operator()(int i);

  //! index-based access to components, first force(0..2), then torque(3..5)
  //! for use with a const Wrench
  double operator()(int i) const;

  double operator[](int index) const { return this->operator()(index); }

  double& operator[](int index) { return this->operator()(index); }

  //! Scalar multiplication
  friend COMMON_API Wrench operator*(const Wrench& lhs, double rhs);
  //! Scalar multiplication
  friend COMMON_API Wrench operator*(double lhs, const Wrench& rhs);
  //! Scalar division
  friend COMMON_API Wrench operator/(const Wrench& lhs, double rhs);

  friend COMMON_API Wrench operator+(const Wrench& lhs, const Wrench& rhs);
  friend COMMON_API Wrench operator-(const Wrench& lhs, const Wrench& rhs);

  //! An unary - operator
  friend COMMON_API Wrench operator-(const Wrench& arg);

  //! get component
  Vec getForce() const;
  Vec getTorque() const;

  //! set component
  void setForce(const Vec& linearF);
  void setTorque(const Vec& angularF);

  //! @return a zero Wrench
  static Wrench Zero();

  //! Changes the reference point of the wrench.
  //! The vector v_base_AB is expressed in the same base as the twist
  //! The vector v_base_AB is a vector from the old point to
  //! the new point.
  //!
  //! Complexity : 6M+6A
  Wrench RefPoint(const Vec& v_base_AB) const;

  //! == operator
  friend COMMON_API bool operator==(const Wrench& a, const Wrench& b);
};

class COMMON_API Twist {
 private:
  Vec vel;  //!< The velocity of that point
  Vec rot;  //!< The rotational velocity of that point.
 public:
  // added by Leon Guanfeng Liu to serialize the data
  std::string ToString() {
    std::ostringstream strs;
    strs << "translational vel " << vel.ToString() << " rotational vel "
         << rot.ToString();
    std::string str = strs.str();
    return str;
  }

  //! The default constructor initialises to Zero via the
  // constructor of Vector.
  Twist() : vel(), rot(){};

  Twist(const Vec& _vel, const Vec& _rot) : vel(_vel), rot(_rot){};

  Twist& operator-=(const Twist& arg);
  Twist& operator+=(const Twist& arg);
  //! index-based access to components, first vel(0..2), then rot(3..5)
  double& operator()(int i);

  //! index-based access to components, first vel(0..2), then rot(3..5)
  //! For use with a const Twist
  double operator()(int i) const;

  double operator[](int index) const { return this->operator()(index); }
  double& operator[](int index) { return this->operator()(index); }

  friend COMMON_API Twist operator*(const Twist& lhs, double rhs);
  friend COMMON_API Twist operator*(double lhs, const Twist& rhs);
  friend COMMON_API Twist operator/(const Twist& lhs, double rhs);
  friend COMMON_API Twist operator+(const Twist& lhs, const Twist& rhs);
  friend COMMON_API Twist operator-(const Twist& lhs, const Twist& rhs);
  friend COMMON_API Twist operator-(const Twist& arg);
  friend COMMON_API double dot(const Twist& lhs, const Wrench& rhs);
  friend COMMON_API double dot(const Wrench& rhs, const Twist& lhs);

  // Spatial cross product for 6d motion vectors, beware
  // all of them have to be expressed in the same reference frame/point
  friend COMMON_API Twist operator*(const Twist& lhs, const Twist& rhs);
  // Spatial cross product for 6d force vectors, beware all
  // of them have to be expressed in the same reference frame/point
  friend COMMON_API Wrench operator*(const Twist& lhs, const Wrench& rhs);

  //! @return a zero Twist : Twist(Vector::Zero(),Vector::Zero())
  static Twist Zero();

  //! get linear and angular component
  Vec getLinearVel() const;
  Vec getAngularVel() const;
  //! set linear or angular component
  void setLinearVel(const Vec& v);
  void setAngularVel(const Vec& w);

  //! Changes the reference point of the twist.
  //! The vector v_base_AB is expressed in the same base as the twist
  //! The vector v_base_AB is a vector from the old point to
  //! the new point.
  //!
  //! Complexity : 6M+6A
  Twist RefPoint(const Vec& v_base_AB) const;

  //! The literal equality operator==(), also identical.
  inline friend COMMON_API bool operator==(const Twist& a, const Twist& b);

  // norm of the twist
  double getLinearNorm() { return vel.Norm(); }

  double getRotNorm() { return rot.Norm(); }
};

}  // namespace kinematics_lib
#endif /* TWISTWRENCH_HPP */
