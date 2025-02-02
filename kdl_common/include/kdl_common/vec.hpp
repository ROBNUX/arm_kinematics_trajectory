/***************************************************************************
  tag: Leon Guanfeng Liu  March 30 2021,  Vec.hpp

                        Vec.hpp -  description
                           -------------------
    begin                : March 30, 2021
    copyright            : (C) 2021 Leon Liu
    email                : leon@robnux.net

******************************************************************************/
/*****************************************************************************
*  \author
*  	Leon Liu, ROBNUX LLC
*
*  \version
*		 V 0.1.0
*
*	\par History
*		- $log$
*
*	\par Release
*		$Id: $
*
*  @brief: This file provides all classes, functions, APIs related to using, and
* manipulating 3D-vectors

****************************************************************************/

#ifndef KINEMATICS_LIB_VEC_HPP
#define KINEMATICS_LIB_VEC_HPP
#include <math.h>

#include <eigen3/Eigen/Core>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Logger/Logger.h"
#include "kdl_common/common_constants.hpp"
#include "kdl_common/common_exportdecl.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//#define K_EPSILON (0.0001)

using namespace ROBNUXLogging;
namespace kinematics_lib {

//! define kinematics exception class
class COMMON_API KinematicsException : public std::exception {
 public:
  KinematicsException(const std::string& msg, const int errorCode)
      : m_msg_(msg), errorCode_(errorCode) {}

  ~KinematicsException(){};
  virtual const char* what() const throw() {
    std::ostringstream strs;
    strs << m_msg_ << " error code " << errorCode_;
    std::string str = strs.str();
    // std::cout << str << std::endl;
    return str.c_str();
  }

 private:
  const std::string m_msg_;
  int errorCode_;
};
/*
 *  @class Vec
 *  @brief provides APIs related to 3D vectors
 */
class COMMON_API Vec {
 public:
  //! default constructor
  Vec();

  //! Eigen::Vector3d input
  Vec(const Eigen::Vector3d& v);

  //! Constructs a vector out of the three values x, y and z
  Vec(const double x, const double y, const double z);

  //! Assignment constructor. The normal copy by value semantics.
  Vec(const Vec& arg);

  //! Assignment operator. The normal copy by value semantics.
  Vec& operator=(const Vec& arg);

  //! Access to elements, range checked from 0..2
  double operator()(int index) const;

  //! Access to elements, range checked from 0..2
  double& operator()(int index);

  //! Equivalent to double operator()(int index) const
  double operator[](int index) const { return this->operator()(index); }

  //! Equivalent to double& operator()(int index)
  double& operator[](int index) { return this->operator()(index); }
  //! access three components
  double x() const;
  double y() const;
  double z() const;
  //! set three components
  void set_x(const double x_);
  void set_y(const double y_);
  void set_z(const double z_);

  //! subtracts a vector from the Vector object itself
  Vec& operator-=(const Vec& arg);

  //! Adds a vector from the Vector object itself
  Vec& operator+=(const Vec& arg);

  //! Multiply a vector with a scalar
  Vec& operator*=(const double value);

  //! Scalar multiplication is defined
  friend COMMON_API Vec operator*(const Vec& lhs, const double rhs);
  //! Scalar multiplication is defined
  friend COMMON_API Vec operator*(const double lhs, const Vec& rhs);
  //! Scalar division is defined

  Vec operator/(const double rhs) const;
  Vec operator+(const Vec& rhs) const;
  Vec operator-(const Vec& rhs) const;
  Vec operator*(const Vec& rhs) const;
  Vec operator-() const;
  double dot(const Vec& rhs) const;
  bool operator==(const Vec& rhs) const;

  //! To have a uniform operator to put an element to zero, for scalar values
  //! and for objects.
  void SetToZero();

  //! @return a zero vector
  static Vec Zero();

  /** Normalizes this vector and returns its norm
   * makes v a unit vector and returns the norm of v.
   * if v is smaller than eps, Vector(1,0,0) is returned with norm 0.
   * if this is not good, check the return value of this method.
   */
  double Normalize(double eps = K_EPSILON);

  //! return a normalized vec
  Vec NormalizeVec(double eps = K_EPSILON) const;

  //!    @return the norm of the vector
  double Norm() const;

  //! return square of the norm
  double sqNorm() const;

  //! added by Leon Guanfeng Liu to serialize the data
  std::string ToString() const {
    std::ostringstream strs;
    strs << "x " << data[0] << " y " << data[1] << " z " << data[2] << " ";
    std::string str = strs.str();
    return str;
  }

  //! get a vector from an Eigen::Vector3d
  static Vec FromEigenVec(const Eigen::Vector3d& v);
  //! transfer a Vec to an Eigen Vector
  Eigen::Vector3d ToEigenVec();
  //! transfer to skew-symmetric matrix hat<V>
  bool ToHat(Eigen::Matrix3d* hatv);

 protected:
  //  internal data
  double data[3];
};

// convert std::vector to VectorXd
COMMON_API bool StdVec2EigenVec(const std::vector<double>& qin,
                                Eigen::VectorXd* qout);
COMMON_API bool EigenVec2StdVec(const Eigen::VectorXd& qin,
                                std::vector<double>* qout);
}  // namespace kinematics_lib
#endif  // KINEMATICS_LIB_VEC_HPP