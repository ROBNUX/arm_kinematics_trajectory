/*
 * File:   rotation.hpp
 * Author: Leon Liu, ROBNUX LLC (leon@robnux.net)
 *
 * Created on April 4, 2021, 5:36 PM
 * It contains definitions/APIs about 3 by 3 rotation matrix, and the conversion
 * between Euler angle, Quaternion and  rotation matrix
 */

#ifndef ROTATION_HPP
#define ROTATION_HPP
#include <math.h>

#include "kdl_common/vec.hpp"
namespace kinematics_lib {
/*
 *  @class EulerAngle, Quaternion, Rotation
 *  @brief provides APIs related to 3D rotations, it includes
 * Euler/Quaternion/Rotation matrix three different
 *  representations
 */
class Quaternion;
class Rotation;

enum COMMON_API EulerType {
  eZYX = 0,
  eZYZ = 1,
};

class COMMON_API EulerAngle : public Vec {
 public:
  EulerAngle();
  EulerAngle(const double A, const double B, const double C,
             const EulerType& eType);
  //! Get the quaternion of this matrix
  //! \post the norm of (x,y,z,w) is 1
  bool GetQuaternion(Quaternion* q) const;
  bool GetRotation(Rotation* r) const;

  double first() { return data[0]; }

  double second() { return data[1]; }

  double third() { return data[2]; }

  Vec getVec() { return Vec(data[0], data[1], data[2]); }

  void setVec(const Vec& p) {
    this->set_x(p.x());
    this->set_y(p.y());
    this->set_z(p.z());
  }

  EulerType getType() { return this->tp_; }

  void setType(const EulerType& tp) { this->tp_ = tp; }

 private:
  EulerType tp_;
};

class COMMON_API Quaternion {
 public:
  //! default constructor
  Quaternion();
  //! copy constructor
  Quaternion(const Quaternion& q);
  //! ROS defines the 4 coordinate element of a Quaternion element
  // by (x,y,z,w)
  Quaternion(const double w, const double x, const double y, const double z);
  //! Quaternion that represents a rotation about axis by an angle <angle>
  Quaternion(const Vec& axis, const double angle);
  //! Create quaternion based upon Z-Y-X Euler angles (i.e., through
  // Yaw-Pitch-Roll)
  void SetEulerZYX(const double yaw, const double pitch, const double roll);
  //! Create quaternion based upn a rotation about an axis by an angle
  // <angle>
  void setRotation(const Vec& axis, const double angle);

  /*
   *  basic functions about Quaternion
   */
  //! Return the angle between this quaternion and the other along shortest
  // path
  double shortestAngle(const Quaternion& q) const;
  //! return the angle [0, 2pi] of rotation represented by this quaternion
  double getAngle() const;
  //! Rturn the angle [0, Pi] of rotation represented by this quaternion
  double getAngleShortestPath(Vec* axis) const;
  //! Return the axis of the rotation represented by this quaternion
  bool GetEulerZYX(double* yaw, double* pitch, double* roll) const;
  //! get rotation axis; abanoned, as getAngleShortestPath already did this
  // Vec getAxis()  const;
  //! get real part
  double getRealPart() const;
  // get virtual part of this quaternion
  Vec getVirtualPart() const;
  //! Return the inverse of this quaternion
  Quaternion inverse() const;
  //! return conjugate
  Quaternion conjugate() const;
  //! return the normalized version of this quaternion, while this
  // quaternion is not changed
  Quaternion normalized() const;
  //! sqNorm of the quaternion
  double sqNorm() const;
  //! normalize the current quaternion, i.e., this quaternion is changed
  void normalize();
  //! Return the length of the quaternion
  double length() const;

  /*
   *  access its members
   */
  double x() const;
  double y() const;
  double z() const;
  double w() const;

  /*
   * set values of members
   */
  void setX(const double x);
  void setY(const double y);
  void setZ(const double z);
  void setW(const double w);
  void setRealPart(const double w);
  void setVirtualPart(const Vec& vec);

  /*
   * operators
   */
  //! assignment
  Quaternion& operator=(const Quaternion& q);
  //! multiple with a scalar
  Quaternion operator*(const double s) const;
  //! q *= s,    i.e. q=q*s, scale the current quaternion
  Quaternion& operator*=(const double s);
  //! q1 *= q;
  Quaternion& operator*=(const Quaternion& q);
  //! multiplication with another rotation
  friend COMMON_API Quaternion operator*(const Quaternion& lq,
                                         const Quaternion& rq);
  //! for use in diffential quaternion
  friend COMMON_API Quaternion operator*(const double coef,
                                         const Quaternion& lq);
  //! q2 = q1 + q;
  Quaternion operator+(const Quaternion& q) const;
  //! q1+=q;
  Quaternion& operator+=(const Quaternion& q);
  //! q2 = q1 - q
  Quaternion operator-(const Quaternion& q) const;
  //! q2 = -q, i.e., return the negative of this quaternion
  Quaternion operator-() const;
  //! q1 -= q, i.e., subtract out q from this quaternion
  Quaternion& operator-=(const Quaternion& q);
  //! q2 = q1 / s
  Quaternion operator/(const double s) const;
  //! q1 /= s;
  Quaternion& operator/=(const double s);
  //! The literal equality operator==(), also identical.
  bool operator==(const Quaternion& a);
  //! dot product with another quaternion (treat quaternion as a R^4 vector)
  double dot(const Quaternion& rhs) const;

  /*
   *  static public function
   */
  static Quaternion getIdentity();
  static Quaternion getZero();

  //! convert to output string
  // if Euler=true, output Euler angles, else output Quaternion
  std::string ToString(bool Euler) const {
    std::ostringstream strs;
    if (Euler) {
      double yaw, pitch, roll;
      bool ret = GetEulerZYX(&yaw, &pitch, &roll);
      if (ret) {
        strs << "Yaw " << yaw << " Pitch " << pitch << " Roll " << roll << " ";
      } else {
        strs << "fails to compute Euler angles ";
      }
    } else {
      strs << "q0 " << q[0] << " q1 " << q[1] << " q2 " << q[2] << " q3 "
           << q[3] << " ";
    }
    std::string str = strs.str();
    return str;
  }

  Eigen::Vector4d ToEigenVec();
  //! get a vector from an Eigen::Vector4d
  static Quaternion FromEigenVec(const Eigen::Vector4d& v);

 private:
  //  the 4 quaternion coodinates
  // q[1]-q[3]:  rotation axis * sin(theta/2), q[0] = cos(theta/2)
  double q[4];
};

/*
 * @brief
 */
class COMMON_API Rotation {
 public:
  //! default constructor
  Rotation();
  //! rotation with 9 inputs, representing 9 elements in rotation matrix
  Rotation(const double m1, const double m2, const double m3, const double m4,
           const double m5, const double m6, const double m7, const double m8,
           const double m9);
  //! constructor from Euler Yaw-pitch-roll angles
  Rotation(const double yaw, const double pitch, const double roll);
  //! constructor based upon three axies (unit vector as well as
  // mutually perpendicular
  Rotation(const Vec& x, const Vec& y, const Vec& z);
  //! constructor based upon rotation axis, and rotation angle about this axis
  Rotation(const Vec& axis, const double angle);
  //! constructor from Quaternion
  Rotation(const Quaternion& q);
  //! constructor based upon rotation axis z only, axis x and y can be any two
  //! mutually perpendicular axis
  // mostly used in tool teaching
  Rotation(const Vec& z);

  /*
   * Operators
   */
  //! assignment operator
  Rotation& operator=(const Rotation& arg);

  //!  Defines a multiplication R*ioTaskStateV between a Rotation R and
  // a Vector V.
  //! Complexity : 9M+6A
  Vec operator*(const Vec& v) const;

  //!    Access to elements 0..2,0..2, bounds are checked
  double operator()(int i, int j) const;

  //! set new x-axis
  void UnitX(const Vec& X);

  //! set new y-axis
  void UnitY(const Vec& Y);

  //! set new z-axis
  void UnitZ(const Vec& Z);

  //! multiplication with another rotation
  friend COMMON_API Rotation operator*(const Rotation& lr, const Rotation& rr);

  //! Transpose function
  Rotation Transpose() const;

  //! Gives back the inverse rotation matrix of *this.
  Rotation Inverse() const;

  /*
   *  static functions, mainly for constructing special rotations directly
   */
  //! Gives back an identity rotaton matrix
  static Rotation Identity();

  //! The Rot... static functions give the value of the appropriate
  // rotation matrix back.
  static Rotation RotX(double angle);
  //! The Rot... static functions give the value of the appropriate
  // rotation matrix back.
  static Rotation RotY(double angle);
  //! The Rot... static functions give the value of the appropriate
  // rotation matrix back.
  static Rotation RotZ(double angle);

  // Note: In the following all EulerVel and EulerAcc are based upon body frame
  //! Given Euler angle, angular velocity, compute EulerDot
  static bool GetEulerVelZYX(const Vec& Euler, const Vec& angularVel,
                             Vec* EulerDot);
  //! Given Euler angle, dotEuler, and angular Acc, compute EulerDDot
  static bool GetEulerAccZYX(const Vec& Euler, const Vec& EulerDot,
                             const Vec& angularAcc, Vec* EulerDDot);
  //! Given Euler angle, EulerDot, compute angular Velocity
  static bool GetTwistVelZYX(const Vec& Euler, const Vec& EulerDot,
                             Vec* angularVel);
  //! Given Euler angle, EulerDot, EulerDDot, compute angular acceleration
  static bool GetTwistAccZYX(const Vec& Euler, const Vec& EulerDot,
                             const Vec& EulerDDot, Vec* angularAcc);
  //! Given Euler angle, angular velocity, compute EulerDot
  static bool GetEulerVelZYZ(const Vec& Euler, const Vec& angularVel,
                             Vec* EulerDot);
  //! Given Euler angle, dotEuler, and angular Acc, compute EulerDDot
  static bool GetEulerAccZYZ(const Vec& Euler, const Vec& EulerDot,
                             const Vec& angularAcc, Vec* EulerDDot);
  //! Given Euler angle, EulerDot, compute angular Velocity
  static bool GetTwistVelZYZ(const Vec& Euler, const Vec& EulerDot,
                             Vec* angularVel);
  //! Given Euler angle, EulerDot, EulerDDot, compute angular acceleration
  static bool GetTwistAccZYZ(const Vec& Euler, const Vec& EulerDot,
                             const Vec& EulerDDot, Vec* angularAcc);

  /*
   *  main APIs to use rotations
   */
  //! get Rotation axis and angle related to this rotation
  /* in the case of angle == 0 : rot axis is undefined and choosen
   * to be +/- Z-axis
   * in the case of angle == PI : 2 solutions, positive Z-component
   * of the axis is choosen.
   * @result returns the rotation angle (between [0..PI] )
   */
  bool GetAxisAngle(Vec* axis, double* angle) const;

  //! Get the quaternion of this matrix
  //! \post the norm of (x,y,z,w) is 1
  bool GetQuaternion(Quaternion* q) const;

  //! Get Yaw-Pitch-Roll Euler angles
  bool GetEulerZYX(double* yaw, double* pitch, double* roll) const;
  //! Get ZYZ Euler angles
  bool GetEulerZYZ(double* A, double* B, double* C) const;
  //! Set ZYX Euler angles
  void SetEulerZYX(const double yaw, const double pitch, const double roll);
  //! Set ZYZ Euler angles
  void SetEulerZYZ(const double A, const double B, const double C);

  //! serialize the rotation matrix
  std::string ToString() const;

  //! The literal equality operator==(), also identical.
  bool operator==(const Rotation& a);

  //! Access to the underlying unitvectors of the rotation matrix
  Vec UnitX() const { return Vec(data[0], data[3], data[6]); }

  //! Access to the underlying unitvectors of the rotation matrix
  Vec UnitY() const { return Vec(data[1], data[4], data[7]); }

  //! Access to the underlying unitvectors of the rotation matrix
  Vec UnitZ() const { return Vec(data[2], data[5], data[8]); }

  //! transfer a rotation matrix to an Eigen matrix
  Eigen::Matrix3d ToEigenMat();

  //! transfer to an ZYX Euler Eigen Vector
  Eigen::Vector3d ToEigenVecZYX();

  //! transfer to an ZYZ Euler Eigen Vector
  Eigen::Vector3d ToEigenVecZYZ();

 private:
  double data[9];
};

}  // namespace kinematics_lib
#endif /* ROTATION_HPP */
