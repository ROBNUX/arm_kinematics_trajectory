#include "robnux_kdl_common/vec.hpp"

namespace kinematics_lib {
Vec::Vec() {
  this->data[0] = 0;
  this->data[1] = 0;
  this->data[2] = 0;
}

Vec::Vec(const Eigen::Vector3d& v) {
  data[0] = v(0);
  data[1] = v(1);
  data[2] = v(2);
}

Vec::Vec(const double x, const double y, const double z) {
  data[0] = x;
  data[1] = y;
  data[2] = z;
}

Vec::Vec(const Vec& arg) {
  data[0] = arg.x();
  data[1] = arg.y();
  data[2] = arg.z();
}

Vec& Vec::operator=(const Vec& arg) {
  data[0] = arg.x();
  data[1] = arg.y();
  data[2] = arg.z();
  return *this;
}

double Vec::operator()(int index) const {
  if (0 <= index && index <= 2) {
    return data[index];
  } else {
    throw KinematicsException("index out of range", -1);
  }
}

double& Vec::operator()(int index) {
  if (0 <= index && index <= 2) {
    return data[index];
  } else {
    throw KinematicsException("index out of range", -1);
  }
}

double Vec::x() const { return data[0]; }

double Vec::y() const { return data[1]; }

double Vec::z() const { return data[2]; }

void Vec::set_x(double x_) { data[0] = x_; }
void Vec::set_y(double y_) { data[1] = y_; }
void Vec::set_z(double z_) { data[2] = z_; }

Vec& Vec::operator+=(const Vec& arg) {
  // Complexity : 3A
  data[0] += arg.data[0];
  data[1] += arg.data[1];
  data[2] += arg.data[2];
  return *this;
}

Vec& Vec::operator-=(const Vec& arg) {
  // Complexity : 3A
  data[0] -= arg.data[0];
  data[1] -= arg.data[1];
  data[2] -= arg.data[2];
  return *this;
}

Vec& Vec::operator*=(const double value) {
  data[0] *= value;
  data[1] *= value;
  data[2] *= value;
  return *this;
}

Vec operator*(const Vec& lhs, const double rhs) {
  Vec tmp;
  tmp.set_x(lhs.x() * rhs);
  tmp.set_y(lhs.y() * rhs);
  tmp.set_z(lhs.z() * rhs);
  return tmp;
}

Vec operator*(const double lhs, const Vec& rhs) {
  Vec tmp;
  tmp.set_x(lhs * rhs.x());
  tmp.set_y(lhs * rhs.y());
  tmp.set_z(lhs * rhs.z());
  return tmp;
}

Vec Vec::operator+(const Vec& rhs) const {
  Vec tmp(data[0] + rhs.x(), data[1] + rhs.y(), data[2] + rhs.z());
  return tmp;
}

Vec Vec::operator-(const Vec& rhs) const {
  Vec tmp(data[0] - rhs.x(), data[1] - rhs.y(), data[2] - rhs.z());
  return tmp;
}

Vec Vec::operator-() const {
  Vec tmp(-data[0], -data[1], -data[2]);
  return tmp;
}

Vec Vec::operator/(const double rhs) const {
  if (fabs(rhs) < K_EPSILON) {
    std::ostringstream strs;
    strs << "divisor is close to 0" << std::endl;
    LOG_ERROR(strs);
    return *this;
  }
  Vec tmp;
  tmp.set_x(data[0] / rhs);
  tmp.set_y(data[1] / rhs);
  tmp.set_z(data[2] / rhs);
  return tmp;
}

//! cross product
Vec Vec::operator*(const Vec& rhs) const {
  // Complexity : 6M+3A
  Vec tmp;
  tmp.set_x(data[1] * rhs.z() - data[2] * rhs.y());
  tmp.set_y(data[2] * rhs.x() - data[0] * rhs.z());
  tmp.set_z(data[0] * rhs.y() - data[1] * rhs.x());
  return tmp;
}

double Vec::dot(const Vec& rhs) const {
  return data[0] * rhs.x() + data[1] * rhs.y() + data[2] * rhs.z();
}

bool Vec::operator==(const Vec& rhs) const {
  if (fabs(this->x() - rhs.x()) < K_EPSILON &&
      fabs(this->y() - rhs.y()) < K_EPSILON &&
      fabs(this->z() - rhs.z()) < K_EPSILON) {
    return true;
  } else {
    return false;
  }
}

void Vec::SetToZero() {
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
}

Vec Vec::Zero() { return Vec(0, 0, 0); }

double Vec::Normalize(double eps) {
  double len = this->Norm();
  if (len > eps) {
    *this = (*this) / len;
  } else {
    std::ostringstream strs;
    strs << "Vec has norm close to 0, so can not normialze " << std::endl;
    LOG_ERROR(strs);
    *this = Vec(1, 0, 0);
  }
  return len;
}

Vec Vec::NormalizeVec(double eps) const {
  double len = this->Norm();
  if (len > eps) {
    return (*this) / len;
  } else {
    std::ostringstream strs;
    strs << "Vec has norm close to 0, so can not normialze " << std::endl;
    LOG_ERROR(strs);
    return Vec(1, 0, 0);
  }
}

double Vec::Norm() const {
  return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
}

double Vec::sqNorm() const {
  return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
}

Vec Vec::FromEigenVec(const Eigen::Vector3d& v) {
  return Vec(v(0), v(1), v(2));
}

Eigen::Vector3d Vec::ToEigenVec() {
  return Eigen::Vector3d(data[0], data[1], data[2]);
}

bool Vec::ToHat(Eigen::Matrix3d* hatv) {
  if (!hatv) {
    std::ostringstream strs;
    strs << "input hatv parameter is null in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  (*hatv)(0, 0) = 0;
  (*hatv)(0, 1) = -data[2];
  (*hatv)(0, 2) = data[1];
  (*hatv)(1, 0) = data[2];
  (*hatv)(1, 1) = 0;
  (*hatv)(1, 2) = -data[0];
  (*hatv)(2, 0) = -data[1];
  (*hatv)(2, 1) = data[0];
  (*hatv)(2, 2) = 0;
  return true;
}

bool StdVec2EigenVec(const std::vector<double>& qin, Eigen::VectorXd* qout) {
  std::ostringstream strs;
  if (!qout) {
    strs.str("");
    strs << "output pointer qout is null in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t i_size = qin.size();
  qout->resize(i_size);
  for (size_t i = 0; i < i_size; i++) {
    (*qout)(i) = qin[i];
  }
  return true;
}
bool EigenVec2StdVec(const Eigen::VectorXd& qin, std::vector<double>* qout) {
  std::ostringstream strs;
  if (!qout) {
    strs.str("");
    strs << "output pointer qout is null in function " << __FUNCTION__
         << ", at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  size_t i_size = qin.rows();
  qout->resize(i_size);
  for (size_t i = 0; i < i_size; i++) {
    qout->at(i) = qin(i);
  }
  return true;
}

}  // namespace kinematics_lib