#include "kdl_common/twistWrench.hpp"

namespace kinematics_lib {

Wrench& Wrench::operator-=(const Wrench& arg) {
  this->force -= arg.getForce();
  this->torque -= arg.getTorque();
  return *this;
}

Wrench& Wrench::operator+=(const Wrench& arg) {
  this->force += arg.getForce();
  this->torque += arg.getTorque();
  return *this;
}

double& Wrench::operator()(int i) {
  if (i >= 0 && i <= 2) {
    return this->force[i];
  } else if (i <= 5) {
    return this->torque[i - 3];
  } else {
    std::ostringstream strs;
    strs << "index is out of range" << std::endl;
    LOG_ERROR(strs);
    // std::exception exc("index is out of range");
    throw KinematicsException("index is out of range", -1);
  }
}

//! index-based access to components, first force(0..2), then torque(3..5)
//! for use with a const Wrench
double Wrench::operator()(int i) const {
  if (i >= 0 && i <= 2) {
    return this->force[i];
  } else if (i <= 5) {
    return this->torque[i - 3];
  } else {
    std::ostringstream strs;
    strs << "index is out of range" << std::endl;
    LOG_ERROR(strs);
    // std::exception exc("index is out of range");
    // throw exception
    throw KinematicsException("index is out of range", -1);
  }
}

Wrench operator*(const Wrench& lhs, double rhs) {
  return Wrench(lhs.getForce() * rhs, lhs.getTorque() * rhs);
}

Wrench operator*(double lhs, const Wrench& rhs) {
  return Wrench(lhs * rhs.getForce(), lhs * rhs.getTorque());
}

Wrench operator/(const Wrench& lhs, double rhs) {
  return Wrench(lhs.getForce() / rhs, lhs.getTorque() / rhs);
}

Wrench operator+(const Wrench& lhs, const Wrench& rhs) {
  return Wrench(lhs.getForce() + rhs.getForce(),
                lhs.getTorque() + rhs.getTorque());
}

Wrench operator-(const Wrench& lhs, const Wrench& rhs) {
  return Wrench(lhs.getForce() - rhs.getForce(),
                lhs.getTorque() - rhs.getTorque());
}

Wrench operator-(const Wrench& arg) {
  return Wrench(-arg.getForce(), -arg.getTorque());
}

Vec Wrench::getForce() const { return this->force; }
Vec Wrench::getTorque() const { return this->torque; }

void Wrench::setForce(const Vec& f) { this->force = f; }
void Wrench::setTorque(const Vec& t) { this->torque = t; }

Wrench Wrench::Zero() { return Wrench(Vec(0, 0, 0), Vec(0, 0, 0)); }

Wrench Wrench::RefPoint(const Vec& v_base_AB) const {
  return Wrench(this->force, this->torque + this->force * v_base_AB);
}

bool operator==(const Wrench& a, const Wrench& b) {
  return (a.getForce() == b.getForce() && a.getTorque() == b.getTorque());
}

Twist& Twist::operator-=(const Twist& arg) {
  this->vel -= arg.getLinearVel();
  this->rot -= arg.getAngularVel();
  return *this;
}

Twist& Twist::operator+=(const Twist& arg) {
  this->vel += arg.getLinearVel();
  this->rot += arg.getAngularVel();
  return *this;
}

double& Twist::operator()(int i) {
  if (i >= 0 && i <= 2) {
    return this->vel[i];
  } else if (i <= 5) {
    return this->rot[i - 3];
  } else {
    std::ostringstream strs;
    strs << "index is out of range" << std::endl;
    LOG_ERROR(strs);
    throw KinematicsException("index out of range", -1);
  }
}

double Twist::operator()(int i) const {
  if (i >= 0 && i <= 2) {
    return this->vel[i];
  } else if (i <= 5) {
    return this->rot[i - 3];
  } else {
    std::ostringstream strs;
    strs << "index is out of range" << std::endl;
    LOG_ERROR(strs);
    throw KinematicsException("index out of range", -1);
  }
}

Twist operator*(const Twist& lhs, double rhs) {
  return Twist(lhs.getLinearVel() * rhs, lhs.getAngularVel() * rhs);
}

Twist operator*(double lhs, const Twist& rhs) {
  return Twist(lhs * rhs.getLinearVel(), lhs * rhs.getAngularVel());
}

Twist operator/(const Twist& lhs, double rhs) {
  return Twist(lhs.getLinearVel() / rhs, lhs.getAngularVel() / rhs);
}

Twist operator+(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.getLinearVel() + rhs.getLinearVel(),
               lhs.getAngularVel() + rhs.getAngularVel());
}

Twist operator-(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.getLinearVel() - rhs.getLinearVel(),
               lhs.getAngularVel() - rhs.getAngularVel());
}

Twist operator-(const Twist& arg) {
  return Twist(-arg.getLinearVel(), -arg.getAngularVel());
}

double dot(const Twist& lhs, const Wrench& rhs) {
  return lhs.getLinearVel().dot(rhs.getForce()) +
         lhs.getAngularVel().dot(rhs.getTorque());
}

double dot(const Wrench& rhs, const Twist& lhs) {
  return lhs.getLinearVel().dot(rhs.getForce()) +
         lhs.getAngularVel().dot(rhs.getTorque());
}

Twist operator*(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.getAngularVel() * rhs.getLinearVel() +
                   lhs.getLinearVel() * rhs.getAngularVel(),
               lhs.getAngularVel() * rhs.getAngularVel());
}

Wrench operator*(const Twist& lhs, const Wrench& rhs) {
  return Wrench(lhs.getAngularVel() * rhs.getForce(),
                lhs.getAngularVel() * rhs.getTorque() +
                    lhs.getLinearVel() * rhs.getTorque());
}

Twist Twist::Zero() { return Twist(Vec(0, 0, 0), Vec(0, 0, 0)); }

Vec Twist::getLinearVel() const { return this->vel; }

Vec Twist::getAngularVel() const { return this->rot; }

void Twist::setLinearVel(const Vec& v) { this->vel = v; }
void Twist::setAngularVel(const Vec& w) { this->rot = w; }

Twist Twist::RefPoint(const Vec& v_base_AB) const {
  return Twist(this->vel + this->rot * v_base_AB, this->rot);
}

bool operator==(const Twist& a, const Twist& b) {
  return a.getLinearVel() == b.getLinearVel() &&
         a.getAngularVel() == b.getAngularVel();
}

}  // namespace kinematics_lib
