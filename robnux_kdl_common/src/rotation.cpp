#include "robnux_kdl_common/rotation.hpp"
namespace kinematics_lib {

EulerAngle::EulerAngle() : Vec(), tp_(eZYX) {}

EulerAngle::EulerAngle(const double A, const double B, const double C,
                       const EulerType& eType)
    : Vec(A, B, C), tp_(eType) {}

bool EulerAngle::GetQuaternion(Quaternion* q) const {
  if (!q) {
    std::ostringstream strs;
    strs << "input pointer to " << __FUNCTION__ << " is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (tp_ == eZYX) {
    q->SetEulerZYX(data[0], data[1], data[2]);
  } else {
    Rotation r;
    r.SetEulerZYZ(data[0], data[1], data[2]);
    return r.GetQuaternion(q);
  }
  return true;
}

bool EulerAngle::GetRotation(Rotation* r) const {
  if (!r) {
    std::ostringstream strs;
    strs << "input pointer to " << __FUNCTION__ << " is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (tp_ == eZYX) {
    r->SetEulerZYX(data[0], data[1], data[2]);
  } else {
    r->SetEulerZYZ(data[0], data[1], data[2]);
  }
  return true;
}

Quaternion::Quaternion() {
  // rotation about z axis by angle 0
  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
}

Quaternion::Quaternion(const Quaternion& qin) {
  q[1] = qin.x();
  q[2] = qin.y();
  q[3] = qin.z();
  q[0] = qin.w();
}

Quaternion::Quaternion(const double w, const double x, const double y,
                       const double z) {
  q[1] = x;
  q[2] = y;
  q[3] = z;
  q[0] = w;
}

Quaternion::Quaternion(const Vec& axis, const double angle) {
  Vec axis1 = axis.NormalizeVec(K_EPSILON);
  double chalf = cos(angle / 2.0);
  double shalf = sin(angle / 2.0);
  q[1] = shalf * axis1.x();
  q[2] = shalf * axis1.y();
  q[3] = shalf * axis1.z();
  q[0] = chalf;
}

void Quaternion::setRotation(const Vec& axis, const double angle) {
  Vec axis1 = axis.NormalizeVec(K_EPSILON);
  double chalf = cos(angle / 2.0);
  double shalf = sin(angle / 2.0);
  q[1] = shalf * axis1.x();
  q[2] = shalf * axis1.y();
  q[3] = shalf * axis1.z();
  q[0] = chalf;
}

void Quaternion::SetEulerZYX(const double yaw, const double pitch,
                             const double roll) {
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

double Quaternion::shortestAngle(const Quaternion& q) const {
  Quaternion tmp = this->inverse() * q;
  Vec axis;
  return tmp.getAngleShortestPath(&axis);
}

// return [0, 2pi]
double Quaternion::getAngle() const {
  if (q[0] >= 1 - K_EPSILON || q[0] <= -1 + K_EPSILON) {
    return 0;
  }
  return 2.0 * acos(q[0]);
}

// return [0, pi]
double Quaternion::getAngleShortestPath(Vec* axis) const {
  if (!axis) {
    std::ostringstream strs;
    strs << "input parameter is null at " << __FUNCTION__ << std::endl;
    LOG_ERROR(strs);
    throw KinematicsException("input null pointer", -2);
  }
  double angle = this->getAngle();
  Vec a(q[1], q[2], q[3]);
  if (angle > M_PI) {
    angle = 2 * M_PI - angle;
    a = -a;  // reverse direction
  }
  double sa = sin(angle / 2.0);
  if (fabs(sa) > K_EPSILON) {
    a = a / sa;
    *axis = a.NormalizeVec(K_EPSILON);
  } else {
    *axis = Vec(0, 0, 1);
  }
  return angle;
}

bool Quaternion::GetEulerZYX(double* yaw, double* pitch, double* roll) const {
  if (!yaw || !pitch || !roll) {
    std::ostringstream strs;
    strs << "input pointers to Quaterion::getEulerZYX() is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (fabs(this->length() - 1) > K_EPSILON) {
    std::ostringstream strs;
    strs << "Quaternion is not normalized, can not compute"
         << "Euler angles, please normalize first" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  if (sinr_cosp * sinr_cosp + cosr_cosp * cosr_cosp < MIN_QUAT_SING_VALUE) {
    if (fabs(q[0]) > MIN_QUAT_SING_VALUE) {
      *roll = 0;  //-M_PI;
    } else {
      *roll = -M_PI;
    }
  } else {
    *roll = std::atan2(sinr_cosp, cosr_cosp);
  }

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::fabs(sinp) >= 1) {
    // use 90 degrees if out of range
    *pitch = sinp > 0 ? M_PI / 2.0 : -M_PI / 2.0;
  } else {
    *pitch = std::asin(sinp);
  }

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  if (siny_cosp * siny_cosp + cosy_cosp * cosy_cosp < MIN_QUAT_SING_VALUE) {
    if (fabs(q[0]) < MIN_QUAT_SING_VALUE) {
      *yaw = 0;
    } else {
      *yaw = -M_PI;
    }
  } else {
    *yaw = std::atan2(siny_cosp, cosy_cosp);
  }
  return true;
}
/*
Vec Quaternion::getAxis() const {
    double angle = this->getAngle(); //this->getAngleShortestPath();
    Vec a(q[1], q[2], q[3]);
    double sa = sin(angle / 2.0);
    if (fabs(sa) > K_EPSILON) {
        a = a / sa;
        return a.NormalizeVec(K_EPSILON);
    } else {
        return Vec(0, 0, 1);
    }
}
 */
double Quaternion::getRealPart() const { return q[0]; }

Vec Quaternion::getVirtualPart() const {
  Vec a(q[1], q[2], q[3]);
  return a;
}

Quaternion Quaternion::inverse() const {
  Quaternion tmp = this->conjugate();
  double len = this->length();
  return tmp / (len * len);
}

Quaternion Quaternion::conjugate() const {
  Quaternion outq(q[0], -q[1], -q[2], -q[3]);
  return outq;
}

Quaternion Quaternion::normalized() const {
  Quaternion tmp = *this;
  tmp.normalize();
  return tmp;
}

Quaternion Quaternion::FromEigenVec(const Eigen::Vector4d& q_) {
  return Quaternion(q_(0), q_(1), q_(2), q_(3));
}

void Quaternion::normalize() {
  double len = this->length();
  if (len > K_EPSILON) {
    q[0] /= len;
    q[1] /= len;
    q[2] /= len;
    q[3] /= len;
  } else {
    // if the quaternion is very close to 0, then just return the quaternion
    // that rotates about z axis by an angle 0
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
}

double Quaternion::length() const {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

double Quaternion::sqNorm() const {
  return q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
}

double Quaternion::x() const { return q[1]; }

double Quaternion::y() const { return q[2]; }

double Quaternion::z() const { return q[3]; }

double Quaternion::w() const { return q[0]; }

void Quaternion::setX(const double x) { q[1] = x; }
void Quaternion::setY(const double y) { q[2] = y; }

void Quaternion::setZ(const double z) { q[3] = z; }

void Quaternion::setW(const double w) { q[0] = w; }

void Quaternion::setRealPart(const double w) { q[0] = w; }

void Quaternion::setVirtualPart(const Vec& vec) {
  q[1] = vec.x();
  q[2] = vec.y();
  q[3] = vec.z();
}

Quaternion& Quaternion::operator=(const Quaternion& q_) {
  q[0] = q_.w();
  q[1] = q_.x();
  q[2] = q_.y();
  q[3] = q_.z();
  return *this;
}

Quaternion Quaternion::operator*(const double s) const {
  Quaternion tmp(this->w() * s, this->x() * s, this->y() * s, this->z() * s);
  return tmp;
}

Quaternion& Quaternion::operator*=(const double s) {
  q[0] *= s;
  q[1] *= s;
  q[2] *= s;
  q[3] *= s;
  return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& qin) {
  double r1 = q[0];
  double r2 = qin.w();
  Vec v1(q[1], q[2], q[3]);
  Vec v2 = qin.getVirtualPart();
  Vec v = r1 * v2 + r2 * v1 + v1 * v2;
  double r = r1 * r2 - v1.dot(v2);
  q[0] = r;
  q[1] = v.x();
  q[2] = v.y();
  q[3] = v.z();
  return *this;
}

Quaternion operator*(const Quaternion& lq, const Quaternion& rq) {
  Quaternion q = lq;
  q *= rq;
  return q;
}

Quaternion operator*(const double coef, const Quaternion& lq) {
  Quaternion q;
  q.setW(coef * lq.w());
  q.setX(coef * lq.x());
  q.setY(coef * lq.y());
  q.setZ(coef * lq.z());
  return q;
}
Quaternion Quaternion::operator+(const Quaternion& rhs) const {
  Quaternion tmp(q[0] + rhs.w(), q[1] + rhs.x(), q[2] + rhs.y(),
                 q[3] + rhs.z());
  return tmp;
}

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
  q[0] += rhs.w();
  q[1] += rhs.x();
  q[2] += rhs.y();
  q[3] += rhs.z();
  return *this;
}

Quaternion Quaternion::operator-(const Quaternion& rhs) const {
  Quaternion tmp(q[0] - rhs.w(), q[1] - rhs.x(), q[2] - rhs.y(),
                 q[3] - rhs.z());
  return tmp;
}

Quaternion Quaternion::operator-() const {
  Quaternion tmp(-q[0], -q[1], -q[2], -q[3]);
  return tmp;
}

Quaternion& Quaternion::operator-=(const Quaternion& rhs) {
  q[0] -= rhs.w();
  q[1] -= rhs.x();
  q[2] -= rhs.y();
  q[3] -= rhs.z();
  return *this;
}

Quaternion Quaternion::operator/(const double s) const {
  if (fabs(s) < K_EPSILON) {
    std::ostringstream strs;
    strs << "divisor is close to 0, return the original quaternion"
         << std::endl;
    LOG_ERROR(strs);
    return *this;
  }
  return (*this) * (1 / s);
}

Quaternion& Quaternion::operator/=(const double s) {
  if (fabs(s) < K_EPSILON) {
    std::ostringstream strs;
    strs << "divisor is close to 0, return the original quaternion"
         << std::endl;
    LOG_ERROR(strs);
    return *this;
  }
  q[0] = q[0] / s;
  q[1] = q[1] / s;
  q[2] = q[2] / s;
  q[3] = q[3] / s;
  return *this;
}

bool Quaternion::operator==(const Quaternion& a) {
  return fabs(q[0] - a.w()) < K_EPSILON &&
         this->getVirtualPart() == a.getVirtualPart();
}

double Quaternion::dot(const Quaternion& rhs) const {
  return q[0] * rhs.w() + q[1] * rhs.x() + q[2] * rhs.y() + q[3] * rhs.z();
}

Quaternion Quaternion::getIdentity() { return Quaternion(1, 0, 0, 0); }

Quaternion Quaternion::getZero() { return Quaternion(0, 0, 0, 0); }

Eigen::Vector4d Quaternion::ToEigenVec() {
  return Eigen::Vector4d(q[0], q[1], q[2], q[3]);
}
Rotation::Rotation() {
  data[0] = 1;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 1;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  data[8] = 1;
}

Rotation::Rotation(const double m1, const double m2, const double m3,
                   const double m4, const double m5, const double m6,
                   const double m7, const double m8, const double m9) {
  data[0] = m1;
  data[1] = m2;
  data[2] = m3;
  data[3] = m4;
  data[4] = m5;
  data[5] = m6;
  data[6] = m7;
  data[7] = m8, data[8] = m9;
}

Rotation::Rotation(const double yaw, const double pitch, const double roll) {
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  data[0] = cp * cy;
  data[1] = -cr * sy + sr * sp * cy;
  data[2] = sr * sy + cr * sp * cy;
  data[3] = cp * sy;
  data[4] = cr * cy + sr * sp * sy;
  data[5] = -sr * cy + cr * sp * sy;
  data[6] = -sp;
  data[7] = sr * cp;
  data[8] = cr * cp;
}

Rotation::Rotation(const Vec& x, const Vec& y, const Vec& z) {
  Vec nx = x.NormalizeVec(K_EPSILON);
  Vec ny = y.NormalizeVec(K_EPSILON);
  Vec nz = z.NormalizeVec(K_EPSILON);
  data[0] = nx.x();
  data[1] = ny.x();
  data[2] = nz.x();
  data[3] = nx.y();
  data[4] = ny.y();
  data[5] = nz.y();
  data[6] = nx.z();
  data[7] = ny.z();
  data[8] = nz.z();
}

Rotation::Rotation(const Vec& z) {
  Vec nz = z.NormalizeVec(K_EPSILON);
  // nx, ny are picked from the column vectors of  \hat{nz}
  Vec x1(0, nz.z(), -nz.y());
  Vec x2(-nz.z(), 0, nz.x());
  Vec x3(nz.y(), -nz.x(), 0);
  Vec nx, ny;
  if (x1.Norm() > K_EPSILON) {
    nx = x1.NormalizeVec(K_EPSILON);
  } else if (x2.Norm() > K_EPSILON) {
    nx = x2.NormalizeVec(K_EPSILON);
  } else {
    nx = x3.NormalizeVec(K_EPSILON);
  }
  ny = nz * nx;
  data[0] = nx.x();
  data[1] = ny.x();
  data[2] = nz.x();
  data[3] = nx.y();
  data[4] = ny.y();
  data[5] = nz.y();
  data[6] = nx.z();
  data[7] = ny.z();
  data[8] = nz.z();
}

Rotation::Rotation(const Vec& axis, const double angle) {
  Vec naxis = axis.NormalizeVec(K_EPSILON);
  double ct = cos(angle);
  double st = sin(angle);
  double vt = 1 - ct;
  double m_vt_0 = vt * naxis.x();
  double m_vt_1 = vt * naxis.y();
  double m_vt_2 = vt * naxis.z();
  double m_st_0 = naxis.x() * st;
  double m_st_1 = naxis.y() * st;
  double m_st_2 = naxis.z() * st;
  double m_vt_0_1 = m_vt_0 * naxis.y();
  double m_vt_0_2 = m_vt_0 * naxis.z();
  double m_vt_1_2 = m_vt_1 * naxis.z();
  data[0] = ct + m_vt_0 * naxis.x();
  data[1] = -m_st_2 + m_vt_0_1;
  data[2] = m_st_1 + m_vt_0_2;
  data[3] = m_st_2 + m_vt_0_1;
  data[4] = ct + m_vt_1 * naxis.y();
  data[5] = -m_st_0 + m_vt_1_2;
  data[6] = -m_st_1 + m_vt_0_2;
  data[7] = m_st_0 + m_vt_1_2;
  data[8] = ct + m_vt_2 * naxis.z();
}

Rotation::Rotation(const Quaternion& q) {
  Quaternion q1 = q.normalized();
  double x = q1.x();
  double y = q1.y();
  double z = q1.z();
  double w = q1.w();
  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;
  double w2 = w * w;
  data[0] = w2 + x2 - y2 - z2;
  data[1] = 2 * x * y - 2 * w * z;
  data[2] = 2 * x * z + 2 * w * y;
  data[3] = 2 * x * y + 2 * w * z;
  data[4] = w2 - x2 + y2 - z2;
  data[5] = 2 * y * z - 2 * w * x;
  data[6] = 2 * x * z - 2 * w * y;
  data[7] = 2 * y * z + 2 * w * x;
  data[8] = w2 - x2 - y2 + z2;
}

Rotation& Rotation::operator=(const Rotation& arg) {
  Vec xaxis = arg.UnitX();
  Vec yaxis = arg.UnitY();
  Vec zaxis = arg.UnitZ();
  data[0] = xaxis.x();
  data[3] = xaxis.y();
  data[6] = xaxis.z();
  data[1] = yaxis.x();
  data[4] = yaxis.y();
  data[7] = yaxis.z();
  data[2] = zaxis.x();
  data[5] = zaxis.y();
  data[8] = zaxis.z();
  return *this;
}

Vec Rotation::operator*(const Vec& v) const {
  return Vec(data[0] * v.x() + data[1] * v.y() + data[2] * v.z(),
             data[3] * v.x() + data[4] * v.y() + data[5] * v.z(),
             data[6] * v.x() + data[7] * v.y() + data[8] * v.z());
}

double Rotation::operator()(int i, int j) const {
  if (0 <= i && i <= 2 && 0 <= j && j <= 2) {
    return data[i * 3 + j];
  } else {
    throw KinematicsException("index out of range", -1);
  }
}

void Rotation::UnitX(const Vec& X) {
  data[0] = X.x();
  data[3] = X.y();
  data[6] = X.z();
}

void Rotation::UnitY(const Vec& Y) {
  data[1] = Y.x();
  data[4] = Y.y();
  data[7] = Y.z();
}

void Rotation::UnitZ(const Vec& Z) {
  data[2] = Z.x();
  data[5] = Z.y();
  data[8] = Z.z();
}

Rotation Rotation::Transpose() const {
  Vec xaxis = this->UnitX();
  Vec yaxis = this->UnitY();
  Vec zaxis = this->UnitZ();

  Vec new_xaxis(xaxis.x(), yaxis.x(), zaxis.x());
  Vec new_yaxis(xaxis.y(), yaxis.y(), zaxis.y());
  Vec new_zaxis(xaxis.z(), yaxis.z(), zaxis.z());

  Rotation rot;
  rot.UnitX(new_xaxis);
  rot.UnitY(new_yaxis);
  rot.UnitZ(new_zaxis);
  return rot;
}

Rotation operator*(const Rotation& lhs, const Rotation& rhs) {
  // Complexity : 27M+27A
  Rotation lr = lhs.Transpose();
  Vec xaxis(lr.UnitX().dot(rhs.UnitX()), lr.UnitY().dot(rhs.UnitX()),
            lr.UnitZ().dot(rhs.UnitX()));
  Vec yaxis(lr.UnitX().dot(rhs.UnitY()), lr.UnitY().dot(rhs.UnitY()),
            lr.UnitZ().dot(rhs.UnitY()));
  Vec zaxis(lr.UnitX().dot(rhs.UnitZ()), lr.UnitY().dot(rhs.UnitZ()),
            lr.UnitZ().dot(rhs.UnitZ()));
  return Rotation(xaxis, yaxis, zaxis);
}

Rotation Rotation::Inverse() const {
  Rotation r(*this);
  Vec xaxis = r.UnitX();
  Vec yaxis = r.UnitY();
  Vec zaxis = r.UnitZ();
  r.UnitX(Vec(xaxis.x(), yaxis.x(), zaxis.x()));
  r.UnitY(Vec(xaxis.y(), yaxis.y(), zaxis.y()));
  r.UnitZ(Vec(xaxis.z(), yaxis.z(), zaxis.z()));
  return r;
}

Rotation Rotation::Identity() {
  Vec xaxis(1, 0, 0);
  Vec yaxis(0, 1, 0);
  Vec zaxis(0, 0, 1);
  Rotation r(xaxis, yaxis, zaxis);
  return r;
}

Rotation Rotation::RotX(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  Vec x(1, 0, 0);
  Vec y(0, cs, sn);
  Vec z(0, -sn, cs);
  return Rotation(x, y, z);
}

Rotation Rotation::RotY(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  Vec x(cs, 0, -sn);
  Vec y(0, 1, 0);
  Vec z(sn, 0, cs);
  return Rotation(x, y, z);
}

Rotation Rotation::RotZ(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  Vec x(cs, sn, 0);
  Vec y(-sn, cs, 0);
  Vec z(0, 0, 1);
  return Rotation(x, y, z);
}

bool Rotation::GetAxisAngle(Vec* axis, double* angle) const {
  double ca = (data[0] + data[4] + data[8] - 1) / 2.0;
  double t = K_EPSILON * K_EPSILON / 2.0;
  if (ca > 1 - t) {
    // undefined choose the Z-axis, and angle 0
    *axis = Vec(0, 0, 1);
    *angle = 0;
  } else if (ca < -1 + t) {
    // The case of angles consisting of multiples of M_PI:
    // two solutions, choose a positive Z-component of the axis
    double x = sqrt((data[0] + 1.0) / 2);
    double y = sqrt((data[4] + 1.0) / 2);
    double z = sqrt((data[8] + 1.0) / 2);
    if (data[2] < 0) x = -x;
    if (data[7] < 0) y = -y;
    // this last line can be necessary when z is 0
    if (x * y * data[1] < 0) x = -x;
    *axis = Vec(x, y, z);
    *angle = M_PI;
  } else {
    double mod_axis;
    double axisx, axisy, axisz;
    axisx = data[7] - data[5];
    axisy = data[2] - data[6];
    axisz = data[3] - data[1];
    mod_axis = sqrt(axisx * axisx + axisy * axisy + axisz * axisz);
    *axis = Vec(axisx / mod_axis, axisy / mod_axis, axisz / mod_axis);
    *angle = atan2(mod_axis / 2, ca);
  }
  return true;
}

bool Rotation::GetQuaternion(Quaternion* q) const {
  if (!q) {
    std::ostringstream strs;
    strs << "input pointer to Rotation::GetQuaternion is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double trace = (*this)(0, 0) + (*this)(1, 1) + (*this)(2, 2) + 1.0;
  double epsilon = 1E-12;
  double x, y, z, w;
  if (trace > epsilon) {
    double s = 0.5 / sqrt(trace);
    w = 0.25 / s;
    x = ((*this)(2, 1) - (*this)(1, 2)) * s;
    y = ((*this)(0, 2) - (*this)(2, 0)) * s;
    z = ((*this)(1, 0) - (*this)(0, 1)) * s;
  } else if ((*this)(0, 0) > (*this)(1, 1) && (*this)(0, 0) > (*this)(2, 2)) {
    double s = 2.0 * sqrt(1.0 + (*this)(0, 0) - (*this)(1, 1) - (*this)(2, 2));
    w = ((*this)(2, 1) - (*this)(1, 2)) / s;
    x = 0.25 * s;
    y = ((*this)(0, 1) + (*this)(1, 0)) / s;
    z = ((*this)(0, 2) + (*this)(2, 0)) / s;
  } else if ((*this)(1, 1) > (*this)(2, 2)) {
    double s = 2.0 * sqrt(1.0 + (*this)(1, 1) - (*this)(0, 0) - (*this)(2, 2));
    w = ((*this)(0, 2) - (*this)(2, 0)) / s;
    x = ((*this)(0, 1) + (*this)(1, 0)) / s;
    y = 0.25 * s;
    z = ((*this)(1, 2) + (*this)(2, 1)) / s;
  } else {
    double s = 2.0 * sqrt(1.0 + (*this)(2, 2) - (*this)(0, 0) - (*this)(1, 1));
    w = ((*this)(1, 0) - (*this)(0, 1)) / s;
    x = ((*this)(0, 2) + (*this)(2, 0)) / s;
    y = ((*this)(1, 2) + (*this)(2, 1)) / s;
    z = 0.25 * s;
  }
  q->setX(x);
  q->setY(y);
  q->setZ(z);
  q->setW(w);
  return true;
}

bool Rotation::GetEulerZYX(double* yaw, double* pitch, double* roll) const {
  if (!yaw || !pitch || !roll) {
    std::ostringstream strs;
    strs << "Input pointers to function " << __FUNCTION__ << " is null"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  *pitch = atan2(-data[6], sqrt(data[0] * data[0] + data[3] * data[3]));
  if (fabs(*pitch) > (M_PI / 2.0 - K_EPSILON)) {
    *yaw = atan2(-data[1], data[4]);
    *roll = 0.0;
  } else {
    *roll = atan2(data[7], data[8]);
    *yaw = atan2(data[3], data[0]);
  }
  return true;
}

//! Get ZYZ Euler angles
bool Rotation::GetEulerZYZ(double* A, double* B, double* C) const {
  if (!A || !B || !C) {
    std::ostringstream strs;
    strs << "Input pointers to function " << __FUNCTION__ << " is null"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  // first assume fabs(sb) > epsilon
  double sb2 = data[2] * data[2] + data[5] * data[5];
  if (sb2 > K_EPSILON) {
    *A = atan2(data[5], data[2]);
    *C = atan2(data[7], -data[6]);
    double cc = cos(*C);
    double sc = sin(*C);
    *B = atan2(data[7] * sc - data[6] * cc, data[8]);
  } else if (data[8] > 1 - K_EPSILON) {  // i.e. CB =  1
    *B = 0;
    *C = 0;
    *A = atan2(data[3], data[4]);
  } else {  //  i.e. CB = -1;
    *B = M_PI;
    *C = atan2(data[3], data[4]);
    *A = 0;
  }
  return true;
}
//! Set ZYX Euler angles
void Rotation::SetEulerZYX(const double yaw, const double pitch,
                           const double roll) {
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  data[0] = cp * cy;
  data[1] = -cr * sy + sr * sp * cy;
  data[2] = sr * sy + cr * sp * cy;
  data[3] = cp * sy;
  data[4] = cr * cy + sr * sp * sy;
  data[5] = -sr * cy + cr * sp * sy;
  data[6] = -sp;
  data[7] = sr * cp;
  data[8] = cr * cp;
}

void Rotation::SetEulerZYZ(const double A, const double B, const double C) {
  double ca = cos(A);
  double sa = sin(A);
  double cb = cos(B);
  double sb = sin(B);
  double cc = cos(C);
  double sc = sin(C);
  data[0] = ca * cb * cc - sa * sc;
  data[1] = -ca * cb * sc - sa * cc;
  data[2] = ca * sb;
  data[3] = sa * cb * cc + ca * sc;
  data[4] = -sa * cb * sc + ca * cc;
  data[5] = sa * sb;
  data[6] = -sb * cc;
  data[7] = sb * sc;
  data[8] = cb;
}

/*
 *  the following 4 functions are static functions, mainly for smoothing
 * the rotational motion
 * between two trajectory segment
 */
bool Rotation::GetEulerVelZYX(const Vec& Euler, const Vec& angularVel,
                              Vec* EulerDot) {
  if (!EulerDot) {
    std::ostringstream strs;
    strs << "The input pointer to " << __FUNCTION__ << " is null " << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double A = Euler.x();  // Euler angle about x axis
  double B = Euler.y();  // Euler angle about y axis
  double C = Euler.z();  // Euler angle about z axis
  double cb, sb, cc, sc;
  cb = cos(B);
  sb = sin(B);
  cc = cos(C);
  sc = sin(C);

  if (cb < K_EPSILON) {
    std::ostringstream strs;
    strs << "ZYX euler representation is in singular in function "
         << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    *EulerDot = Vec();
    return false;
  } else {
    double e1 = (-sc * angularVel.y() - cc * angularVel.x()) / (-cb);
    double e2 = (-cb * cc * angularVel.y() + cb * sc * angularVel.x()) / (-cb);
    double e3 = (-cb * angularVel.z() - sb * sc * angularVel.y() -
                 sb * cc * angularVel.x()) /
                (-cb);
    *EulerDot = Vec(e1, e2, e3);
    /*  new implement similar to old
    double Bdot = cc * angularVel.y() - sc * angularVel.x();
    double Adot = (sc * angularVel.y() + cc * angularVel.x()) / cb;
    double Cdot = angularVel.z() + sb * Adot;
    *EulerDot = Vec(Adot, Bdot, Cdot);
     */
    return true;
  }
}

bool Rotation::GetEulerAccZYX(const Vec& Euler, const Vec& EulerDot,
                              const Vec& angularAcc, Vec* EulerDDot) {
  double A, B, C, Adot, Bdot, Cdot, ABdot, ACdot, BCdot;
  double cc, sc, cb, sb;

  B = Euler.y();
  cb = cos(B);
  if (cb < K_EPSILON) {
    *EulerDDot = Vec();
    std::ostringstream strs;
    strs << "ZYX euler representation is in singular in function "
         << __FUNCTION__ << " at line " << __LINE__ << std::endl;
    LOG_ERROR(strs);
    return false;
  } else {
    C = Euler.z();
    cc = cos(C);
    sc = sin(C);
    cb = cos(B);
    sb = sin(B);

    Adot = EulerDot.x();
    Bdot = EulerDot.y();
    Cdot = EulerDot.z();
    ABdot = Adot * Bdot;
    ACdot = Adot * Cdot;
    BCdot = Bdot * Cdot;

    double e1 = angularAcc.z() - (-cb * ABdot);
    double e2 =
        angularAcc.y() - (-sb * sc * ABdot + cb * cc * ACdot - sc * BCdot);
    double e3 =
        angularAcc.x() - (-cc * BCdot - sb * cc * ABdot - cb * sc * ACdot);

    double f1 = (-sc * e2 - cc * e3) / (-cb);   // Addot
    double f2 = (-cb * cc * e2 + cb * sc * e3)  // Bddot
                / (-cb);
    double f3 = (-cb * e1 - sb * sc * e2  // Cddot
                 - sb * cc * e3) /
                (-cb);

    *EulerDDot = Vec(f1, f2, f3);
    return true;
  }
}

bool Rotation::GetTwistVelZYX(const Vec& Euler, const Vec& EulerDot,
                              Vec* angularVel) {
  double A, B, C, Adot, Bdot, Cdot;
  double sc, cc, sb, cb;

  A = Euler.x();        // Euler angle about x axis
  B = Euler.y();        // Euler angle about y axis
  C = Euler.z();        // Euler angle about z axis
  Adot = EulerDot.x();  // Euler angle dot about x axis
  Bdot = EulerDot.y();  // Euler angle dot about y axis
  Cdot = EulerDot.z();  // Euler angle dot about z axis

  sc = sin(C);
  cc = cos(C);
  sb = sin(B);
  cb = cos(B);
  // ******************* the following could be wrong, because e1 seems to be
  // rotation about z axis, e2 about y axis, e3 about x axis
  double e1 = -sb * Adot + Cdot;
  double e2 = cb * sc * Adot + cc * Bdot;
  double e3 = cb * cc * Adot - sc * Bdot;
  // *angularVel = Vec(e1,e2,e3);   *** old implementation
  *angularVel = Vec(e3, e2, e1);  // angular speed should be [vx, vy, vz]
  return true;
}

bool Rotation::GetTwistAccZYX(const Vec& Euler, const Vec& EulerDot,
                              const Vec& EulerDDot, Vec* angularAcc) {
  double A, B, C, Adot, Bdot, Cdot, ABdot, ACdot, BCdot, Addot, Bddot, Cddot;
  double sc, cc, sb, cb;
  A = Euler.x();  // Euler angle about x axis
  B = Euler.y();  // Euler angle about y axis
  C = Euler.z();  // Euler angle about z axis
  Adot = EulerDot.x();
  Bdot = EulerDot.y();
  Cdot = EulerDot.z();
  ABdot = Adot * Bdot;
  ACdot = Adot * Cdot;
  BCdot = Bdot * Cdot;

  Addot = EulerDDot.x();
  Bddot = EulerDDot.y();
  Cddot = EulerDDot.z();

  sc = sin(C);
  cc = cos(C);
  sb = sin(B);
  cb = cos(B);

  double e1 = -sb * Addot + Cddot;
  double e2 = cb * sc * Addot + cc * Bddot;
  double e3 = cb * cc * Addot - sc * Bddot;
  double f1 = -cb * ABdot + e1;
  double f2 = -sb * sc * ABdot + cb * cc * ACdot - sc * BCdot + e2;
  double f3 = -sb * cc * ABdot - cb * sc * ACdot - cc * BCdot + e3;
  // *angularAcc = Vec(f1,f2,f3);  ** old implementation
  *angularAcc = Vec(f3, f2, f1);
  return true;
}

//! Given Euler angle, angular velocity, compute EulerDot
bool Rotation::GetEulerVelZYZ(const Vec& Euler, const Vec& angularVel,
                              Vec* EulerDot) {
  if (!EulerDot) {
    std::ostringstream strs;
    strs << "The input pointer to Rotation::GetEulerVelZYX is null "
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double A = Euler.x();
  double B = Euler.y();
  double C = Euler.z();
  double sb = sin(B);
  if (fabs(sb) < K_EPSILON) {
    std::ostringstream strs;
    strs << "The ZYZ euer angle representation in function " << __FUNCTION__
         << " is singular" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double cb = cos(B);
  double cc = cos(C);
  double sc = sin(C);
  EulerDot->set_x((-cc * angularVel.x() + sc * angularVel.y()) / sb);
  EulerDot->set_y(sc * angularVel.x() + cc * angularVel.y());
  EulerDot->set_z((cb * cc * angularVel.x() - cb * sc * angularVel.y()) / sb +
                  angularVel.z());
  return true;
}

//! Given Euler angle, dotEuler, and angular Acc, compute EulerDDot
bool Rotation::GetEulerAccZYZ(const Vec& Euler, const Vec& EulerDot,
                              const Vec& angularAcc, Vec* EulerDDot) {
  if (!EulerDDot) {
    std::ostringstream strs;
    strs << "The input pointer to Rotation::GetEulerAccZYX is null "
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double A = Euler.x();
  double B = Euler.y();
  double C = Euler.z();
  double Adot = EulerDot.x();
  double Bdot = EulerDot.y();
  double Cdot = EulerDot.z();
  double ABdot = Adot * Bdot;
  double ACdot = Adot * Cdot;
  double BCdot = Bdot * Cdot;
  double cb = cos(B);
  double sb = sin(B);
  double cc = cos(C);
  double sc = sin(C);

  double b1 = angularAcc.x() + cb * cc * ABdot - sb * sc * ACdot - cc * BCdot;
  double b2 = angularAcc.y() - cb * sc * ABdot - sb * cc * ACdot + sc * BCdot;
  double b3 = angularAcc.z() + sb * ABdot;

  EulerDDot->set_x((-cc * b1 + sc * b2) / sb);
  EulerDDot->set_y(sc * b1 + cc * b2);
  EulerDDot->set_z((cb * cc * b1 - cb * sc * b2) / sb + b3);
  return true;
}

//! Given Euler angle, EulerDot, compute angular Velocity
bool Rotation::GetTwistVelZYZ(const Vec& Euler, const Vec& EulerDot,
                              Vec* angularVel) {
  if (!angularVel) {
    std::ostringstream strs;
    strs << " The input pointer to  " << __FUNCTION__ << "is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double B = Euler.y();
  double C = Euler.z();
  double cb = cos(B);
  double sb = sin(B);
  double cc = cos(C);
  double sc = sin(C);

  angularVel->set_x(-sb * cc * EulerDot.x() + sc * EulerDot.y());
  angularVel->set_y(sb * sc * EulerDot.x() + cc * EulerDot.y());
  angularVel->set_z(cb * EulerDot.x() + EulerDot.z());
  return true;
}

//! Given Euler angle, EulerDot, EulerDDot, compute angular acceleration
bool Rotation::GetTwistAccZYZ(const Vec& Euler, const Vec& EulerDot,
                              const Vec& EulerDDot, Vec* angularAcc) {
  if (!angularAcc) {
    std::ostringstream strs;
    strs << " The input pointer to  " << __FUNCTION__ << "is null" << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  double A = Euler.x();
  double B = Euler.y();
  double C = Euler.z();
  double Adot = EulerDot.x();
  double Bdot = EulerDot.y();
  double Cdot = EulerDot.z();
  double ABdot = Adot * Bdot;
  double ACdot = Adot * Cdot;
  double BCdot = Bdot * Cdot;
  double cb = cos(B);
  double sb = sin(B);
  double cc = cos(C);
  double sc = sin(C);

  double b1 = -cb * cc * ABdot + sb * sc * ACdot + cc * BCdot;
  double b2 = cb * sc * ABdot + sb * cc * ACdot - sc * BCdot;
  double b3 = -sb * ABdot;

  angularAcc->set_x(-sb * cc * EulerDDot.x() + sc * EulerDDot.y() + b1);
  angularAcc->set_y(sb * sc * EulerDDot.x() + cc * EulerDDot.y() + b2);
  angularAcc->set_z(cb * EulerDDot.x() + EulerDDot.z() + b3);
  return true;
}

std::string Rotation::ToString() const {
  double A, B, C;  // Z,Y,X Euler angles
  this->GetEulerZYX(&A, &B, &C);
  Quaternion q;
  this->GetQuaternion(&q);
  std::ostringstream strs;
  strs << "Euler angles (roll, pitch, yaw): roll " << C << " pitch " << B
       << " yaw " << A << " "
       << " or in Quaternion " << q.ToString(false);
  std::string str = strs.str();
  return str;
}

bool Rotation::operator==(const Rotation& a) {
  return this->UnitX() == a.UnitX() && this->UnitY() == a.UnitY() &&
         this->UnitZ() == a.UnitZ();
}

Eigen::Matrix3d Rotation::ToEigenMat() {
  Eigen::Matrix3d mat;
  mat.col(0) = UnitX().ToEigenVec();
  mat.col(1) = UnitY().ToEigenVec();
  mat.col(2) = UnitZ().ToEigenVec();
  return mat;
}

Eigen::Vector3d Rotation::ToEigenVecZYX() {
  double yaw, pitch, roll;
  if (GetEulerZYX(&yaw, &pitch, &roll)) {
    return Eigen::Vector3d(yaw, pitch, roll);
  } else {
    return Eigen::Vector3d(0, 0, 0);
  }
}

Eigen::Vector3d Rotation::ToEigenVecZYZ() {
  double A, B, C;
  if (GetEulerZYZ(&A, &B, &C)) {
    return Eigen::Vector3d(A, B, C);
  } else {
    return Eigen::Vector3d(0, 0, 0);
  }
}

}  // namespace kinematics_lib
