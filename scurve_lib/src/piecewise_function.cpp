#include "scurve_lib/piecewise_function.hpp"

namespace kinematics_lib {
PolyNomials::PolyNomials() : max_degree_(0) {
  coef_.resize(1, 0);  // constant 0 function  (default)
}

PolyNomials::PolyNomials(const int max_degree) : max_degree_(max_degree) {
  coef_.resize(max_degree + 1, 0);  // all coef is initialized to 0
}

PolyNomials::PolyNomials(const std::vector<double> coef) : coef_(coef) {
  max_degree_ = coef.size() - 1;  // a0 +a1 t +a2t^2, 3 coef, but max degree = 2
}

PolyNomials PolyNomials::Integrate(const double inte_constants) {
  // we need to interate this polynomial to give rise a new polynomial
  // c0 + int{a_0 +a_1t +a_2 t^2 + a_3t^3} = c0 + a_0t+ 1/2 a_1 t^2 + 1/3 a_2
  // t^3 + 1/4a_3 t^4 ...
  std::vector<double> new_p(max_degree_ + 2);
  // PolyNormials new_p(max_degree_ + 1);
  new_p[0] = inte_constants;
  for (size_t i = 1; i <= max_degree_ + 1; i++) {
    new_p[i] = coef_[i - 1] / i;
  }
  return PolyNomials(new_p);
}

double PolyNomials::evaluate(const double in_value) const {
  double sumVal = 0;
  for (int i = max_degree_; i >= 0; i--) {
    sumVal *= in_value;
    sumVal += coef_[i];
  }
  return sumVal;
}

bool PolyNomials::TimeAtDist(const double in_value, double& time) const {
  if (max_degree_ > 3 || max_degree_ < 1) {
    time = 0;  // return 0
    return false;
  }
  std::vector<double> roots;
  if (max_degree_ == 1) {  // linear equation
    time = (in_value - coef_[0]) / coef_[1];
  } else if (max_degree_ == 2) {  // quadratic equation
    if (!Compute_Roots_Utility::QuadEqRealRoots(coef_[2], coef_[1],
                                                coef_[0] - in_value, roots)) {
      time = 0;
      return false;
    }
  } else if (!Compute_Roots_Utility::CubicEqRealRoots(
                 coef_[3], coef_[2], coef_[1], coef_[0] - in_value, roots)) {
    time = 0;
    return false;
  }
  size_t n_rts = roots.size();
  if (n_rts == 1) {
    time = roots[0];
  } else if (n_rts == 2) {
    time = Compute_Roots_Utility::min_positive_root2(roots[0], roots[1]);
  } else if (n_rts == 3) {
    time =
        Compute_Roots_Utility::min_positive_root3(roots[0], roots[1], roots[2]);
  }
  return true;
}

std::vector<double> PolyNomials::GetCoef() const { return coef_; }

int PolyNomials::GetMaxDegree() const { return max_degree_; }

PolyNomials& PolyNomials::operator=(const PolyNomials& arg) {
  max_degree_ = arg.GetMaxDegree();
  coef_ = arg.GetCoef();
  return *this;
}

PiecewiseFunction& PiecewiseFunction::operator=(const PiecewiseFunction& arg) {
  boundaries_ = arg.GetBoundary();
  functions_ = arg.GetFunction();
  lowest_bound_ = boundaries_.front();
  largest_bound_ = boundaries_.back();
  duration_ = largest_bound_ - lowest_bound_;
  dist_ = evaluate(largest_bound_) - evaluate(lowest_bound_);
  initialized_ = true;
  return *this;
}

PiecewiseFunction::PiecewiseFunction(const std::vector<double>& boundaries,
                                     const std::vector<PolyNomials>& functions)
    : boundaries_(boundaries),
      functions_(functions),
      duration_(0),
      dist_(0),
      lowest_bound_(0),
      largest_bound_(0),
      initialized_(false) {
  std::ostringstream strs;
  if (boundaries_.size() != functions_.size() + 1 && boundaries_.size() < 2) {
    strs << __FUNCTION__ << ":" << __LINE__
         << ": boundary size=" << boundaries_.size()
         << ", functions size=" << functions_.size() << std::endl;
    LOG_ERROR(strs);
  }
  lowest_bound_ = boundaries.front();
  largest_bound_ = boundaries.back();
  duration_ = largest_bound_ - lowest_bound_;
  dist_ = evaluate(largest_bound_) - evaluate(lowest_bound_);
  sign_ = dist_ >= 0.0 ? 1.0 : -1.0;
  dist_ *= sign_;
  initialized_ = true;
}

double PiecewiseFunction::evaluate(const double value) const {
  int intv_index;
  // const double max_bound = boundaries_.back();
  // const double min_bound = bounaries_.front();
  double in_value = std::min(value, largest_bound_);
  in_value = std::max(in_value, lowest_bound_);
  for (int i = boundaries_.size() - 1; i >= 1; i--) {
    if (in_value <= boundaries_[i] && in_value > boundaries_[i - 1]) {
      return functions_[i - 1].evaluate(in_value - boundaries_[i - 1]);
    }
  }
  return functions_[0].evaluate(in_value);
}

bool PiecewiseFunction::duration(double& duration_time) {
  if (!initialized_) {
    return false;
  }
  duration_time = duration_;
  return true;
}

bool PiecewiseFunction::lowest_t_bound(double& low_bound) {
  if (!initialized_) {
    return false;
  }
  low_bound = lowest_bound_;
  return true;
}

bool PiecewiseFunction::largest_t_bound(double& up_bound) {
  if (!initialized_) {
    return false;
  }
  up_bound = largest_bound_;
  return true;
}

bool PiecewiseFunction::TimeAtDist(const double in_value, double& time) const {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__ << " object is not initialized"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (in_value > dist_) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__ << " in_value is too large"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (in_value < 0) {
    time = 0;
  } else if (in_value > dist_) {
    time = duration_;
  } else {
    double min_s = evaluate(lowest_bound_);
    for (size_t i = 1; i < boundaries_.size(); i++) {
      double cur_s = evaluate(boundaries_[i]);
      if (in_value <= fabs(cur_s - min_s)) {
        double tmp_time;
        bool ret =
            functions_[i - 1].TimeAtDist(sign_ * in_value + min_s, tmp_time);
        if (ret) {
          time = tmp_time + boundaries_[i - 1];
        }
        return ret;
      } else {
        continue;
      }
    }
  }
  strs.str("");
  strs << __FUNCTION__ << ":" << __LINE__
       << " shouldn't run to this line, serious error" << std::endl;
  LOG_ERROR(strs);
  return false;
}

bool PiecewiseFunction::extend(const std::vector<double>& boundaries,
                               const std::vector<PolyNomials>& functions) {
  std::ostringstream strs;
  if (!initialized_) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__ << " object is not initialized"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (boundaries.empty() || functions.empty()) {
    strs << __FUNCTION__ << ":" << __LINE__
         << " input has empty vectors: boundary size=" << boundaries_.size()
         << ", functions size=" << functions_.size() << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (fabs(boundaries[0]) > std::numeric_limits<double>::epsilon()) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__
         << "input boundaries[0] is not 0, but= " << boundaries[0] << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (boundaries.size() != functions.size() + 1) {
    strs << __FUNCTION__ << ":" << __LINE__
         << ": input boundary size=" << boundaries.size()
         << ", input functions size=" << functions.size() << std::endl;
    LOG_ERROR(strs);
  }
  boundaries_.insert(boundaries_.end(), boundaries.begin(), boundaries.end());
  functions_.insert(functions_.end(), functions.begin(), functions.end());
  lowest_bound_ = boundaries.front();
  largest_bound_ = boundaries.back();
  duration_ = largest_bound_ - lowest_bound_;
  dist_ = evaluate(largest_bound_) - evaluate(lowest_bound_);
  return true;
}

bool PiecewiseFunction::sample(const int npoints, std::vector<double>& values) {
  std::ostringstream strs;
  if (boundaries_.empty() || functions_.empty()) {
    strs << __FUNCTION__ << ":" << __LINE__ << ", has empty bounds or functions"
         << std::endl;
    LOG_ERROR(strs);
    return false;
  }
  if (npoints <= 2) {
    strs.str("");
    strs << __FUNCTION__ << ":" << __LINE__ << ", npoints=" << npoints
         << " is too small" << std::endl;
    LOG_ERROR(strs);
    return false;
  }

  double dist = boundaries_.back() - boundaries_.front();
  double step_dist = dist / (npoints - 1);
  values.clear();
  double in_val = boundaries_.front();
  values.push_back(evaluate(in_val));
  for (int i = 1; i <= npoints - 1; i++) {
    in_val += step_dist;
    values.push_back(evaluate(in_val));
  }
  return true;
}

PiecewiseFunction PiecewiseFunction::Integrate(const double inte_constants) {
  std::ostringstream strs;
  if (boundaries_.empty() || functions_.empty()) {
    strs << __FUNCTION__ << ":" << __LINE__ << ", has empty bounds or functions"
         << std::endl;
    LOG_ERROR(strs);
    return *this;
  }

  std::vector<PolyNomials> inte_functions;
  int num_funcs = functions_.size();
  double init_const = inte_constants;
  for (int i = 0; i < num_funcs; i++) {
    PolyNomials pn = functions_[i].Integrate(init_const);
    inte_functions.push_back(pn);
    init_const = pn.evaluate(boundaries_[i + 1] - boundaries_[i]);
  }
  return PiecewiseFunction(boundaries_, inte_functions);
}

}  // namespace kinematics_lib