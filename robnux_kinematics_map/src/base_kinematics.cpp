#include "robnux_kinematics_map/base_kinematics.hpp"
namespace kinematics_lib {
BaseKinematicMap::BaseKinematicMap(const size_t active_dof, const size_t dof)
    : A_DoF_(active_dof),
      DoF_(dof),
      initialized_(false),
      useCalibrated_(false),
      isDHCalibrated_(false) {
    pitch_.resize(dof);
    for (size_t i=0; i < dof; i++) {
        pitch_(i) = 1.0;  // initialized to 1.0 (default)
    }
    backlash_ = Eigen::VectorXd::Zero(dof);
}

BaseKinematicMap::~BaseKinematicMap() {}

std::vector<std::string>& BaseKinematicMap::GetJntNames() {
  return jnt_names_;
}

bool BaseKinematicMap::isInitialized() const { 
    return initialized_;
}

bool BaseKinematicMap::isDHCalibrated() const {
    return isDHCalibrated_;
}

size_t BaseKinematicMap::GetDoF() const { 
    return DoF_; 
}

size_t BaseKinematicMap::GetActDoF() const {
    return A_DoF_;
}

void BaseKinematicMap::SetUseCalibrated(const bool useCalibrated) {
    useCalibrated_ = useCalibrated;
}

void BaseKinematicMap::SetDefaultBaseOff(
    const Eigen::VectorXd& baseoff) {
    std::ostringstream strs;
    if (baseoff.size() != 7) {
      strs.str("");
      strs << GetName() << ":"
           << "input of SetDefaultBaseOff = " << baseoff
           << ", has wrong dimension" << std::endl;
      LOG_ERROR(strs);
      return;
    }

    Vec t(baseoff.segment(0, 3));
    Quaternion q(baseoff(3), baseoff(4), baseoff(5), baseoff(6));
    defaultBaseOff_.setTranslation(t);
    defaultBaseOff_.setQuaternion(q);
    strs.str("");
    strs << GetName() << ":"
         << "SetDefaultBaseOff = " << defaultBaseOff_.ToString(true)
         << ", quat=" << defaultBaseOff_.ToString(false)
         << ", in edgen=" << baseoff.transpose() << std::endl;
    LOG_INFO(strs);
}

void BaseKinematicMap::GetDefaultBaseOff(Eigen::VectorXd& baseoff) const {
    std::ostringstream strs;
    
    baseoff = defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getDefaultBaseOff =" << baseoff.transpose() << std::endl;
    LOG_INFO(strs);

}


void BaseKinematicMap::GetPitchAndBacklash(Eigen::VectorXd& pitch,
                           Eigen::VectorXd& backlash) const {
  pitch = pitch_;
  backlash = backlash_;
}

void BaseKinematicMap::SetPitchAndBacklash(const Eigen::VectorXd& pitch,
                           const Eigen::VectorXd& backlash) {
  if (pitch.size() != DoF_) {
    LOG_ERROR("pitch size does not match DoF");
    return;
  }
  if (backlash.size() != DoF_) {
    LOG_ERROR("backlash size does not match DoF");
    return;
  }
  pitch_ = pitch;
  backlash_ = backlash;
}

void BaseKinematicMap::ResetCalibration() {
    isDHCalibrated_ = false;
}

}
