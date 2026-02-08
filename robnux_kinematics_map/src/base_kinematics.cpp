#include "robnux_kinematics_map/base_kinematics.hpp"
namespace kinematics_lib {
BaseKinematicMap::BaseKinematicMap(const size_t active_dof, const size_t dof)
    : A_DoF_(active_dof),
      DoF_(dof),
      initialized_(false),
      useCalibrated_(false),
      isDHCalibrated_(false) {}

BaseKinematicMap::~BaseKinematicMap() {}

std::vector<std::string>& BaseKinematicMap::GetJntNames() {
  return jnt_names_;
}

bool BaseKinematicMap::isInitialized() const { 
    return initialized_;
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
    const EigenDRef<Eigen::VectorXd>& baseoff) {
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

void BaseKinematicMap::GetDefaultBaseOff(
                EigenDRef<Eigen::VectorXd>& baseoff) {
    std::ostringstream strs;
    
    baseoff = defaultBaseOff_.ToEigenVecQuat();
    strs.str("");
    strs << GetName() << ":"
         << " getDefaultBaseOff =" << baseoff.transpose() << std::endl;
    LOG_INFO(strs);

  }

}