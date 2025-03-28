
#include "robnux_trajectory/base_profile.hpp"
namespace kinematics_lib {
bool BaseProfile::syncDuration(const double duration) {
  if (initialized_ && duration >= duration_ && duration_ >= MIN_TRAJ_DURATION) {
    scale_ = duration / duration_;
    return true;
  } else if (duration_ < MIN_TRAJ_DURATION) {
    std::ostringstream ss;
    ss << " syncDuration = " << true << std::endl;
    LOG_INFO(ss);
    return true;
  }
  return false;
}

}  // namespace kinematics_lib