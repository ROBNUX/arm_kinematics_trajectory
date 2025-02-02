#include <math.h>
//#include <ros/ros.h>
#include <signal.h>
#include "intp/intp.hpp"

using namespace kinematics_lib;
static const double RAD2DEG=57.2957805;
//void sigintHandler(int sig) {
  //  ros::shutdown();
//}

int main(int argc, char** argv) {
  // ros::init(argc, argv, "test_quattro_rviz");
          
  // override default sigint handler
  //signal(SIGINT, sigintHandler);
  int x;
  std::cout<< " input any integer"<<std::endl;
  std::cin >> x;
  
  Eigen::VectorXd jnts(4, 0);
  jnts(0) = 0; //0.00909016;
  jnts(1) = 0; //0.00909045;
  jnts(2) = 0; //1.09954;
  jnts(3) = 0; //1.09954;
  Eigen::VectorXd parameters_;
  parameters_.resize(8);
  parameters_(0) = 0.15; // base is a square of side-length 0.3
  parameters_(1) = -M_PI/2.0;
  parameters_(2) = 0.2;
  parameters_(3) = 0;
  parameters_(4) = 0.51;
  parameters_(5) = 0.05;
  parameters_(6) = 0;
  parameters_(7) = 0.09 * sqrt(2);
  Eigen::VectorXd v(7); 
  v << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  
  ProfileData pf(1.0, 2.0, 4.0, 1.0, 2.0, 4.0);
  CreateRobot rob("quattro", parameters_, v, v, pf);
  if (!rob.SetJntFeedback(jnts)) {
    std::cout << "robot set joint feedback failure" << std::endl;
    return 0;
  }
  
  // start motions
  LocData loc1(0, 0, -0.4, 0, 0, 0, 1, 0);
  FrameData fd(1, 1, ID_WORLD); // default base and tool fram id, IPO mode = world
  if (!rob.LIN(loc1, fd, 10)) {
    std::cout << "robot add Linear command failure" << std::endl;
    return 0;
  }
  
  loc1.x_ = 0.2;
  loc1.y_ = 0.2;
  if (!rob.LIN(loc1, fd, 10)) {
    std::cout << "robot add Linear command failure" << std::endl;
    return 0;
  }
  if (!rob.StartMotion()) {
    std::cout << "robot start motion fails" << std::endl;
    return 0;
  }
  loc1.x_ = -0.2;
  loc1.y_ = -0.2;
  loc1.z_ = -0.6;
  if (!rob.LIN(loc1, fd, 10)) {
    std::cout << "robot add Linear command failure" << std::endl;
    return 0;
  }
  //ros::spin();

  return 0;
}