#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <robnux_kinematics_map/quattro.hpp>
#include <robnux_kdl_common/pose.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <robnux_trajectory/line_trajectory.hpp>
#include <robnux_trajectory/s_curve.hpp>
#include <signal.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

using namespace kinematics_lib;
static const double RAD2DEG=57.2957805;

class TestQuattroKinematics {
 public:
   //! constructor 
   TestQuattroKinematics();
   //! destructor
   ~TestQuattroKinematics();
   //! send joint position command
   void SetJointCommands(const Eigen::VectorXd &jtCmd_a,
                         const Eigen::VectorXd &jtCmd_p);
   //! processFeedback from robot
   //void processFeedback(const sensor_msgs::JointState& servo_des_states);
   //! receive goal from users
   void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
   //! control loop
   void motionControlLoop(const double frequency);
   
 private:
     
    // ros node handle
    ros::NodeHandle nh_;
    // control thread
    boost::thread* control_thread_;
    // quattro kinematic model object
    kinematics_lib::Quattro  quattro_;
    // desired active joint angles
    Eigen::VectorXd jnts_a_;
    // desired passive joint angles
    Eigen::VectorXd jnts_p_;
    // topics for commanding quattro joints
    // std::vector<std::string> topicNames_;
    
    //! joint command publishers
    ros::Publisher pub_joint_cmd_;
    //! subscriber to user setting goals
    ros::Subscriber goalSubscriber_;
    //! joint feedback subscriber
    // ros::Subscriber jointStates_;
    
    // pose of robot
    kinematics_lib::Pose  pose_c_, pose_d_, last_pose_d_;
    // have we got feedback
    bool got_feedback_;
    // got new goal flag
    bool new_goal_;
    // motion complete flag
    bool motion_complete_;

    // number of joints
    int noJoints_;
    
    // mutex
    // boost::recursive_mutex  feedback_mutex_, goal_mutex_;
    // kinematic parameters
    Eigen::VectorXd parameters_;
    //std::vector<int> branchFlags_;
};


TestQuattroKinematics::TestQuattroKinematics(): nh_("~"),
    got_feedback_(false), new_goal_(false), motion_complete_(false) {
    noJoints_ = quattro_.GetActDoF();
    jnts_a_.resize(noJoints_, 0); // initializes to 0
    jnts_p_.resize(17); // 13 passive joints 
    
    parameters_.resize(8);
    parameters_[0] = 0.15; // base is a square of side-length 0.3
    parameters_[1] = -M_PI/2.0;
    parameters_[2] = 0.2;
    parameters_[3] = 0;
    parameters_[4] = 0.51;
    parameters_[5] = 0.05;
    parameters_[6] = 0;
    parameters_[7] = 0.09 * sqrt(2);
    //branchFlags_.resize(1,1);
    quattro_.SetGeometry(parameters_);
    
    // subscribe to joint feedback
    pub_joint_cmd_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 2);
    // subscribe to external user command
    goalSubscriber_= nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1,
          &TestQuattroKinematics::receiveGoal, this);
    
    int ret = quattro_.JntToCart(jnts_a_, &pose_c_);
    if (ret < 0) {
       ROS_ERROR("Forward kinematics failure, error code is, %d ", ret);
    } else {
       ROS_INFO("jnt feedback (%lf,%lf,%lf,%lf)", jnts_a_[0],
                         jnts_a_[1], jnts_a_[2], jnts_a_[3]);
       pose_d_ = pose_c_;
       last_pose_d_ = pose_d_;
       got_feedback_ = true;
    }
    // set up control loop, 10 hz
    control_thread_ = new boost::thread(
            boost::bind(&TestQuattroKinematics::motionControlLoop, this,
            100));
    ROS_INFO("test quattro object initilized");
}

TestQuattroKinematics::~TestQuattroKinematics() {
    if (control_thread_) {
        control_thread_->join();
        delete control_thread_;
        control_thread_ = NULL;
    }
}

void TestQuattroKinematics::motionControlLoop(const double frequency) {
    ros::NodeHandle nh;
    ros::Rate r = ros::Rate(frequency);
    
     // we only using line trajectory in this test
    std::shared_ptr<kinematics_lib::LineTrajectory> traj = NULL;
    std::shared_ptr<kinematics_lib::SCurveProfile> prof = NULL;
    
    // start_vel and end_vel set as zeros
    kinematics_lib::Vec start_vel, end_vel;
    // starting and end_pos
    kinematics_lib::Vec start_pos, end_pos;
    kinematics_lib::Vec temps, tempv, tempa;
  
  
    // apply inverse kinematics to obtain the position
    Pose tmp;
    Quaternion default_q;
    tmp.setQuaternion(default_q);
    double traj_time = 0;
    
    while (nh.ok()) { 
      //boost::recursive_mutex::scoped_lock l(goal_mutex_);
      if (new_goal_) {
         motion_complete_ = false;
          // if there is a new goal come, but traj not finishes, then we delete it anyway
         if (traj) {
             traj.reset();
             traj=nullptr;
         }
         ROS_INFO("motion loop got new goal");
         prof = std::make_shared<kinematics_lib::SCurveProfile>();
         prof->setConstraints(0.1, 0.4, 0.8);
         traj = std::make_shared<kinematics_lib::LineTrajectory>();
         traj->setProfile(prof);
         // if got feedback
         start_pos= last_pose_d_.getTranslation();
         end_pos = pose_d_.getTranslation(); 
         traj->setBoundaryCond(start_pos, start_vel, end_pos, end_vel);
         traj_time = 0;
         last_pose_d_ = pose_d_;
         new_goal_ = false;
      }
      if (traj) {
          if (traj_time <= traj->Duration()) {
             traj->Trajectory(traj_time,  &temps, &tempv, &tempa);
             tmp.setTranslation(temps);
             int ret=quattro_.CartToJnt(tmp, &jnts_a_);
             if (ret ==0 ) {
                 int ret1=quattro_.CalcPassive(jnts_a_, tmp,
                          &jnts_p_);
                 if (ret1==0) {
                   ROS_INFO("jnt cmd (%lf,%lf,%lf,%lf)", jnts_a_[0],
                         jnts_a_[1], jnts_a_[2], jnts_a_[3]);
                   
                   SetJointCommands(jnts_a_, jnts_p_);
                 }
             }
             traj_time += 1/frequency;
          } else {
              traj.reset();
              traj = nullptr;
          }
      }
      ros::spinOnce();
      r.sleep();
  }
}

// jtCmd requires to be degree
void TestQuattroKinematics::SetJointCommands(const Eigen::VectorXd &jtCmd_a,
                                             const Eigen::VectorXd &jtCmd_p) {
    sensor_msgs::JointState msgs;
    msgs.header.stamp = ros::Time::now();
    msgs.name.resize(21);
    msgs.name[0]="JOINT_1_ACT";
    msgs.name[1]="JOINT_2_ACT";
    msgs.name[2]="JOINT_3_ACT";
    msgs.name[3]="JOINT_4_ACT";
    msgs.name[4]="uleg1_con";
    msgs.name[5]="con_lleg_1";
    msgs.name[6]="con2_lleg_1";
    msgs.name[7]="uleg2_con";
    msgs.name[8]="con_lleg_2";
    msgs.name[9]="con2_lleg_2";
    msgs.name[10] = "uleg3_con";
    msgs.name[11] ="con_lleg_3";
    msgs.name[12] = "con2_lleg_3";
    msgs.name[13] = "uleg4_con";
    msgs.name[14] = "con_lleg_4";
    msgs.name[15] = "con2_lleg_4";
    msgs.name[16] = "con43_lleg_4";
    msgs.name[17] = "con33_lleg_3";
    msgs.name[18] = "con23_lleg_2";
    msgs.name[19] = "con13_lleg_1";
    msgs.name[20] = "lleg1_base";

    msgs.position.resize(21);
    for (size_t i = 0; i < jtCmd_a.size(); i++) {
        // msgs.name[i] = "active joint " + std::to_string(i);
        msgs.position[i] = jtCmd_a(i);
    }
    for (size_t i = 0; i < jtCmd_p.size(); i++) {
        // msgs.name[4+i] = "passive joint " + std::to_string(i);
        msgs.position[4+i] = jtCmd_p(i);
    }
    pub_joint_cmd_.publish(msgs);
}

void TestQuattroKinematics::receiveGoal(
    const geometry_msgs::PoseStamped::ConstPtr& goal) {
    //boost::recursive_mutex::scoped_lock l(goal_mutex_);
    // if (motion_complete_ && !new_goal_) {
    if (!new_goal_) {
        geometry_msgs::Pose p_goal = goal->pose;
        Vec trans(p_goal.position.x, p_goal.position.y, p_goal.position.z);
        kinematics_lib::Quaternion q(p_goal.orientation.w, p_goal.orientation.x,
                     p_goal.orientation.y, p_goal.orientation.z);
        pose_d_.setTranslation(trans);
        pose_d_.setQuaternion(q);
        new_goal_ = true;
        ROS_INFO("receive goal, position %s, orientation %s " 
                , trans.ToString().c_str(),
                q.ToString(false).c_str());
        motion_complete_ = false;
   }
}


std::shared_ptr<TestQuattroKinematics>  test_quattro_ptr;

void sigintHandler(int sig) {
    ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_quattro_rviz");

  ros::NodeHandle nhl;
          
  // override default sigint handler
  signal(SIGINT, sigintHandler);
  
  test_quattro_ptr= std::make_shared<TestQuattroKinematics>();
  
  ros::spin();
  
  test_quattro_ptr.reset();

  return 0;
}