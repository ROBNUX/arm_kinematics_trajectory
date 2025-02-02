#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <kinematics_map/quattro.hpp>
#include <kdl_common/pose.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory/line_trajectory.hpp>
#include <trajectory/s_curve.hpp>
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
   void SetJointCommands(Eigen::VectorXd jtCmd);
   //! processFeedback from robot
   void processFeedback(const sensor_msgs::JointState& servo_des_states);
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
    // current joint angles
    Eigen::VectorXd jnts_c_;
    // desired joint angles
    Eigen::VectorXd jnts_d_;
    // topics for commanding quattro joints
    std::vector<std::string> topicNames_;
    //! joint command publishers
    std::vector<ros::Publisher> pub_joint_cmd_;
    //! subscriber to user setting goals
    ros::Subscriber goalSubscriber_;
    //! joint feedback subscriber
    ros::Subscriber jointStates_;
    
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
    boost::recursive_mutex  feedback_mutex_, goal_mutex_;
    // kinematic parameters
    Eigen::VectorXd parameters_;
    std::vector<int> branchFlags_;
};


TestQuattroKinematics::TestQuattroKinematics(): nh_("~"),
    got_feedback_(false), new_goal_(false), motion_complete_(false) {
    noJoints_ = quattro_.GetActDoF();
    jnts_c_.resize(noJoints_);
    jnts_d_.resize(noJoints_);
    topicNames_.resize(noJoints_);
    pub_joint_cmd_.resize(noJoints_);
    // initialize jnt cmd topic names and publisher
    for (int i=0; i< noJoints_; i++) {
        topicNames_[i] ="/delta/joint" +std::to_string(i+1) + 
                        "_position_controller/command";
        pub_joint_cmd_[i] = nh_.advertise<std_msgs::Float64>(topicNames_[i], 100);
    }
    
    
    parameters_.resize(8);
    parameters_[0] = 0.15; // base is a square of side-length 0.3
    parameters_[1] = -M_PI/2.0;
    parameters_[2] = 0.2;
    parameters_[3] = 0;
    parameters_[4] = 0.51;
    parameters_[5] = 0.05;
    parameters_[6] = 0;
    parameters_[7] = 0.09 * sqrt(2);
    branchFlags_.resize(1,1);
    quattro_.SetGeometry(parameters_);
    
    // subscribe to joint feedback
    jointStates_ = nh_.subscribe("/delta/joint_states", 
                         100, &TestQuattroKinematics::processFeedback, this);
    // subscribe to external user command
    goalSubscriber_= nh_.subscribe<geometry_msgs::PoseStamped>("goal", 100,
          &TestQuattroKinematics::receiveGoal, this);
    
    /*
    ros::Time stamp = ros::Time::now();
    ros::Time waitLimit = stamp + ros::Duration(20); // 5 seconds
    
    while ((!got_feedback_) && (ros::Time::now() <= waitLimit)) {
       // ros::spinOnce(); 
       usleep(1000); 
    }
    if (got_feedback_) {
       ROS_INFO("got feedback");
    } else {
       ROS_INFO("something is wrong, no feedback"); 
    }
    boost::recursive_mutex::scoped_lock l(feedback_mutex_);
    pose_d_ = pose_c_;
    last_pose_d_ = pose_d_;
    */
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
             traj = nullptr;
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
             int ret=quattro_.CartToJnt(tmp, &jnts_d_);
             if (ret ==0 ) {
                ROS_INFO("jnt cmd (%lf,%lf,%lf,%lf)", jnts_d_[0],
                         jnts_d_[1], jnts_d_[2], jnts_d_[3]);
                SetJointCommands(jnts_d_);
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
void TestQuattroKinematics::SetJointCommands(Eigen::VectorXd jtCmd) {
    std_msgs::Float64 msgs;
    for (unsigned int i = 0; i < noJoints_; i++) {
        msgs.data = jtCmd(i) * RAD2DEG;
        pub_joint_cmd_[i].publish(msgs);
    }
}

void TestQuattroKinematics::processFeedback(
      const sensor_msgs::JointState& servo_des_states) {

    // boost::recursive_mutex::scoped_lock l(feedback_mutex_);
    for (int i = 0; i < noJoints_; i++) {
         jnts_c_(i) = servo_des_states.position[i];
    }
    int ret = quattro_.JntToCart(jnts_c_, &pose_c_);
    if (ret < 0) {
       ROS_ERROR("Forward kinematics failure, error code is, %d ", ret);
    } else {
         ROS_INFO("jnt feedback (%lf,%lf,%lf,%lf)", jnts_c_(0),
                         jnts_c_(1), jnts_c_(2), jnts_c_(3));
         //ROS_INFO("got feedback, and compute current pose as %s",
         //       pose_c_.getTranslation().ToString().c_str());
         if (!got_feedback_) {
           pose_d_ = pose_c_;
           last_pose_d_ = pose_d_;
           got_feedback_ = true;
         }
    }
    
    if (got_feedback_) {
      Vec diffpos = pose_c_.getTranslation() - pose_d_.getTranslation();
      double diffpos_norm = diffpos.Norm();
      //ROS_INFO("diffpos norm %lf", diffpos_norm);
      // boost::recursive_mutex::scoped_lock l1(goal_mutex_);
      if (diffpos.Norm() < 0.01 && !motion_complete_) {
         motion_complete_ = true;
         ROS_INFO("motion complete, desired %s, current %s",
                 pose_d_.getTranslation().ToString().c_str(),
                 pose_c_.getTranslation().ToString().c_str());
         
      }
    }
     
}

void TestQuattroKinematics::receiveGoal(
    const geometry_msgs::PoseStamped::ConstPtr& goal) {
    //boost::recursive_mutex::scoped_lock l(goal_mutex_);
    // if (motion_complete_ && !new_goal_) {
    if (!new_goal_) {
        geometry_msgs::Pose p_goal = goal->pose;
        Vec trans(p_goal.position.x, p_goal.position.y, p_goal.position.z);
        Quaternion q(p_goal.orientation.w, p_goal.orientation.x,
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
  ros::init(argc, argv, "quattro");

  ros::NodeHandle nhl;
          
  // override default sigint handler
  signal(SIGINT, sigintHandler);
  
  test_quattro_ptr= std::make_shared<TestQuattroKinematics>();
  
  ros::spin();
  
  test_quattro_ptr.reset();

  return 0;
}