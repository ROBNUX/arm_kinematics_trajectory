#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <robnux_kinematics_map/quattro_4.hpp>
#include <robnux_kdl_common/pose.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <robnux_trajectory/line_trajectory.hpp>
#include <robnux_trajectory/shortest_quat_trajectory.hpp>
#include <robnux_trajectory/trajectory_segment.hpp>
#include <robnux_trajectory/s_curve.hpp>
#include <signal.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

using namespace kinematics_lib;
static const double RAD2DEG=57.2957805;

class TestQuattro4Kinematics {
 public:
   //! constructor 
   TestQuattro4Kinematics();
   //! destructor
   ~TestQuattro4Kinematics();
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
    kinematics_lib::Quattro_4  quattro_4_;
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


TestQuattro4Kinematics::TestQuattro4Kinematics(): nh_("~"),
    got_feedback_(false), new_goal_(false), motion_complete_(false) {
    noJoints_ = quattro_4_.GetActDoF();
    jnts_a_.resize(noJoints_, 0); // initializes to 0
    jnts_p_.resize(20); // 20 passive joints 
    
    parameters_.resize(11);
    
    parameters_[0] = 0.15; // base is a square of side-length 0.3
    parameters_[1] = -M_PI/2.0;  // first upper_arm yaw angle inside xy-plane (have to match with your model)
    parameters_[2] = 0.2;  // b1_, upper arm length
    parameters_[3] = 0;    // c1_, upper bar (of parallelogram) joint offset from the center plane of upper arm 
    parameters_[4] = 0.51; // length of the parallelogram
    parameters_[5] = 0.05;  // width of the parallelogram (not used so far)
    parameters_[6] = 0.127;   
    parameters_[7] = 0.09; // v1x_  (lower bar center of parallelogram of arm 1 toward the target hinge joint)
    parameters_[8] = 0.09; //   v1y_
    parameters_[9] = 0;    // v2x_  (lower bar center of parallelogram of arm 2 toward the target hinge joint)
    parameters_[10] = 0;   // v2y_
    
    //branchFlags_.resize(1,1);
    quattro_4_.SetGeometry(parameters_);
    
    // subscribe to joint feedback
    pub_joint_cmd_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 2);
    // subscribe to external user command
    goalSubscriber_= nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1,
          &TestQuattro4Kinematics::receiveGoal, this);
    
    int ret = quattro_4_.JntToCart(jnts_a_, &pose_c_);
    if (ret < 0) {
       ROS_ERROR("Forward kinematics failure, error code is, %d ", ret);
    } else {
         ROS_INFO("jnt feedback (%lf,%lf,%lf,%lf)", jnts_a_[0],
                         jnts_a_[1], jnts_a_[2], jnts_a_[3]);
         ROS_INFO("got feedback, and compute current pose as trans=%s, quat=%s",
                pose_c_.getTranslation().ToString().c_str(),
                pose_c_.getQuaternion().ToString(false).c_str());
         // if (!got_feedback_) {
           pose_d_ = pose_c_;
           last_pose_d_ = pose_d_;
           got_feedback_ = true;
         //}
    }
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
            boost::bind(&TestQuattro4Kinematics::motionControlLoop, this,
            100));
    ROS_INFO("test quattro object initilized");
}

TestQuattro4Kinematics::~TestQuattro4Kinematics() {
    if (control_thread_) {
        control_thread_->join();
        delete control_thread_;
        control_thread_ = NULL;
    }
}

void TestQuattro4Kinematics::motionControlLoop(const double frequency) {
    ros::NodeHandle nh;
    ros::Rate r = ros::Rate(frequency);
    
     // we only using line trajectory in this test
    std::shared_ptr<kinematics_lib::LineTrajectory> traj_t = NULL;
    std::shared_ptr<kinematics_lib::SCurveProfile> prof_t = NULL;
    
    std::shared_ptr<kinematics_lib::ShortestQuatTrajectory> traj_r = NULL;
    std::shared_ptr<kinematics_lib::SCurveProfile> prof_r = NULL;
    std::shared_ptr<kinematics_lib::TrajectorySegment> seg=NULL; 
    
    // start_vel and end_vel set as zeros
    kinematics_lib::Vec start_vel, end_vel;
    // starting and end_pos
    kinematics_lib::Vec start_pos, end_pos;
    kinematics_lib::Quaternion sq, eq;
    kinematics_lib::Quaternion sqdot = kinematics_lib::Quaternion::getZero();
    kinematics_lib::Quaternion eqdot = kinematics_lib::Quaternion::getZero();

    // apply inverse kinematics to obtain the position
    Pose tmp;
    Twist tw, acctw;
    double traj_time = 0;
    
    while (nh.ok()) { 
      //boost::recursive_mutex::scoped_lock l(goal_mutex_);
      if (new_goal_) {
         motion_complete_ = false;
          // if there is a new goal come, but traj not finishes, then we delete it anyway
         if (seg) {
             seg.reset();
             seg=nullptr;
         }
         ROS_INFO("motion loop got new goal");
         prof_t = std::make_shared<kinematics_lib::SCurveProfile>();
         prof_t->setConstraints(0.1, 0.4, 0.8);
         traj_t = std::make_shared<kinematics_lib::LineTrajectory>();
         traj_t->setProfile(prof_t);
         
         prof_r = std::make_shared<kinematics_lib::SCurveProfile>();
         prof_r->setConstraints(0.2, 0.4, 0.8);
         traj_r = std::make_shared<kinematics_lib::ShortestQuatTrajectory>();
         traj_r->setProfile(prof_r);
         
         // if got feedback
         start_pos= last_pose_d_.getTranslation();
         end_pos = pose_d_.getTranslation(); 
         traj_t->setBoundaryCond(start_pos, start_vel, end_pos, end_vel);
         sq = last_pose_d_.getQuaternion();
         eq = pose_d_.getQuaternion();
         traj_r->setBoundaryCond(sq, sqdot, eq, eqdot);
         // it will try to clean out component pointer when destructed
         seg= std::make_shared<kinematics_lib::TrajectorySegment>();
         seg->InitializeTrajectoryPlanners(traj_t, traj_r);
         traj_time = 0;
         last_pose_d_ = pose_d_;
         new_goal_ = false;
      }
      if (seg) {
          if (traj_time <= seg->Duration()) {
             seg->Trajectory(traj_time,  &tmp, &tw, &acctw);
             int ret=quattro_4_.CartToJnt(tmp, &jnts_a_);
             if (ret ==0 ) {
                 ROS_INFO("planned pose trans=%s, quat=%s",
                 tmp.getTranslation().ToString().c_str(),
                 tmp.getQuaternion().ToString(false).c_str());
                
                 int ret2=quattro_4_.JntToCart(jnts_a_, &tmp);
                 if (ret2 == 0) {
                   ROS_INFO("FK pose trans=%s, quat=%s",
                   tmp.getTranslation().ToString().c_str(),
                   tmp.getQuaternion().ToString(false).c_str());
                 } else {
                     ROS_INFO("FK pose verification fails");
                 }
                
                 int ret1=quattro_4_.CalcPassive(jnts_a_, tmp,
                          &jnts_p_);
                 if (ret1==0) {
                   ROS_INFO("jnt cmd (%lf,%lf,%lf,%lf)", jnts_a_[0],
                         jnts_a_[1], jnts_a_[2], jnts_a_[3]);
                   
                   SetJointCommands(jnts_a_, jnts_p_);
                 } else {
                     ROS_INFO("Quattro_4 CalcPassive fails, error code =%d",
                              ret1);
                 }
                 
             } else {
                 ROS_INFO("quattro_4_.CartToJnt fails, errorcode =%d", ret);
             }
             traj_time += 1/frequency;
          } else {
              seg.reset();
              seg = nullptr;
          }
      }
      ros::spinOnce();
      r.sleep();
  }
}

// jtCmd requires to be degree
void TestQuattro4Kinematics::SetJointCommands(
         const Eigen::VectorXd &jtCmd_a,
         const Eigen::VectorXd &jtCmd_p) {
    sensor_msgs::JointState msgs;
    msgs.header.stamp = ros::Time::now();
    msgs.name.resize(24);
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
    msgs.name[20] = "lleg1_base_1";
    msgs.name[21] = "base_1_2";
    msgs.name[22] = "lleg3_base_3";
    msgs.name[23] = "base_3_4";

    msgs.position.resize(24);
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

/*
void TestQuattroKinematics::processFeedback(
      const sensor_msgs::JointState& servo_des_states) {

    // boost::recursive_mutex::scoped_lock l(feedback_mutex_);
    for (int i = 0; i < noJoints_; i++) {
         jnts_c_[i] = servo_des_states.position[i];
    }
    int ret = quattro_.JntToCart(jnts_c_, &pose_c_);
    if (ret < 0) {
       ROS_ERROR("Forward kinematics failure, error code is, %d ", ret);
    } else {
         ROS_INFO("jnt feedback (%lf,%lf,%lf,%lf)", jnts_c_[0],
                         jnts_c_[1], jnts_c_[2], jnts_c_[3]);
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
 */ 

void TestQuattro4Kinematics::receiveGoal(
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



boost::shared_ptr<TestQuattro4Kinematics>  test_quattro4_ptr;

void sigintHandler(int sig) {
    ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_quattro4_rviz");

  ros::NodeHandle nhl;
          
  // override default sigint handler
  signal(SIGINT, sigintHandler);
  
  test_quattro4_ptr.reset(new TestQuattro4Kinematics());
  
  ros::spin();
  
  test_quattro4_ptr.reset();

  return 0;
}