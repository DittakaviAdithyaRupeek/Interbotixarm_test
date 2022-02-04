#include <string>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define IK_ITERS 1000
#define IK_ACCU 0.001
#define JS_KDL  "/kdl/joint_states"
#define ARM_LIMIT 0.5
#define JS_LIMIT 327
#define DELTA 0.01

class kdlArm {
public:
  // essential class customary functions
  kdlArm(const ros::NodeHandle&, std::string);
  kdlArm(const ros::NodeHandle&, std::string, std::string, std::string, std::string);
  virtual ~kdlArm();

  // KDL library based feature functions
  bool createTreeFromURDF();
  bool createChainFromTree(std::string, std::string);
  void setArmJointLimitsFromParam();
  bool configureSolvers();
  bool runIKSolver(KDL::JntArray, KDL::Frame, KDL::JntArray&);
  void poseCallback(const geometry_msgs::Twist::ConstPtr&);

  // Setters for member variables
  void setTree(KDL::Tree);
  void setChain(KDL::Chain);
  void setRobotDescription(std::string);
  void setqInit(KDL::JntArray);
  void setqOut(KDL::JntArray);

  // Functions for debugging and printing things
  void printTreeSegments();

private:
  int _mode;
  std::string _robot_description;
  ros::NodeHandle _node;
  ros::Subscriber _pose_sub;
  ros::Publisher  _joint_pub;
  KDL::Tree _tree;
  KDL::Chain _chain;
  KDL::Frame _dest;
  KDL::JntArray _joints_min;
  KDL::JntArray _joints_max;
  KDL::JntArray _q_init;
  KDL::JntArray _q_out;
  KDL::ChainIkSolverPos_NR_JL *_iksolverpos;
  KDL::ChainFkSolverPos_recursive *_fksolver;
  KDL::ChainIkSolverVel_pinv *_iksolverv;
};
