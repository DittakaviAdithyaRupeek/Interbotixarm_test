#include "kdlArm.h"

kdlArm::kdlArm(const ros::NodeHandle& nodehandle, std::string input_pose_topic) {
  _node = nodehandle;
  _mode = 0;
  _pose_sub = _node.subscribe(input_pose_topic, 1000, &kdlArm::poseCallback, this);
  _joint_pub = _node.advertise<std_msgs::Float64MultiArray>(JS_KDL, 1000);
  _dest.p(0) = 0.44700856;
  _dest.p(1) = 0;
  _dest.p(2) = 0.36201706;
  _fksolver = NULL;
  _iksolverv = NULL;
  _iksolverpos = NULL;
}

kdlArm::kdlArm(const ros::NodeHandle& nodehandle, std::string urdf_param_name, std::string chain_root, std::string chain_tip, std::string input_pose_topic) {
  _node = nodehandle;
  _mode = 0;
  _node.param(urdf_param_name, _robot_description, std::string());
  _pose_sub = _node.subscribe(input_pose_topic, 1000, &kdlArm::poseCallback, this);
  _joint_pub = _node.advertise<std_msgs::Float64MultiArray>(JS_KDL, 1000);
  _dest.p(0) = 0.44700856;
  _dest.p(1) = 0;
  _dest.p(2) = 0.36201706;

  if(createTreeFromURDF()) {
    if(createChainFromTree(chain_root, chain_tip)) {
      setArmJointLimitsFromParam();
      if(configureSolvers()) {
          ROS_INFO("KDL Arm Configuration Done !");
      }
    }
  }
}

kdlArm::~kdlArm() {
  delete _iksolverpos;
  delete _fksolver;
  delete _iksolverv;
}

void kdlArm::setTree(KDL::Tree tree) {
  _tree = tree;
}

void kdlArm::setChain(KDL::Chain chain) {
  _chain = chain;
  // resize joint limits JntArray
  _joints_max.resize(_chain.getNrOfJoints());
  _joints_min.resize(_chain.getNrOfJoints());

  // resize joint state initial and final solutions JntArray
  _q_init.resize(_chain.getNrOfJoints());
  for(int i = 0; i < _chain.getNrOfJoints(); i++) {
    _q_init(i) = 0;
  }
  _q_out.resize(_chain.getNrOfJoints());
}

void kdlArm::setRobotDescription(std::string robot_description) {
  _robot_description = robot_description;
}

void kdlArm::setqInit(KDL::JntArray q_init) {
  _q_init = q_init;
}

void kdlArm::setqOut(KDL::JntArray q_out) {
  _q_out = q_out;
}

bool kdlArm::createTreeFromURDF() {
  bool ret = kdl_parser::treeFromString(_robot_description, _tree);
  if (ret) {
    ROS_INFO("KDL Tree Construction from URDF - Success !");
  }
  else {
    ROS_ERROR("KDL Tree Construction from URDF - Failed !");
  }
  return ret;
}

bool kdlArm::createChainFromTree(std::string chain_root, std::string chain_tip) {
    bool ret = _tree.getChain(chain_root, chain_tip, _chain);
    if(ret) {
      // resize joint limits JntArray
      _joints_max.resize(_chain.getNrOfJoints());
      _joints_min.resize(_chain.getNrOfJoints());

      // resize joint state initial and final solutions JntArray
      _q_init.resize(_chain.getNrOfJoints());
      for(int i = 0; i < _chain.getNrOfJoints(); i++) {
        _q_init(i) = 0.1;
      }
      _q_out.resize(_chain.getNrOfJoints());

      ROS_INFO("KDL Chain Construction from Tree - Success !");
      ROS_INFO("Number of Joints = %d", _chain.getNrOfJoints());
      ROS_INFO("Number of Segments = %d", _chain.getNrOfSegments());
    }
    else {
      ROS_ERROR("KDL Chain Construction from Tree - Failed !");
    }
    return ret;
}

void kdlArm::printTreeSegments() {
  KDL::SegmentMap segment_map = _tree.getSegments();
  ROS_INFO("Printing Tree segments !");
  for(auto it : segment_map) {
    ROS_INFO("%s", it.first.c_str());
  }
}

void kdlArm::setArmJointLimitsFromParam() {
  _node.getParam("/joint_limits/1_min", _joints_min(0));
  _node.getParam("/joint_limits/1_max", _joints_max(0));
  _node.getParam("/joint_limits/2_min", _joints_min(1));
  _node.getParam("/joint_limits/2_max", _joints_max(1));
  _node.getParam("/joint_limits/3_min", _joints_min(2));
  _node.getParam("/joint_limits/3_max", _joints_max(2));
  _node.getParam("/joint_limits/4_min", _joints_min(3));
  _node.getParam("/joint_limits/4_max", _joints_max(3));
  _node.getParam("/joint_limits/5_min", _joints_min(4));
  _node.getParam("/joint_limits/5_max", _joints_max(4));
  _node.getParam("/joint_limits/6_min", _joints_min(5));
  _node.getParam("/joint_limits/6_max", _joints_max(5));
  ROS_DEBUG("Joint Lower Limits: [%f, %f, %f, %f, %f, %f]", _joints_min(0), _joints_min(1), _joints_min(2), _joints_min(3), _joints_min(4), _joints_min(5));
  ROS_DEBUG("Joint Upper Limits: [%f, %f, %f, %f, %f, %f]", _joints_max(0), _joints_max(1), _joints_max(2), _joints_max(3), _joints_max(4), _joints_max(5));
}

bool kdlArm::configureSolvers() {
  // Forward position solver
  _fksolver = new KDL::ChainFkSolverPos_recursive (_chain);
  // Inverse velocity solver
  _iksolverv = new KDL::ChainIkSolverVel_pinv (_chain);
  // Inverse position solver
  _iksolverpos = new KDL::ChainIkSolverPos_NR_JL (_chain, _joints_min, _joints_max, *_fksolver, *_iksolverv, IK_ITERS, IK_ACCU);

  if(_iksolverv != NULL) {
    ROS_INFO("KDL Solvers Configuration - Success !");
    return true;
  }
  else {
    ROS_INFO("KDL Solvers Configuration - Failed !");
    return false;
  }
}

bool kdlArm::runIKSolver(KDL::JntArray q_init, KDL::Frame desired_pos, KDL::JntArray &q_out) {
  int ret = _iksolverpos->CartToJnt(q_init, desired_pos, q_out);
	if(ret < 0) {
		ROS_ERROR("IK - No Solution Found !");
    return false;
	}
  else {
		ROS_INFO("IK - Solution Found !");
		ROS_INFO("JS: [%f, %f, %f, %f, %f, %f]", q_out(0), q_out(1), q_out(2), q_out(3), q_out(4), q_out(5));
    return true;
	}
}

void kdlArm::poseCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  float x = msg->linear.x * ARM_LIMIT / JS_LIMIT;
  float y = msg->linear.y * ARM_LIMIT / JS_LIMIT;
  float z = msg->linear.z * ARM_LIMIT / JS_LIMIT;

  if(_mode == 0) {
    _dest.p(0) = (x == 0) ? _dest.p(0) : ((x < 0) ? (_dest.p(0) - DELTA) : (_dest.p(0) + DELTA));
    _dest.p(1) = (y == 0) ? _dest.p(1) : ((y < 0) ? (_dest.p(1) - DELTA) : (_dest.p(1) + DELTA));
    _dest.p(2) = (z == 0) ? _dest.p(2) : ((z < 0) ? (_dest.p(2) - DELTA) : (_dest.p(2) + DELTA));
  }
  else if(_mode == 1) {
    _dest.p(0) = x;
    _dest.p(0) = y;
    _dest.p(0) = z;
  }
  else {
    ROS_ERROR("Invalid Operating Mode !");
  }
  ROS_DEBUG("Desired Cartesian Position: [%f %f %f]", _dest.p(0), _dest.p(1), _dest.p(2));

  if(runIKSolver(_q_init, _dest, _q_out)) {
    std_msgs::Float64MultiArray jointmsg;
    jointmsg.data.push_back(_q_out(0));
    jointmsg.data.push_back(_q_out(1));
    jointmsg.data.push_back(_q_out(2));
    jointmsg.data.push_back(_q_out(3));
    jointmsg.data.push_back(_q_out(4));
    jointmsg.data.push_back(_q_out(5));
    _joint_pub.publish(jointmsg);
    _q_init=_q_out;
  }
}
