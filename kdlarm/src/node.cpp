#include "kdlArm.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kdl_arm");
  ros::NodeHandle nh("~");

  // get some parameters for arm kinematics details
  std::string urdf_param_name;
  nh.param("urdf_param_name", urdf_param_name, std::string());
  std::string chain_root;
  nh.param("chain_root", chain_root, std::string());
  std::string chain_tip;
  nh.param("chain_tip", chain_tip, std::string());
  std::string input_pose_topic;
  nh.param("input_pose_topic", input_pose_topic, std::string());

  // main KDL based controller object for the arm
  kdlArm bot(nh, urdf_param_name, chain_root, chain_tip, input_pose_topic);

  while(ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
