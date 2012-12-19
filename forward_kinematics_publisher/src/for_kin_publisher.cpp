#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

KDL::TreeFkSolverPos_recursive* fksolver = NULL;

void trajectoryCallback(const sensor_msgs::JointStateConstPtr& jointState)
{
	int dof = jointState->position.size();
	KDL::JntArray jointpositions = KDL::JntArray(dof);


	for (int i = 0; i < dof; ++i)
	{
		jointpositions(i) = jointState->position[i];
	}

	// Create the frame that will contain the results
	KDL::Frame cartpos;

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver->JntToCart(jointpositions,cartpos, "katana_gripper_tool_frame");
	if(kinematics_status>=0){
		std::cout << cartpos.p <<std::endl;
		printf("%s \n","Succes, thanks KDL!");
	}else{
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  KDL::Tree my_tree;
  
  ros::NodeHandle node;
  
  // set up robot description
  std::string robot_desc_string;
 node.param("robot_description", robot_desc_string, std::string());
 if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
	ROS_ERROR("Failed to construct kdl tree");
	return 1;
 }

 // Create solver based on kinematic tree
 fksolver = new KDL::TreeFkSolverPos_recursive(my_tree);


  
  // initialize joint state listener
	ros::Subscriber jointStateListener = node.subscribe("joint_states",
			1000,
			trajectoryCallback);

  
  ros::spin();

  delete fksolver;
  return 0;
}
