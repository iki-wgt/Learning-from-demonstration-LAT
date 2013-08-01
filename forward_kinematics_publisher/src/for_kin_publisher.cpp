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
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/utilities/utility.h>

KDL::TreeFkSolverPos_recursive* fksolver = NULL;
KDL::ChainFkSolverPos_recursive* chainfksolver = NULL;
KDL::Tree my_tree;

void trajectoryCallback(const sensor_msgs::JointStateConstPtr& jointState)
{
	int dof = jointState->position.size();
	int dofJac = dof - 2;
	KDL::JntArray jointpositions = KDL::JntArray(dof);
	KDL::JntArray jointpositionsJac = KDL::JntArray(dofJac);


	for (int i = 0; i < dof; ++i)
	{
		jointpositions(i) = jointState->position[i];
		//std::cout << jointState->position[i] << "\t";
	}

	std::cout << std::endl;

	for (int i = 0; i < dofJac; ++i)
	{
		jointpositionsJac(i) = jointState->position[i];
	}

	// Create the frame that will contain the results
	KDL::Frame cartpos;

	//double roll, pitch, yaw;
	// Calculate forward position kinematics
	int kinematics_status;
	//kinematics_status = fksolver->JntToCart(jointpositions,cartpos, "katana_gripper_tool_frame");
	kinematics_status = chainfksolver->JntToCart(jointpositionsJac,cartpos);
	if(kinematics_status>=0){
		std::cout << cartpos.p << std::endl;

		//std::cout << cartpos.M.GetRot() << std::endl;
		//printf("%s \n","Succes, thanks KDL!");
	}else{
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}


	// just for testing JntToJac
	// create Jacobian
	KDL::Chain chain;		// create chain
	my_tree.getChain("katana_base_link", "katana_gripper_tool_frame", chain);	//TODO: Katana specific
for (unsigned int segmentIdx = 0; segmentIdx < chain.getNrOfSegments(); ++segmentIdx) {
		KDL::Segment segment = chain.getSegment(segmentIdx);
		ROS_INFO("Segment Idx: %d, name: %s", segmentIdx, segment.getName().c_str());
		ROS_INFO("[%f, %f, %f]", segment.getFrameToTip().p.x(), segment.getFrameToTip().p.y(),segment.getFrameToTip().p.z());
	}
	KDL::ChainJntToJacSolver jacSolver = KDL::ChainJntToJacSolver(chain);
	KDL::Jacobian kdlJacobian(dofJac);

	jacSolver.JntToJac(jointpositionsJac, kdlJacobian);

	//std::cout << kdlJacobian.data << std::endl;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  

  ros::NodeHandle node;
  
  // set up robot description
  std::string robot_desc_string;
 node.param("robot_description", robot_desc_string, std::string());
 if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
	ROS_ERROR("Failed to construct kdl tree");
	return 1;
 }
KDL::Chain chain;		// create chain
	my_tree.getChain("katana_base_link", "katana_gripper_tool_frame", chain);	//TODO: Katana specific
 // Create solver based on kinematic tree
 fksolver = new KDL::TreeFkSolverPos_recursive(my_tree);
 chainfksolver = new KDL::ChainFkSolverPos_recursive(chain);


  
  // initialize joint state listener
	ros::Subscriber jointStateListener = node.subscribe("joint_states",
			1000,
			trajectoryCallback);

  
  ros::spin();

  delete fksolver;
  return 0;
}
