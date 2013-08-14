/*
 * lat_reproducer.cpp
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */
#include "ros_lfd_lat/lat_reproducer.h"

std::deque<object> objects;
std::vector<std::string> jointNames;
std::vector<std::string> gripperJointNames;
std::vector<double> jointPositions;
std::vector<double> gripperJointPositions;


std::vector<std::string> getAvailableTrajectories(std::string trajectoryDir)
{
	std::string filename;

	std::vector<std::string> trajectories;
	const std::string POSTFIX = ".tra";	//every trajectory directory ends with .tra

	boost::filesystem::path trajDir;

	if(trajectoryDir == USE_USER_HOME_STRING)
	{
		trajDir = getHomeDir();
	}
	else
	{
		trajDir = boost::filesystem::path(trajectoryDir);

		if(!boost::filesystem::is_directory(trajDir))
		{
			ROS_ERROR("%s is not a directory! (Will use home dir instead)", trajectoryDir.c_str());
			trajDir = getHomeDir();
		}
	}

	boost::filesystem::directory_iterator endIter;

	for (boost::filesystem::directory_iterator dirIter(trajDir); dirIter != endIter; ++dirIter)
	{
		if(boost::filesystem::is_directory(dirIter->path()))
		{
			if(boost::algorithm::ends_with(
					dirIter->path().filename().c_str(),
					POSTFIX))
			{
				filename = dirIter->path().filename().c_str();
				boost::algorithm::erase_tail(filename, POSTFIX.size());
				trajectories.push_back(filename);
			}
		}
	}

	return trajectories;
}

void objectCallback(const actionlib::SimpleClientGoalState& state,
		const object_recognition_msgs::ObjectRecognitionResultConstPtr& result)
{
	tf::TransformListener tfListener;
	//tfListener.transformPoint()

	int objCount = result->recognized_objects.objects.size();
	for (int i = 0; i < objCount; ++i) {
		object obj = object();
		geometry_msgs::PointStamped pointStampedIn, pointStampedOut;
		pointStampedIn.point = result->recognized_objects.objects[i].pose.pose.pose.position;
		pointStampedIn.header.frame_id = result->recognized_objects.objects[i].pose.header.frame_id;
		ROS_INFO("frame id %s", pointStampedIn.header.frame_id.c_str());
		ROS_INFO("in coordinates: [%f, %f, %f]", pointStampedIn.point.x, pointStampedIn.point.y, pointStampedIn.point.z);
		ros::Duration waitTimeout(3.0);
		tfListener.waitForTransform(OBJECT_TARGET_FRAME, pointStampedIn.header.frame_id, pointStampedIn.header.stamp, waitTimeout);
		tfListener.transformPoint(OBJECT_TARGET_FRAME, pointStampedIn, pointStampedOut);
		ROS_INFO("pointStampedOut frame id %s", pointStampedOut.header.frame_id.c_str());
		obj.set_name(result->recognized_objects.objects[i].type.key);
		obj.add_coordinate(pointStampedOut.point.x);
			//result->recognized_objects.objects[i].pose.pose.pose.position.x);
		obj.add_coordinate(pointStampedOut.point.y);
			//result->recognized_objects.objects[i].pose.pose.pose.position.y);
		obj.add_coordinate(pointStampedOut.point.z);
			//result->recognized_objects.objects[i].pose.pose.pose.position.z);

		objects.push_back(obj);

		ROS_INFO("Added object %s on coordinate [%f, %f, %f]", obj.get_name().c_str(),
				obj.get_coordinate(0),
				obj.get_coordinate(1),
				obj.get_coordinate(2));
	}
}

void activeCb()
{
	ROS_INFO("Object recognition goal just went active");
}

void feedbackCb(
		const object_recognition_msgs::ObjectRecognitionFeedbackConstPtr& feedback)
{

}

std::vector<std::string> getJointNames(bool inSimulation)
{
	// initialize joint state listener
	ros::NodeHandle jointNode;
	ros::Subscriber jointStateListener = jointNode.subscribe("joint_states",
		1,
		jointStateCallback);
	ROS_INFO("subscribed");

	jointNames.clear();
	jointPositions.clear();

	while(jointNames.empty())
	{
		ros::Duration(0.0001).sleep();
		ros::spinOnce();
	}

	if(inSimulation)
	{
		gripperJointNames.clear();
		gripperJointPositions.clear();

		// copy the gripper names and joint positions
		gripperJointNames.push_back(jointNames.at(jointNames.size() - 2));
		gripperJointNames.push_back(jointNames.at(jointNames.size() - 1));

		gripperJointPositions.push_back(
			jointPositions.at(jointPositions.size() - 2)
		);

		gripperJointPositions.push_back(
			jointPositions.at(jointPositions.size() - 1)
		);

		// in Gazebo both finger joints
		jointNames.pop_back();
		jointPositions.pop_back();
	}

	// in real life the katana_l_finger_joint has to be removed only
	jointPositions.pop_back();
	jointNames.pop_back();

	return jointNames;

	// with the destruction of jointStateListener the topic is automatically
	// unsubscribed
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointState)
{
	ROS_INFO("In joint state callback");
	if(jointNames.empty())
	{
		jointNames = jointState->name;
		jointPositions = jointState->position;
		ROS_INFO("Copied joint names");
	}
}

void moveRobotToStartPos(const std::deque<std::deque<double> >& trajectory, bool inSimulation)
{
	unsigned int armJointCount = 6;
	if(inSimulation)
	{
		armJointCount = 5;
	}

	MoveClient moveClient("katana_arm_controller/joint_movement_action");
	TrajClient gripperClient("katana_arm_controller/gripper_joint_trajectory_action");
	moveClient.waitForServer();

	katana_msgs::JointMovementGoal moveGoal;
	pr2_controllers_msgs::JointTrajectoryGoal gripperGoal;
	moveGoal.jointGoal.name = getJointNames(inSimulation);

	if(inSimulation)
	{
		gripperGoal.trajectory.joint_names = gripperJointNames;
		gripperGoal.trajectory.points.resize(2);	// current joint state
													// and desired joint state

		// current state
		gripperGoal.trajectory.points[0].positions.resize(2);
		gripperGoal.trajectory.points[0].positions[0] = gripperJointPositions[0];
		gripperGoal.trajectory.points[0].positions[1] = gripperJointPositions[1];
		gripperGoal.trajectory.points[0].velocities.resize(2);
		gripperGoal.trajectory.points[0].velocities[0] = 0.0;
		gripperGoal.trajectory.points[0].velocities[1] = 0.0;
		gripperGoal.trajectory.points[0].time_from_start = ros::Duration(0.5);

		// desired initial position
		gripperGoal.trajectory.points[1].positions.resize(2);
		gripperGoal.trajectory.points[1].positions[0] = trajectory[5][0];
		gripperGoal.trajectory.points[1].positions[1] = trajectory[6][0];
		gripperGoal.trajectory.points[1].velocities.resize(2);
		gripperGoal.trajectory.points[1].velocities[0] = 0.0;
		gripperGoal.trajectory.points[1].velocities[1] = 0.0;
		gripperGoal.trajectory.points[1].time_from_start = ros::Duration(3.0);
	}

	moveGoal.jointGoal.position.resize(armJointCount);
	for (unsigned int jointIndex = 0; jointIndex < armJointCount; ++jointIndex) {
		moveGoal.jointGoal.position[jointIndex] = trajectory[jointIndex][0];
	}

	ROS_INFO("Moving arm to initial position.");
	moveClient.sendGoal(moveGoal);

	while(!moveClient.getState().isDone() && ros::ok())
	{
		ros::Duration(0.001).sleep();
		ros::spinOnce();
	}

	if(moveClient.getState().state_ == moveClient.getState().SUCCEEDED)
	{
		ROS_INFO("Arm movement finished successfully.");
	}
	else
	{
		ROS_WARN("Arm movement finished not successfully! (%s)",
				moveClient.getState().toString().c_str());
	}

	if (inSimulation)
	{
		// when to start the trajectory
		gripperGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.4);
		ROS_INFO("Moving gripper to initial position");
		gripperClient.sendGoal(gripperGoal);

		while(!gripperClient.getState().isDone() && ros::ok())
		{
			ros::Duration(0.001).sleep();
			ros::spinOnce();
		}

		if(gripperClient.getState().state_ == gripperClient.getState().SUCCEEDED)
		{
			ROS_INFO("Gripper trajectory finished successfully.");
		}
		else
		{
			ROS_WARN("Gripper trajectory finished not successfully! (%s)",
					gripperClient.getState().toString().c_str());
		}
	}
}

std::string readTrajectoryFromUser(std::string trajectoryDir)
{
	unsigned int traNumber = -2;
	std::string traNumberStr = "-1";
	std::string trajectoryName = "default_trajectory_name";

	ROS_INFO("Available trajectories:");
	std::vector<std::string> trajectories = getAvailableTrajectories(trajectoryDir);

	for (unsigned int i = 0; i < trajectories.size(); ++i) {
			ROS_INFO_STREAM(i << ") " << trajectories.at(i).c_str());
	}

	ROS_INFO("Which trajectory should be reproduced? (Select by number)");
	getline(std::cin, traNumberStr);

	try
	{
		traNumber = boost::lexical_cast<unsigned int>(traNumberStr);

		if (traNumber < trajectories.size())
		{
			trajectoryName = trajectories.at(traNumber);
		}
		else
		{
			ROS_WARN("Selection out of range!");
			trajectoryName = "out_of_range_selection";
		}
	}
	catch (boost::bad_lexical_cast &)
	{
		ROS_WARN("No valid input!");
		trajectoryName = "no_valid_selection";
	}
	catch (...)
	{
		ROS_ERROR("Unhandled exeption in readTrajectoryFromUser!");
		ROS_BREAK();
	}

	return trajectoryName;
}

void printHelpMessage()
{
	ROS_INFO(
		"Usage: ./lat_reproducer <object tracking topic> <trajectory name> <trajectory dir> <draw graph>"
		" <object shift threshold> <arm controller>"
		);
	ROS_INFO("or roslaunch ros_lfd_lat lat_reproducer [trajectory_name:=<trajectory name>]");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lat_reproducer");

	if(argc != (NO_OF_ARGS + 1))	// +1 because argv[0] is the program name
	{
		ROS_ERROR("Not enough arguments provided!");
		printHelpMessage();

		return EXIT_SUCCESS;
	}

	ROS_INFO("lat_reproducer started");

	lfd lfd;



	// flag that determines whether the program runs on gazebo or not
	bool inSimulation = false;

	unsigned int armJointCount = ARM_JOINT_COUNT_NOT_YET_DEFINED;

	//////////////////////////////////////////////////////////////////////
	// parse arguments
	// use this topic to receive object tracking data
	std::string objectTrackingTopic(argv[OBJECT_TRACKING_TOPIC_ARG_IDX]);

	// specifies the folder where the trajectories are stored (default user_home)
	std::string trajectoryDir(argv[TRAJECOTRY_DIR_ARG_IDX]);

	// reproduce this trajectory
	std::string trajectoryName(argv[TRAJECTORY_NAME_ARG_IDX]);
	if(trajectoryName == USE_USER_SELECT_STRING)
	{
		trajectoryName = readTrajectoryFromUser(trajectoryDir);
	}

	// draw the reproduction diagramm or not
	bool drawGraph = true;
	if(strcmp(argv[DRAW_GRAPH_ARG_IDX], "false"))
	{
		drawGraph = false;
	}

	// threshold an object has to move, so that it is recognized
	double objectShiftThreshold = atof(argv[OBJECT_SHIFT_THRESHOLD_ARG_IDX]);

	// use this namespace for the trajectory action and so on
	std::string armController(argv[ARM_CONTROLLER_ARG_IDX]);

	// finished parsing arguments
	////////////////////////////////////////////////////////////////////////

	ROS_INFO("Selected trajectory: %s (home dir: %s)", trajectoryName.c_str(), getHomeDir().c_str());

	// check if this trajectory exists
	if (lfd.leatra_knows_task("/" + trajectoryName, getHomeDir().c_str()))
	{
		ROS_INFO("Trajectory name known.");

		// Initialize object recognition
		Or_Client objectClient("object_recognition", true);
		ROS_INFO("Waiting for object recognition server");
		objectClient.waitForServer();
		ROS_INFO("Object recognition server ready");

		// get objects
		objectClient.sendGoal(
				object_recognition_msgs::ObjectRecognitionGoal(),
				&objectCallback,
				&activeCb,
				&feedbackCb
		);

		objectClient.waitForResult();

		if(objectClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("Error in object recognition");
			ROS_ERROR("State text: %s",
					objectClient.getState().getText().c_str());

			return EXIT_FAILURE;
		}

		ROS_INFO("Finished object recognition");

		// check if all mandatory objects are there
		bool allItemsThere = false;
		allItemsThere =
			lfd.mandatory_objects(&objects, "/" + trajectoryName, getHomeDir().c_str());

		if (allItemsThere)
		{
			ROS_INFO("Found all mandatory objects.");

			//////////////////////////////////////////////////////////////////////
			// compute the trajecotry
			std::deque< std::deque< double > > reproducedTrajectory;
			reproducedTrajectory = lfd.reproduce(objects, "/" + trajectoryName, getHomeDir().c_str());

			ROS_INFO("Trajectory length: %i", (int)(reproducedTrajectory.size()));
			if(reproducedTrajectory.size() == 0)
			{
				return EXIT_FAILURE;
			}


			/*for (unsigned int i = 0; i < reproducedTrajectory[0].size(); ++i) {
				for (unsigned int j = 0; j < reproducedTrajectory.size(); ++j) {
					std::cout << reproducedTrajectory.at(j).at(i) << "\t";
				}
				std::cout << std::endl;
			}*/

			// now move the arm
			// after the example from http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
			TrajClient gripperClient("katana_arm_controller/gripper_joint_trajectory_action");		//TODO: Katana specific
			TrajClient trajClient("katana_arm_controller/joint_trajectory_action", true);		//TODO: Katana specific

			trajClient.waitForServer();
			inSimulation = gripperClient.waitForServer(ros::Duration(0.01));
			ROS_INFO("Action client ready.");

			pr2_controllers_msgs::JointTrajectoryGoal goal;
			pr2_controllers_msgs::JointTrajectoryGoal gripperGoal;
			ROS_INFO("Created goal");

			goal.trajectory.joint_names = getJointNames(inSimulation);
			goal.trajectory.points.resize(reproducedTrajectory[0].size() /*+ 1*/);

			// if the node runs in Gazebo the gripper has to be controlled
			// with the gripper_joint_trajectory_action
			if(inSimulation)
			{
				ROS_INFO("in simulation");
				gripperGoal.trajectory.joint_names = gripperJointNames;
				gripperGoal.trajectory.points.resize(reproducedTrajectory[0].size()/* + 1*/);

				armJointCount = 5;
			}
			else
			{
				ROS_INFO("On real robot");
				armJointCount = 6;
			}

			ROS_INFO("Copy the joint positions");

			// copy the waypoints
			for (unsigned int pointNo = 0/*1*/; pointNo </*=*/ reproducedTrajectory[0].size(); ++pointNo)
			{
				goal.trajectory.points[pointNo].
					positions.resize(armJointCount);

				goal.trajectory.points[pointNo].time_from_start =
						ros::Duration(1.0 / REPRODUCE_HZ * pointNo + (TIME_FROM_START));

				for (unsigned int jointNo = 0; jointNo < armJointCount; ++jointNo)
				{
					goal.trajectory.points[pointNo].positions[jointNo] =
						reproducedTrajectory.at(jointNo).at(pointNo/* - 1*/);
						// the current position is the first element so, every
						// following element is shifted by one
				}

				// gripper
				if(inSimulation)
				{
					gripperGoal.trajectory.points[pointNo].positions.resize(GRIPPER_JOINT_COUNT);
					gripperGoal.trajectory.points[pointNo].velocities.resize(GRIPPER_JOINT_COUNT);

					gripperGoal.trajectory.points[pointNo].time_from_start =
						ros::Duration(1.0 / REPRODUCE_HZ * pointNo + (TIME_FROM_START));

					for (unsigned int jointNo = 0; jointNo < GRIPPER_JOINT_COUNT; ++jointNo)
					{
						gripperGoal.trajectory.points[pointNo].positions[jointNo] =
							reproducedTrajectory.at(jointNo + 5).at(pointNo/* - 1*/);		// gripper joints are number 5 and 6
							// the current position is the first element so, every
							// following element is shifted by one

						// velocity not needed, so set it to zero
						gripperGoal.trajectory.points[pointNo].velocities[jointNo] = 0.0;
					}
				}
			}

			// When to start the trajectory: 0.5s from now
			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
			gripperGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

			if(!trajClient.isServerConnected())
			{
				ROS_ERROR("Action Server is not connected");
			}

			// before starting move the robot to the first position of the trajectory
			moveRobotToStartPos(reproducedTrajectory, inSimulation);

			// Finally start the trajectory!!!!!
			ROS_INFO("Starting now with the trajectory.");
			trajClient.sendGoal(goal);

			if(inSimulation)
			{
				gripperClient.sendGoal(gripperGoal);
			}

			while(!trajClient.getState().isDone() && ros::ok())
			{
				ros::Duration(0.001).sleep();
				ros::spinOnce();
			}

			if (inSimulation)
			{
				while(!gripperClient.getState().isDone() && ros::ok())
				{
					ros::Duration(0.001).sleep();
					ros::spinOnce();
				}

				if(gripperClient.getState().state_ == gripperClient.getState().SUCCEEDED)
				{
					ROS_INFO("Gripper trajectory finished successfully.");
				}
				else
				{
					ROS_WARN("Gripper trajectory finished not successfully! (%s)",
							gripperClient.getState().toString().c_str());
				}
			}

			if(trajClient.getState().state_ == trajClient.getState().SUCCEEDED)
			{
				ROS_INFO("Arm trajectory finished successfully.");
			}
			else
			{
				ROS_WARN("Arm trajectory finished not successfully! (%s)",
						trajClient.getState().toString().c_str());
			}

		}
		else
		{
			ROS_WARN("Mandatory objects missing!");
		}
	}
	else
	{
		ROS_WARN("Trajectory name unknown!");
	}
}

