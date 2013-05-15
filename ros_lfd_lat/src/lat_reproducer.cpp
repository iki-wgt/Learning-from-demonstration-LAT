/*
 * lat_reproducer.cpp
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */
#include "lat_reproducer.h"

std::deque<object> objects;
std::vector<std::string> jointNames;
std::vector<std::string> gripperJointNames;
std::vector<double> jointPositions;
std::vector<double> gripperJointPositions;


std::vector<std::string> getAvailableTrajectories()
{
	std::string filename;

	std::vector<std::string> trajectories;
	std::string postfix = ".tra";	//every trajectory directory ends with .tra

	boost::filesystem::path homeDir = getHomeDir();

	boost::filesystem::directory_iterator endIter;

	for (boost::filesystem::directory_iterator dirIter(homeDir); dirIter != endIter; ++dirIter)
	{
		if(boost::filesystem::is_directory(dirIter->path()))
		{
			if(boost::algorithm::ends_with(
					dirIter->path().filename().c_str(),
					postfix))
			{
				filename = dirIter->path().filename().c_str();
				boost::algorithm::erase_tail(filename, postfix.size());
				trajectories.push_back(filename);
			}
		}
	}

	return trajectories;
}

void objectCallback(const actionlib::SimpleClientGoalState& state,
		const object_recognition_msgs::ObjectRecognitionResultConstPtr& result)
{
	int objCount = result->recognized_objects.objects.size();
	for (int i = 0; i < objCount; ++i) {
		object obj = object();

		obj.set_name(result->recognized_objects.objects[i].id.id);
		obj.add_coordinate(
			result->recognized_objects.objects[i].pose.pose.pose.position.x);
		obj.add_coordinate(
			result->recognized_objects.objects[i].pose.pose.pose.position.y);
		obj.add_coordinate(
			result->recognized_objects.objects[i].pose.pose.pose.position.z);

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

std::vector<std::string> getJointNames(ros::NodeHandle& node)
{
	// initialize joint state listener
	ros::NodeHandle jointNode;
	ros::Subscriber jointStateListener = jointNode.subscribe("joint_states",
		1,
		jointStateCallback);
	ROS_INFO("subscribed");
	while(jointNames.empty())
	{
		ros::Duration(0.0001).sleep();
		ros::spinOnce();
	}

	// copy the gripper names and joitn positions
	//gripperJointNames.push_back(jointNames.at(jointNames.size() - 2));
	//gripperJointNames.push_back(jointNames.at(jointNames.size() - 1));

	//gripperJointPositions.push_back(
	//	jointPositions.at(jointPositions.size() - 2)
	//	);

	//gripperJointPositions.push_back(
	//	jointPositions.at(jointPositions.size() - 1)
	//	);

	//jointNames.pop_back();
	jointNames.pop_back();

	//jointPositions.pop_back();
	jointPositions.pop_back();

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lat_reproducer");
	ros::NodeHandle node;

	ROS_INFO("lat_reproducer started");

	lfd lfd;
	unsigned int traNumber = -2;
	std::string traNumberStr = "-1";
	std::string trajectoryName = "default_trajectory_name";

	if(argc != 2)
	{
		ROS_INFO("Available trajectories:");
		std::vector<std::string> trajectories = getAvailableTrajectories();

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
			trajectoryName = "not_valid_selection";
		}

	}
	else
	{
		trajectoryName = argv[1];
	}

	ROS_INFO("Selected trajectory: %s", trajectoryName.c_str());

	// check if this trajectory exists
	if (lfd.leatra_knows_task(trajectoryName, "/home/benny/"))
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

			return 1;
		}

		ROS_INFO("Finished object recognition");

		// check if all mandatory objects are there
		bool allItemsThere = false;
		allItemsThere =
			lfd.mandatory_objects(&objects, trajectoryName, "/home/benny/");

		if (allItemsThere)
		{
			ROS_INFO("Found all mandatory objects.");

			//////////////////////////////////////////////////////////////////////
			// compute the trajecotry
			std::deque< std::deque< double > > reproducedTrajectory;
			reproducedTrajectory = lfd.reproduce(objects, trajectoryName, "/home/benny/");

			ROS_INFO("Trajectory length: %i", (int)(reproducedTrajectory.size()));
			if(reproducedTrajectory.size() == 0)
			{
				return 1;
			}


			/*for (unsigned int i = 0; i < reproducedTrajectory[0].size(); ++i) {
				for (unsigned int j = 0; j < reproducedTrajectory.size(); ++j) {
					std::cout << reproducedTrajectory.at(j).at(i) << "\t";
				}
				std::cout << std::endl;
			}*/

			ROS_INFO("Create action client");
			// now move the arm
			// after the example from http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
			TrajClient trajClient("katana_arm_controller/joint_trajectory_action", true);		//TODO: Katana specific
			//TrajClient gripperClient("katana_arm_controller/gripper_joint_trajectory_action");		//TODO: Katana specific
			trajClient.waitForServer();
			ROS_INFO("Action client ready.");

			pr2_controllers_msgs::JointTrajectoryGoal goal;
			pr2_controllers_msgs::JointTrajectoryGoal gripperGoal;
			ROS_INFO("Created goal");

			goal.trajectory.joint_names = getJointNames(node);
			gripperGoal.trajectory.joint_names = gripperJointNames;

			goal.trajectory.points.resize(reproducedTrajectory[0].size() + 1);
			gripperGoal.trajectory.points.resize(reproducedTrajectory[0].size() + 1);

			ROS_INFO("Copy the joint positions");

			// the first waypoint has to be the current joint state
			// otherwise the joint trajectory action fails
			goal.trajectory.points[0].positions.resize(reproducedTrajectory.size() - 1);
			goal.trajectory.points[0].velocities.resize(reproducedTrajectory.size() - 1);
			goal.trajectory.points[0].time_from_start = ros::Duration(3);

			for (unsigned int j = 0; j < reproducedTrajectory.size() - 1; ++j)
			{
				goal.trajectory.points[0].positions[j] =
					jointPositions[j];

				// velocity not needed, so set it to zero
				goal.trajectory.points[0].velocities[j] = 0.0;
			}

			gripperGoal.trajectory.points[0].positions.resize(reproducedTrajectory.size());
			gripperGoal.trajectory.points[0].velocities.resize(reproducedTrajectory.size() - 2);
			gripperGoal.trajectory.points[0].time_from_start = ros::Duration(3);

			/*for (unsigned int j = 0; j < 2; ++j)
			{
				gripperGoal.trajectory.points[0].positions[j] =
					gripperJointPositions[j];

				// velocity not needed, so set it to zero
				gripperGoal.trajectory.points[0].velocities[j] = 0.0;
			}*/

			// copy the other waypoints
			for (unsigned int i = 1; i <= reproducedTrajectory[0].size(); ++i) {
				goal.trajectory.points[i].
					positions.resize(reproducedTrajectory.size() - 1);

				goal.trajectory.points[i].
					velocities.resize(reproducedTrajectory.size() - 1);

				// right would be  1.0 / but then gazebo destroys the katana
				goal.trajectory.points[i].time_from_start =
						ros::Duration(8.0 / RECORDING_HZ * i + 6);
				for (unsigned int j = 0; j < reproducedTrajectory.size(); ++j) {
					goal.trajectory.points[i].positions[j] =
						reproducedTrajectory.at(j).at(i - 1);
						// the current position is the first element so, every
						// following element is shifted by one

					// velocity not needed, so set it to zero
					goal.trajectory.points[i].velocities[j] = 0.0;
				}

				// gripper
				gripperGoal.trajectory.points[i].positions.resize(2);

				gripperGoal.trajectory.points[i].velocities.resize(2);

				gripperGoal.trajectory.points[i].time_from_start =
					ros::Duration(3.0 / RECORDING_HZ * i + 6);

				for (unsigned int j = 0; j < 2; ++j) {
					gripperGoal.trajectory.points[i].positions[j] =
						reproducedTrajectory.at(j + 5).at(i - 1);		// gripper joints are number 5 and 6
						// the current position is the first element so, every
						// following element is shifted by one

					// velocity not needed, so set it to zero
					gripperGoal.trajectory.points[i].velocities[j] = 0.0;
				}
			}
			// When to start the trajectory: 0.5s from now
			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
			gripperGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

			if(!trajClient.isServerConnected())
			{
				ROS_ERROR("Action Server is not connected");
			}

			// Finally start the trajectory!!!!!
			ROS_INFO("Starting now with the trajectory.");
			trajClient.sendGoal(goal);
			//gripperClient.sendGoal(gripperGoal);

			while(!(trajClient.getState().isDone()
					/*&& gripperClient.getState().isDone()*/)
					&& ros::ok())
			{
				ros::Duration(0.001).sleep();
				ros::spinOnce();
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

			/*if(gripperClient.getState().state_ == gripperClient.getState().SUCCEEDED)
			{
				ROS_INFO("Gripper trajectory finished successfully.");
			}
			else
			{
				ROS_WARN("Gripper trajectory finished not successfully! (%s)",
						gripperClient.getState().toString().c_str());
			}*/

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

