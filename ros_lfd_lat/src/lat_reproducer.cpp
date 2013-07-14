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

std::vector<std::string> getJointNames(bool inSimulation)
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

	if(inSimulation)
	{
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lat_reproducer");

	ROS_INFO("lat_reproducer started");

	lfd lfd;
	unsigned int traNumber = -2;
	std::string traNumberStr = "-1";
	std::string trajectoryName = "default_trajectory_name";

	// flag that determines whether the program runs on gazebo or not
	bool inSimulation = false;

	unsigned int armJointCount = 999;

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

			return 1;
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
				return 1;
			}


			for (unsigned int i = 0; i < reproducedTrajectory[0].size(); ++i) {
				for (unsigned int j = 0; j < reproducedTrajectory.size(); ++j) {
					std::cout << reproducedTrajectory.at(j).at(i) << "\t";
				}
				std::cout << std::endl;
			}

			ROS_INFO("Create action client");
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
			goal.trajectory.points.resize(reproducedTrajectory[0].size() + 1);

			// if the node runs in Gazebo the gripper has to be controlled
			// with the gripper_joint_trajectory_action
			if(inSimulation)
			{
				ROS_INFO("in simulation");
				gripperGoal.trajectory.joint_names = gripperJointNames;
				gripperGoal.trajectory.points.resize(reproducedTrajectory[0].size() + 1);

				armJointCount = 5;

				gripperGoal.trajectory.points[0].positions.resize(GRIPPER_JOINT_COUNT);
				gripperGoal.trajectory.points[0].velocities.resize(GRIPPER_JOINT_COUNT);
				gripperGoal.trajectory.points[0].time_from_start = ros::Duration(TIME_FROM_START);

				for (unsigned int jointNo = 0; jointNo < GRIPPER_JOINT_COUNT; ++jointNo)
				{
					gripperGoal.trajectory.points[0].positions[jointNo] =
						gripperJointPositions[jointNo];

					// velocity not needed, so set it to zero
					gripperGoal.trajectory.points[0].velocities[jointNo] = 0.0;
				}
			}
			else
			{
				ROS_INFO("On real robot");
				armJointCount = 6;
			}

			ROS_INFO("Copy the joint positions");

			// the first waypoint has to be the current joint state
			// otherwise the joint trajectory action fails
			goal.trajectory.points[0].positions.resize(armJointCount);
			goal.trajectory.points[0].velocities.resize(armJointCount);
			goal.trajectory.points[0].time_from_start = ros::Duration(TIME_FROM_START);

			for (unsigned int jointNo = 0; jointNo < armJointCount; ++jointNo)
			{
				goal.trajectory.points[0].positions[jointNo] =
					jointPositions[jointNo];

				// velocity not needed, so set it to zero
				goal.trajectory.points[0].velocities[jointNo] = 0.0;
			}

			// copy the other waypoints
			for (unsigned int pointNo = 1; pointNo <= reproducedTrajectory[0].size(); ++pointNo)
			{
				goal.trajectory.points[pointNo].
					positions.resize(armJointCount);

				//goal.trajectory.points[pointNo].
					//velocities.resize(armJointCount);

				// right would be  1.0 / but then gazebo destroys the katana
				goal.trajectory.points[pointNo].time_from_start =
						ros::Duration(1.0 / REPRODUCE_HZ * pointNo + (TIME_FROM_START * 2));

				for (unsigned int jointNo = 0; jointNo < armJointCount; ++jointNo)
				{
					goal.trajectory.points[pointNo].positions[jointNo] =
						reproducedTrajectory.at(jointNo).at(pointNo - 1);
						// the current position is the first element so, every
						// following element is shifted by one

					// velocity not needed, so set it to zero
					//goal.trajectory.points[pointNo].velocities[jointNo] = 0.0;
					//double positionDelta = goal.trajectory.points[pointNo].positions[jointNo]
					      //                 - goal.trajectory.points[pointNo - 1].positions[jointNo];
					//double timeDelta = goal.trajectory.points[pointNo].time_from_start.toSec()
						//	 - goal.trajectory.points[pointNo - 1].time_from_start.toSec();

					//goal.trajectory.points[pointNo].velocities[jointNo] = 0.0;
							//positionDelta / timeDelta; // position difference divided by the time

					//ROS_INFO("Velocity: %f joint %d, point %d", goal.trajectory.points[pointNo].velocities[jointNo], jointNo, pointNo);
				}

				// gripper
				if(inSimulation)
				{
					gripperGoal.trajectory.points[pointNo].positions.resize(GRIPPER_JOINT_COUNT);
					gripperGoal.trajectory.points[pointNo].velocities.resize(GRIPPER_JOINT_COUNT);

					gripperGoal.trajectory.points[pointNo].time_from_start =
						ros::Duration(1.0 / REPRODUCE_HZ * pointNo + (TIME_FROM_START * 2));

					for (unsigned int jointNo = 0; jointNo < GRIPPER_JOINT_COUNT; ++jointNo)
					{
						gripperGoal.trajectory.points[pointNo].positions[jointNo] =
							reproducedTrajectory.at(jointNo + 5).at(pointNo - 1);		// gripper joints are number 5 and 6
							// the current position is the first element so, every
							// following element is shifted by one

						// velocity not needed, so set it to zero
						gripperGoal.trajectory.points[pointNo].velocities[jointNo] = 0.0;
					}
				}
			}

			/*for (unsigned int jointNo = 0; jointNo < armJointCount; ++jointNo)
			{
				goal.trajectory.points[reproducedTrajectory[0].size()].velocities[jointNo] = 0.0;
			}*/

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

