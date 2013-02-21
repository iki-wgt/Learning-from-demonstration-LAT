/*
 * lat_reproducer.cpp
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */
#include "lat_reproducer.h"

std::deque<object> objects;
std::vector<std::string> jointNames;

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

		ROS_INFO("Added object %s.", obj.get_name().c_str());
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
	ros::Subscriber jointStateListener = node.subscribe("joint_states",
		1000,
		jointStateCallback);
	ROS_INFO("subscribed");
	while(jointNames.empty())
	{
		ros::Duration(0.0001).sleep();
		ros::spinOnce();
	}

	jointNames.pop_back();
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
		ROS_INFO("Copied joint names");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lat_reproducer");
	ros::NodeHandle node;

	ROS_INFO("lat_reproducer started");

	lfd lfd;
	std::string trajectoryName = "default_trajectory_name";

	ROS_INFO("Which trajectory should be reproduced?");
	getline(std::cin, trajectoryName);
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

			// compute the trajecotry
			std::deque< std::deque< double > > reproducedTrajectory;
			reproducedTrajectory = lfd.reproduce(objects, trajectoryName, "/home/benny/");

			ROS_INFO("length: %i", (int)(reproducedTrajectory.size()));
			for (unsigned int i = 0; i < reproducedTrajectory[0].size(); ++i) {
				for (unsigned int j = 0; j < reproducedTrajectory.size(); ++j) {
					std::cout << reproducedTrajectory.at(j).at(i) << "\t";
				}
				std::cout << std::endl;
			}

			ROS_INFO("Create action client");
			// now move the arm
			// after the example from http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
			TrajClient trajClient("katana_arm_controller/joint_trajectory_action");		//TODO: Katana specific
			pr2_controllers_msgs::JointTrajectoryGoal goal;
			ROS_INFO("Created goal");

			goal.trajectory.joint_names = getJointNames(node);
			ROS_INFO("Copied joint names");
			goal.trajectory.points.resize(reproducedTrajectory[0].size());
			ROS_INFO("COpy the joint positions");
			for (unsigned int i = 0; i < reproducedTrajectory[0].size(); ++i) {
				goal.trajectory.points[i].
					positions.resize(reproducedTrajectory.size() - 2);

				goal.trajectory.points[i].
					velocities.resize(reproducedTrajectory.size() - 2);

				// not sure if this time value is correct, but a wrong value should lead to an obvious error
				goal.trajectory.points[i].time_from_start =
						ros::Duration(1.0 / RECORDING_HZ * i);

				for (unsigned int j = 0; j < reproducedTrajectory.size() - 2; ++j) {
					goal.trajectory.points[i].positions[j] =
						reproducedTrajectory.at(j).at(i);

					// velocity not needed, so set it to zero
					goal.trajectory.points[i].velocities[j] = 0.0;
				}
			}
			// When to start the trajectory: 0.5s from now
			goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

			// Finally start the trajectory!!!!!
			ROS_INFO("Starting now with the trajectory.");
			trajClient.sendGoal(goal);
			ROS_INFO("%s", trajClient.getState().getText().c_str());
			while(!trajClient.getState().isDone() && ros::ok())
			{
				ros::Duration(0.0001).sleep();
				ros::spinOnce();
			}
			ROS_INFO("Trajectory finished.");
			ROS_INFO("%s", trajClient.getState().getText().c_str());
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

