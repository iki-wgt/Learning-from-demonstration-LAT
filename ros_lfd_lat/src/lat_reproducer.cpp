/*
 * lat_reproducer.cpp
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */
#include "lat_reproducer.h"

std::deque<object> objects;

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

void activeCb() {
	ROS_INFO("Object recognition goal just went active");
}

void feedbackCb(
		const object_recognition_msgs::ObjectRecognitionFeedbackConstPtr& feedback)
{

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
		objectClient.waitForServer();
		ROS_INFO("object recognition server ready");

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

