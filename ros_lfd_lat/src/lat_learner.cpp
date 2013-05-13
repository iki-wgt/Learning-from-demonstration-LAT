#include "lat_learner.h"

std::string trajectoryName = "default_trajectory_name";
std::string demoName = "default_demo_name";
std::string dirName = "default_dir_name";
bool active = false;
bool continueProgramm = false;
int demoCount = 0;
ndmap map;
trajectory_lat trajectory;

void waitForEnter()
{
	continueProgramm = false;
	std::string tmp;
	getline(std::cin, tmp);

	continueProgramm = true;
}

void trajectoryCallback(const sensor_msgs::JointStateConstPtr& jointState)
{
	if(active)
	{
		int dof = jointState->position.size();
		// check if all deques have already been created
		if (dof == map.get_dim())
		{
			std::deque<double> point;
			for (int i = 0; i < dof; ++i)
			{
				//map.get_row(i).push_back(jointState->position[i]);	// does not work
				point.push_back(jointState->position[i]);
			}
			map.push_back(point);
		}
		else if (dof > map.get_dim())	// not all deque have been created
		{
			while (dof > map.get_dim())
			{
				ROS_DEBUG("Adding deque to map");
				map.add_deque(std::deque<double>());
			}
		}
		else
		{
			ROS_ERROR("Size of map is bigger than the dof of the robot!");
		}

	}
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

		trajectory.add_object(obj);

		ROS_INFO("Added object %s to trajectory.", obj.get_name().c_str());
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
	ros::init(argc, argv, "lat_learner");
	ros::NodeHandle node;

	ROS_INFO("lat_learner started");

	// Initialize object recognition
	Or_Client objectClient("object_recognition", true);
	objectClient.waitForServer();
	ROS_INFO("object recognition server ready");

	// initialize joint state listener
	ros::Subscriber jointStateListener = node.subscribe("joint_states",
			1,
			trajectoryCallback);

	// initialize service client to switch the motors on and off
	// TODO: might be katana specific
	ros::ServiceClient motorsOn =
			node.serviceClient<std_srvs::Empty>("switch_motors_on");
	ros::ServiceClient motorsOff =
			node.serviceClient<std_srvs::Empty>("switch_motors_off");

	ROS_INFO("Please enter a name for the trajectory:");
	getline(std::cin, trajectoryName);
	ROS_INFO("Selected name for trajectory: %s", trajectoryName.c_str());

	lfd lfd;
	std::deque<trajectory_lat> trajectories;

	bool finished = false;
	while(ros::ok() && !finished)
	{
		demoCount++;
		map = ndmap();

		ROS_INFO(
			"Please enter a name for this demo (left empty for default name)"
			);
		getline(std::cin, demoName);

		if(demoName == "" || demoName == "default_demo_name")
		{
			std::stringstream dName;
			dName << "demo" << demoCount;
			demoName = dName.str();
		}

		ROS_INFO("Selected name for the demo: %s", demoName.c_str());
		trajectory = trajectory_lat();
		trajectory.set_name(demoName);
		map.set_name(demoName);

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
			ROS_ERROR("State text: %s", objectClient.getState().getText().c_str());

			return 1;
		}

		ROS_INFO("Finished object recognition");
		ROS_INFO("Please press enter when you are ready to demonstrate the task");

		std::string tmp;
		getline(std::cin, tmp);

		ROS_INFO("Now begin the demonstration.");
		std_srvs::Empty srv;
		motorsOff.call(srv);

		active = true;

		ROS_INFO("Press enter to end the demonstration.");

		//getline(std::cin, tmp);	does not work blocks the callbacks!
		continueProgramm = false;
		boost::thread waitThread = boost::thread(waitForEnter);
		ros::Rate rate(20);
		while (!continueProgramm && ros::ok())
		{
			ros::spinOnce();		// pumps the callbacks
			rate.sleep();
		}
		waitThread.join();


		active = false;
		motorsOn.call(srv);

		/*for (int i = 0; i < map.get_dim(); ++i) {
			std::deque<double> row = map.get_row(i);

			for (unsigned int j = 0; j < row.size(); ++j) {
				ROS_INFO("row: %i, column: %i, value %f", i, j, row.at(j));
			}
		}*/

		trajectory.set_ndmap(map);
		trajectories.push_back(trajectory);

		ROS_INFO("Do you want to do another demonstration? (y/n)");
		getline(std::cin, tmp);

		if (tmp == "n" || tmp == "no")
		{
			finished = true;
		}
	}

	bool success = lfd.save_demo(trajectories, trajectoryName);

	if (success) {
		ROS_INFO("Demonstration saved successfully.");
	} else {
		ROS_ERROR("Could not save demonstration!");
	}

	return 0;
}
