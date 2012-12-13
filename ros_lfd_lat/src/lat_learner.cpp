#include "lat_learner.h"

std::string trajectoryName = "default_trajectory_name";
std::string demoName = "default_demo_name";
std::string dirName = "default_dir_name";
bool active = false;
int demoCount = 0;
ndmap map;
trajectory_lat trajectory;


void trajectoryCallback(const sensor_msgs::JointStateConstPtr& jointState)
{
	if(active)
	{
		int dof = jointState->position.size();

		// check if all deques have already been created
		if (dof == map.get_dim())
		{
			for (int i = 0; i < dof; ++i)
			{
				map.get_row(i).push_back(jointState->position[i]);
			}
		}
		else if (dof > map.get_dim())
		{
			while (dof > map.get_dim())
			{
				map.add_deque(std::deque<double>());
			}
		}
		else
		{
			ROS_ERROR("size of map is bigger than the dof of the robot!");
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
			1000,
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

	// TODO here a lfd object and a deque of trajectories has to be created
	trajectory = trajectory_lat();
	trajectory.set_name(trajectoryName);

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
		map.set_name(demoName);

		// get objects
		objectClient.sendGoal(
				object_recognition_msgs::ObjectRecognitionGoal(),
				&objectCallback,
				&activeCb,
				&feedbackCb
		);

		if(objectClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("error in object recognition");
			ROS_ERROR("state text: %s", objectClient.getState().getText().c_str());

			return 1;
		}

		ROS_INFO("finished object recognition");
		ROS_INFO("please press enter when you are ready to demonstrate the task");

		std::string tmp;
		getline(std::cin, tmp);

		ROS_INFO("now begin the demonstration");
		std_srvs::Empty srv;
		motorsOff.call(srv);

		active = true;

		ROS_INFO("press enter to end the demonstration");

		getline(std::cin, tmp);

		active = false;
		motorsOn.call(srv);

		// TODO add trajectory to deque
		trajectory.set_ndmap(map);
	}

	// TODO call save_demo

	return 0;
}
