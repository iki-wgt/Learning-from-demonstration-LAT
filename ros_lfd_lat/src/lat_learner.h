#ifndef __LAT_LEARNER_H__
#define __LAT_LEARNER_H__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_recognition_msgs/ObjectId.h"
#include "object_recognition_msgs/RecognizedObject.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "object_recognition_msgs/ObjectRecognitionAction.h"
#include "std_srvs/Empty.h"

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <deque>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "leatra/lfd.hh"
#include "leatra/stringhelp.hh"

typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> Or_Client;



std::vector<std::string> getAvailableTrajectories();

/**
 * \brief Callback function for the joint state listener.
 *
 * Pure callback function. Do not call it by yourself.
 * This function only does its stuff, if the active variable is set to true.
 *
 * @param jointState The message containing the joint states.
 */
void trajectoryCallback(const sensor_msgs::JointStateConstPtr& jointState);

/**
 * \brief Callback function for the object recognition.
 *
 * Pure callback function for an actionlib goal. Do not call it by yourself.
 *
 * @param state actionlib goal state
 * @param result Message containing the recognized objects and their positions.
 */
void objectCallback(const actionlib::SimpleClientGoalState& state,
		const object_recognition_msgs::ObjectRecognitionResultConstPtr& result);

/**
 * Callback function for the actionlib event "goal went active".
 */
void activeCb();

/**
 * Callback for the actionlib feedback. Not in use.
 *
 * @param feedback Feedback of the action.
 */
void feedbackCb(
		const object_recognition_msgs::ObjectRecognitionFeedbackConstPtr& feedback);

/**
 * \brief Main routine for learning trough averaging.
 *
 * In this program the user is asked for a name of the trajectory and of each
 * demo. As many demos are recorded as the user wishes.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv);

/**
 * \mainpage ros_lfd_lat
 *
 * This project provides a ROS stack for teaching a robot through averaging trajectories.
 *
 * This project is based on the bachelor-thesis of Heiko Posenauer and his program Leatra.
 *
 * The targeted ROS distribution is Fuerte. The used arms are the Katana and the Powerball.
 *
 * As a dependecy Eigen3 has to be located in /usr/local/include.
 */

#endif
