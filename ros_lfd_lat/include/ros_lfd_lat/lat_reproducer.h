/*
 * lat_reproducer.h
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */

#ifndef LAT_REPRODUCER_H_
#define LAT_REPRODUCER_H_

#define BOOST_FILESYSTEM_VERSION 3

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_recognition_msgs/ObjectId.h"
#include "object_recognition_msgs/RecognizedObject.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "object_recognition_msgs/ObjectRecognitionAction.h"
#include "std_srvs/Empty.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"


#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <deque>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>



#include "../leatra/lfd.hh"
#include "../leatra/stringhelp.hh"
#include "ros_lfd_lat/helpers.h"

typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> Or_Client;
typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

// the joint_state topic publishes at 25 Hz
#define RECORDING_HZ		25.0
#define TIME_FROM_START		3
#define GRIPPER_JOINT_COUNT	2

std::vector<std::string> getAvailableTrajectories();

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
 * Returns a vector with all the joint names of the robot.
 * Subscribes shortly for the /joint_state topic.
 *
 * @param node Current node
 * @return vector with the joint names
 */
std::vector<std::string> getJointNames(bool inSimulation);

/**
 * Callback for the /joint_state topic. Unsubscribes immediately after first call.
 */
void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointState);

int main(int argc, char **argv);

#endif /* LAT_REPRODUCER_H_ */
