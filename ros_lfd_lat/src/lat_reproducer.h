/*
 * lat_reproducer.h
 *
 *  Created on: 07.01.2013
 *      Author: Benjamin Reiner
 */

#ifndef LAT_REPRODUCER_H_
#define LAT_REPRODUCER_H_

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
#include <iostream>
#include <sstream>
#include <deque>

#include "leatra/lfd.hh"
#include "leatra/stringhelp.hh"

typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> Or_Client;

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

int main(int argc, char **argv);

#endif /* LAT_REPRODUCER_H_ */
