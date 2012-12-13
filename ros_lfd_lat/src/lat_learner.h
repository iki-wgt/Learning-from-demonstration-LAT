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
#include <iostream>
#include <sstream>
#include <deque>

#include "leatra/lfd.hh"
#include "leatra/stringhelp.hh"

typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> Or_Client;


#endif
