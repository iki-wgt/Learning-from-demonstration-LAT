/*
 * LatConstants.h
 *
 *  Created on: 14.08.2013
 *      Author: Benjamin Reiner
 */

#ifndef LATCONSTANTS_H_
#define LATCONSTANTS_H_

// the joint_state topic publishes at 25 Hz
const double RECORDING_HZ = 25.0;
const unsigned int THINNING_FACTOR = 10;
const double REPRODUCE_HZ = RECORDING_HZ / THINNING_FACTOR;
const double TIME_FROM_START = 0;//3;
const unsigned int GRIPPER_JOINT_COUNT = 2;
const std::string OBJECT_TARGET_FRAME = "/katana_base_link";
const std::string USE_USER_HOME_STRING = "user_home";
const std::string USE_USER_SELECT_STRING = "user_select";
const unsigned int ARM_JOINT_COUNT_NOT_YET_DEFINED = 999;

// consts for program argument processing
const unsigned int NO_OF_ARGS						= 6;
const unsigned int OBJECT_TRACKING_TOPIC_ARG_IDX	= 1;
const unsigned int TRAJECTORY_NAME_ARG_IDX		= 2;
const unsigned int TRAJECOTRY_DIR_ARG_IDX			= 3;
const unsigned int DRAW_GRAPH_ARG_IDX				= 4;
const unsigned int OBJECT_SHIFT_THRESHOLD_ARG_IDX	= 5;
const unsigned int ARM_CONTROLLER_ARG_IDX			= 6;

const unsigned int NO_OF_OBJECTS = 2;
const unsigned int IKEA_CUP_SOLBRAEND_BLUE_ID = 1;
const unsigned int COCA_COLA_CAN_250ML_ID = 5;
const std::string OBJECT_NAMES[] = {"unused!!!",
		"IKEA-CUP-SOLBRAEND-BLUE",
		"unused!!!",
		"unused!!!",
		"unused!!!",
		"COCA-COLA-CAN-250ML"};

#endif /* LATCONSTANTS_H_ */
