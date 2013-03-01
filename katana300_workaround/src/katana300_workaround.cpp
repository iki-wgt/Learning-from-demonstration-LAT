/*
 * katana300_workaround.cpp
 *
 *  Created on: 26.02.2013
 *      Author: Benjamin Reiner
 */

#include "ros/ros.h"
#include "kniBase.h"

void convertJointRadToEnc(const std::vector<double>& angles, std::vector<int>& encoders)
{
	// make the encoder vector the same size as the angels vector
	encoders.resize(angles.size());

	CMotBase* mArray =
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "katana300_workaround");
	ros::NodeHandle n;

	return 0;
}


