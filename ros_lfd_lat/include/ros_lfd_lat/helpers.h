/*
 * helpers.h
 *
 *  Created on: 15.05.2013
 *      Author: Benjamin Reiner
 */

#ifndef HELPERS_H_
#define HELPERS_H_

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <boost/filesystem.hpp>

#include "ros_lfd_lat/LatConstants.h"

boost::filesystem::path getHomeDir()
{
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;

	boost::filesystem::path homeDir(homedir);

	return homeDir;
}

/**
 * Checks if the object is reachable by the arm.
 *
 * @param objectLocation the position of the arm in the base frame from the arm.
 * @return true if reachable, false otherwise
 */
bool isObjectReachable(const geometry_msgs::PointStamped& objectLocation)
{	// TODO use something like inverse kinematics to do this
	bool reachable = true;
	const double Z_OFFSET = 0.22;		// katana specific
	const double Z_MINIMUM = -0.3;

	if(objectLocation.header.frame_id != OBJECT_TARGET_FRAME)
	{
		ROS_WARN("objects in isObjectReachable should be in OBJECT_TARGET_FRAME!");
	}

	double distance = sqrt(
			pow(objectLocation.point.x, 2)
			+ pow(objectLocation.point.y, 2)
			+ pow(objectLocation.point.z - Z_OFFSET, 2)
			);

	if(distance > ARM_RANGE || objectLocation.point.z < Z_MINIMUM)
	{
		reachable = false;
	}

	return reachable;
}

#endif /* HELPERS_H_ */
