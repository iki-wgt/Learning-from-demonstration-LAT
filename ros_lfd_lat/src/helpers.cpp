/*
 * helpers.cpp
 *
 *  Created on: 01.10.2013
 *      Author: benny
 */
#include "ros_lfd_lat/helpers.h"

boost::filesystem::path getHomeDir()
{
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;

	boost::filesystem::path homeDir(homedir);

	return homeDir;
}

bool isObjectReachable(const geometry_msgs::PointStamped& objectLocation)
{	// TODO use something like inverse kinematics to do this
	bool reachable = true;
	const double Z_OFFSET = 0.22;		// katana specific
	const double Z_MINIMUM = -0.35;

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

const std::string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

