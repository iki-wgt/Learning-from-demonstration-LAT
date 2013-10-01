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

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include "ros_lfd_lat/LatConstants.h"

boost::filesystem::path getHomeDir();

/**
 * Checks if the object is reachable by the arm.
 *
 * @param objectLocation the position of the arm in the base frame from the arm.
 * @return true if reachable, false otherwise
 */
bool isObjectReachable(const geometry_msgs::PointStamped& objectLocation);

/**
 * Get current date/time, format is YYYY-MM-DD.HH:mm:ss
 *
 * Found here: http://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
 *
 * @return the current time and date
 */
const std::string currentDateTime();

#endif /* HELPERS_H_ */
