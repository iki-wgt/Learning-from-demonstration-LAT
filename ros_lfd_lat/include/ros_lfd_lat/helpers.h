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

boost::filesystem::path getHomeDir()
{
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;

	boost::filesystem::path homeDir(homedir);

	return homeDir;
}


#endif /* HELPERS_H_ */
