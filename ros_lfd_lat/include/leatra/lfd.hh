#ifndef LFD_H
#define LFD_H

#include <deque>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "leatra.hh"
#include "litera.hh"

#include "ros_lfd_lat/LatConstants.h"
#include "ros_lfd_lat/helpers.h"

class lfd{

private:
	/**
	 * \brief Stores the model.
	 *
	 * The model is stored at the first call of lfd::reproduce with useInterim = true.
	 */
	ndmapSetGroup interimModel;

	/**
	 * \brief Stores the averaged joint space trajectories and the stddev.
	 *
	 * The mean_JS is stored at the first call of lfd::reproduce with useInterim = true.
	 */
	ndmapSet interimMean_JS;

	/**
	 * \brief Returns true if the interim data is filled.
	 */
	bool isInterimPrepared();

public:
    
  bool save_demo(std::deque< trajectory_lat > trajectories,
		  std::string _task_name, std::string path = "");

  std::deque< std::deque< double > > reproduce(std::deque< object > obj,
		  std::string _task_name, std::deque<int>& constraints, std::string path = "", bool useInterim = false,
		  bool drawGraph = false);

  bool leatra_knows_task(std::string _task_name, std::string path = "");

  bool mandatory_objects(std::deque< object >* obj, std::string task_name, std::string path = "");
};


#endif
