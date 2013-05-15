#ifndef LFD_H
#define LFD_H

#include <deque>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>


#include "leatra.hh"
#include "litera.hh"


class lfd{

public:
    
  bool save_demo(std::deque< trajectory_lat > trajectories, std::string _task_name, std::string path = "");
  std::deque< std::deque< double > > reproduce(std::deque< object > obj, std::string _task_name, std::string path = "");
  bool leatra_knows_task(std::string _task_name, std::string path = "");
  bool mandatory_objects(std::deque< object >* obj, std::string task_name, std::string path = "");
};


#endif
