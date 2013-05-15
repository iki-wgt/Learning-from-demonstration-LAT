#ifndef LITERA_H
#define LITERA_H

#include <iostream>
#include <string>
#include <sstream>
#include <iterator>
#include <deque>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "leatra.hh"
#include "stringhelp.hh"


/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class litera provides the interface to textfiles needed, in order to write and read data (ndmaps).
 */
class litera{

private:
  int write_ndmap(ndmap map, std::string subfolder, std::string name);
  int rules_add_set(ndmapSet set, std::string rulefile);
  int write_ndmapSet(ndmapSet set, std::string subfolder);
  std::deque< double > parse_coordinates(std::string line);
  bool rules_add_tra(std::string name, std::deque< object > obj, std::string rule_file, std::string _path = ""); 

public:
  bool write_trajectories(std::deque< trajectory_lat > tra, std::string rule_file, std::string path);
  std::deque< trajectory_lat > read_trajectories(std::string rule_file, std::string path = "");
  std::deque< object > read_objects(std::string object_file);  
    
  ndmap read_ndmap(std::string subfolder, std::string f_name);
  ndmapSet read_set(std::string rule_file);
  ndmapSetGroup read_group(std::string rule_file);
  
  int write_ndmap(ndmap map);
  int write_ndmapSet(ndmapSet set);
  int write_ndmapSetGroup(ndmapSetGroup group);
  
  int write_all(ndmapSetGroup group, std::string path);
  bool folder_or_file_exist(std::string path, std::string file = "");
  
};


#endif