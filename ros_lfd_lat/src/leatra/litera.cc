#include "litera.hh"

// path needs to end with a slash /

bool litera::write_trajectories(std::deque< trajectory_lat > tra, std::string rule_file, std::string path){
  // Create a folder for the task:  
  if( 0 != system(("mkdir " + path + rule_file + ".tra").c_str())) return false;
    
  for(unsigned int i=0; i < tra.size(); i++){
    
    // Write data to file    
    if(write_ndmap(tra[i].get_ndmap(), path + rule_file + ".tra" + "/", tra[i].get_name()) != 0){ 
      //std::cout<<"Stelle i = "<<i<<" subfolder: "<< subfolder <<" name: "<<set.set[i].get_name()<<"\n";
      return false; // Error while writing
    }

    // Add information to rule file:  
    if(! rules_add_tra(tra[i].get_name() ,tra[i].get_objects(), rule_file, path)) return false;

  }
  return true;
}

bool litera::rules_add_tra(std::string name, std::deque< object > obj, std::string rule_file, std::string _path){ 

  std::string path = _path + rule_file + ".tra" + "/" + rule_file + ".tra";
  std::ofstream wFile;
  wFile.open(path.c_str(), std::ios::app);
  if(wFile.is_open()){
    wFile<< name <<"\n";
    for(unsigned int i=0; i < obj.size(); i++){
      wFile << obj[i].get_name() <<"(";
      for(int j=0; j < obj[i].get_num_of_coordinate(); j++){
        wFile << obj[i].get_coordinate(j);
        if(j == (obj[i].get_num_of_coordinate() - 1)) wFile << ")";
        else wFile << ",";
      }
    }
    wFile<<"\n";
    wFile.close();
    return true;
  }
  return false;
}

bool litera::folder_or_file_exist(std::string path, std::string file){

  struct stat file_statistics;
  if(stat( (path + file).c_str() ,&file_statistics) == 0)
    return true;
  else
    return false;
}



/**
 *      The method write_ndmap() writes a ndmap to a file. 
 *      The name of the map (parameter) is used to name the file, the data is saved in.
 *      The n dimensions are saved in n columns separated with a whitspace.
 *      The subfolder, specified by the parameter "subfolder" has to contain the ending slash (/).
 *      
 *      Return value: 
 *               0: all ok; 
 *              -1: file couldn't be opened; 
 *              -2: the map is not consistent;
 */
int litera::write_ndmap(ndmap map, std::string subfolder, std::string name){
  
  std::string path = subfolder + name;
  std::ofstream wFile((path).c_str());
  
  int el, dim;
  el = map.map_is_consistent();
  dim = map.get_dim();
  
  if(map.map_is_consistent() <= 0)
  {
	  return -2;		//return -2: The map to be written isn't consistent or is empty
  }
  
  if (wFile.is_open()) {

    for(int i=0; i < el; i++){
      for(int j=0; j < dim; j++){
	wFile<<map.map[j][i];
	wFile<<" "; 
      }
      wFile<<"\n";
    }
    wFile.close();
    return 0;
  }
  return -1;			//Error: File couldn't be opened.  
}

/**
 *  The public method write_ndmap, calls the private method and sets the folder name to the map name.
 */
int litera::write_ndmap(ndmap map){
  if(0 != system(("mkdir " + map.get_name()).c_str())) throw data_error(map.get_name(), -41);               //File exists!  
  write_ndmap(map, map.get_name() + "/", map.get_name());

  return 0;		// added because compiler complained BR
}

/**
 *      The method write_ndmapSet saves all ndmaps in data files and adds the set to the rule file (model file) defined by the parameter subfolder.
 */
int litera::write_ndmapSet(ndmapSet set, std::string subfolder){

  for(unsigned int i=0; i < set.set.size(); i++){
    if(write_ndmap(set.set[i], subfolder + ".mod" +"/", set.set[i].get_name()) != 0){ 
        std::cout<<"Stelle i = "<<i<<" subfolder: "<< subfolder <<" name: "<<set.set[i].get_name()<<"\n"; return -1; // Error while writing
    }           
  }
  rules_add_set(set, subfolder);
  return 0;
}

/**
 *      The method write_ndmapSet without the string parameter creates the rule file (model file) regarding the set's name
 *      and calls the related method write_ndmapSet with the string parameter for the subfolder.
 */
int litera::write_ndmapSet(ndmapSet set){
  if( 0 != system(("mkdir " + set.get_name() + ".mod").c_str())) throw data_error(set.get_name(), -41);                //File exists!  
  write_ndmapSet(set, set.get_name());

  return 0;		// added because compiler complained BR
}

/**
 *      The method write_ndmapSetGroup creates a rule file (model file) and saves all related data.
 *      The output folder is determined by the group's name.
 */
int litera::write_ndmapSetGroup(ndmapSetGroup group){
  if( 0 != system(("mkdir " + group.get_name() + ".mod").c_str())) throw data_error(group.get_name(), -41);                 // File exists! 
   
  for(int i=0; i < group.get_num_of_sets(); i++){
    if(write_ndmapSet(group.get_set(i), group.get_name()) != 0) return -1;         // Error while writing
  }
  return 0;
}

/**
 *      The method rules_add_set extends a rule file by one ndmapSet.
 *      The file is defined by the parameter rulefile. 
 *      If the rulefile couldn't be accessed the method returns -1. 
 *      The method is used by the method write_ndmapSet.
 */
int litera::rules_add_set(ndmapSet set, std::string rulefile){
  
  std::string path = rulefile + ".mod" + "/" + rulefile + ".mod";
  std::ofstream wFile;
  wFile.open(path.c_str(), std::ios::app);
  if(wFile.is_open()){
    wFile<< set.get_name() <<"\n";
    for(int i=0; i < set.get_num_of_maps(); i++) wFile << set.set[i].get_name() <<";";
    wFile<<"\n";
    wFile.close();
    return 0;
  }
  return -1;
}

/**
 *      This method writes the data to file differently. This is a helper function for gnuplot, as gnuplot can make 
 *      a figure only from one data file.
 *      For each dimension and each set of the group a datafile is created: 
 *      For example: "d0s1" is the name of the file containing the data from set1, but only for dimension 0.
 *      If the files have been successfully written, the return value is 0. Else the returnvalue of"write_ndmap" is used.
 */
int litera::write_all(ndmapSetGroup group, std::string path){

  int dim = group.get_dim();
  int sets = group.get_num_of_sets();
  int ret = 0;
    
  for(int d = 0; d < dim; d++){
    
    for(int s = 0; s < sets; s++){
      std::string name = "d" + intTOstring(d) + "s" + intTOstring(s);
      ndmap map;
      for(unsigned int i = 0; i < group.group[s].set.size(); i++) map.add_deque(group.group[s].set[i].map[d]);
      map.set_name(name);
      ret = write_ndmap(map, path, name);
    }   
  }
  return ret;
}


/**
 *      This method reads a set of trajectories from file. 
 *      The parameter _path needs to end with a slash: /
 *      The file therefore ends on .tra and needs to have a certain structure:  
 *      The name of the ndmap-file in one line (code: see read_stage = 1) example1.data
 *      and the objects and their coordinates in the next line: (code see read_stage = 2) Cup(2.3, 4.5)Coffee_maker(4.5, 8.8)
 */
std::deque< trajectory_lat > litera::read_trajectories(std::string rule_file, std::string _path){
    
  std::string path = _path + rule_file + ".tra" + "/" + rule_file + ".tra";
  std::deque< trajectory_lat > trajectories;
  
  trajectory_lat tra;
  object obj;
  
  std::string line;
  std::ifstream rFile;
  rFile.open((path).c_str());
  int read_stage = 1;
			// read_stage = 1: read trajectory ndmap.
			// read_stage = 2: read objects.

  if (rFile.is_open()) {

    while (getline(rFile, line,'\n')) { 		// interprete input!
      ndmap map;
      
      switch (read_stage){
        case 1:
          tra.set_name(line);
          tra.set_ndmap(read_ndmap(_path + rule_file + ".tra" , line));
          tra.set_ndmap_name(line);
          read_stage++;
        break;
        case 2:
          read_stage--;
          int pos1 = 0;
          int pos2 = 0;
          while(pos2 < (int)line.size()){
            pos2 = line.find("(", pos1); 
            if(pos2 == -1) pos2 = line.size();
            else {
              obj.set_name(line.substr(pos1,pos2 - pos1));
              pos1 = pos2 + 1;
              pos2 = line.find(")", pos1);
              if(pos2 == -1) pos2 = line.size();
              else{
                obj.set_coordinates(parse_coordinates(line.substr(pos1, pos2 - pos1)));
                tra.add_object(obj);
                pos1 = pos2 + 1;  
              }
            }
          }
          trajectories.push_back(tra);
          tra.clear();
        break;
      }
    }
    rFile.close();

  }
  else throw data_error(path, -21);

  return trajectories;			//Error: File couldn't be opened.     
}

/**
 *      The method parse_coordinates is a helper function for the method read_trajectories.
 *      As a parameter it receives a string of numbers, separated by a comma.
 *      The function reads all the numbers and returns them all in the deque "coord".
 */
std::deque< double > litera::parse_coordinates(std::string line){
  
  std::deque< double > coord;
  line = line + ",";
  int pos1 = 0;
  int pos2 = 0;
 
   while(pos2 < (int)line.size()){
    pos2 = line.find(",", pos1);
    if(pos2 == -1) pos2 = line.size();
    else{
      coord.push_back(stringTOdouble(line.substr(pos1, pos2 - pos1)));
      pos1 = pos2 + 1;
    }
  }
  return coord;   
}

/**
 *      The method read_objects reads objects from the file specified by the parameter.
 *      The directory it reads it from is the directory, the executable is located.
 *      The return value is a standard deque of objects.
 *
 *      The corresponding file needs to be of this kind of structure:
 *      <object name>(<value of coordinate  1>,<value of coordinate 2>, ... , <value of coordinate n>)
 *
 *      If more than one objects need to be read, all objects need to be specified in !one! line, no separator is allowed.
 *      Example file fruits.dat:
 *      Apple(0.2,4.9)Orange(7.1,5.6)Strawberries(1.0,15.2)       
 */
std::deque< object > litera::read_objects(std::string object_file){
    
  std::string path = object_file;
  std::deque< object > objects;
  
  object obj;
  
  std::string line;
  std::ifstream rFile;
  rFile.open((path).c_str());

  if (rFile.is_open()) {

    getline(rFile, line,'\n');  		// interprete input!

    int pos1 = 0;
    int pos2 = 0;
    while(pos2 < (int)line.size()){
      pos2 = line.find("(", pos1); 
      if(pos2 != -1){
        obj.set_name(line.substr(pos1,pos2 - pos1));
        pos1 = pos2 + 1;
        pos2 = line.find(")", pos1);
        if(pos2 != -1){
          obj.set_coordinates(parse_coordinates(line.substr(pos1, pos2 - pos1)));
          objects.push_back(obj);
          pos1 = pos2 + 1;  
        }
      }
    }
    rFile.close();
  }
  else throw data_error(object_file, -21);
  return objects;			//Error: File couldn't be opened.     
}


/**
 *      The method read_ndmap reads a ndmap from the file with the name of the parameter "f_name".
 *      If the reading was successfull, it returns the ndmap.
 *      Otherwise an empty ndmap is returned.
 */
ndmap litera::read_ndmap(std::string subfolder, std::string f_name){

  ndmap m;
  int dim = -1;
  
  m.name = f_name;
  //int ret = -1;
  std::string path = subfolder + "/" + f_name;
  std::string line;
  std::ifstream rFile;
  rFile.open((path).c_str());
  
  if (rFile.is_open()) {
    
    std::deque<double> list;
    std::deque< std::deque<double> > dlist;
    
    while (getline(rFile, line,'\n')) {	// input interpretieren und abspeichern!
     
      if(dim == -1){			// getting the number of dimensions from the first line
        std::istringstream ostr(line);
        std::istream_iterator<double> it(ostr);
        std::istream_iterator<double> end;
	
        size_t words = 0;
        while (it++ != end) words++;
        dim = (int) words;
	
        for(int i=0; i < dim; i++) dlist.push_back(list); // prepare the amount of dimentions
      } 
     
      std::istringstream ostr(line);
      std::istream_iterator<double> it(ostr);
      std::istream_iterator<double> end;
      
      int i = 0;
      while ((it != end) && (i < dim)){

        dlist[i].push_back(*it); 
	
        i++;
        it++;
      }
    }
    for(unsigned int i = 0; i < dlist.size(); i++) m.add_deque( dlist[i] );
    rFile.close();
    return m;
  }
  else throw data_error(f_name, -21);
  
  return m;			//Return empty ndmap -> Error: File couldn't be opened.
}

/**
 *      The method read_set reads a ndmapSet, specified in the rule_file.
 *      On success the read ndmapSet is returned, otherwise the ndmapSet is empty.
 *      The parameter is the name to the file, located at the directory of this program.
 *      The rulefile as this structure:
 *      
 *      name_of_set1
 *      ndmap_file1;ndmap_file2;...
 *      
 *      First the set_name and in the second line the corresponding ndmap file names.
 *      For example "Tasse" is the set names and f1 and f2 are the names that contain the ndmap data:
 *
 *      Tasse
 *      f1;f2;
 *      
 */
ndmapSet litera::read_set(std::string rule_file){
  
  ndmapSetGroup group = read_group(rule_file);
  ndmapSet set;
  if(group.get_num_of_sets() > 0) set = group.get_set(0);
  return set;			                                //If set is emptyError: File couldn't be opened. 
}

/**
 *      The method read_group reads a ndmapSetGroup, specified in the rule_file.
 *      On success the read ndmapSetGroup is returned, otherwise an empty ndmapSetGroup is returned.
 *      The parameter is the name to the file, located at the directory of this program.
 *      The rulefile has this structure:
 *      
 *      name_of_set1
 *      ndmap_file1;ndmap_file2;
 *      name_of_set2
 *      ndmap_file3;ndmap_file4;...
 *      ...
 *      
 *      Alternating: set_name and corresponding ndmap file names.
 *      For example "Tasse" and "Kaffeemaschine" are the set names and f1 - f4 are the names that contain the ndmap data:
 *
 *      Tasse
 *      f1;f2;
 *      Kaffeemaschine
 *      f3;f4;
 *      
 */
ndmapSetGroup litera::read_group(std::string rule_file){
  
  std::string path = rule_file + ".mod" + "/" + rule_file + ".mod";
  ndmapSetGroup group;
  ndmapSet set;
  
  group.set_name(rule_file);
  std::string line;
  std::ifstream rFile;
  rFile.open((path).c_str());
  int read_stage = 1;
			// read_stage = 1: read set name.
			// read_stage = 2: read the ndmaps.
  if (rFile.is_open()) {
    while (getline(rFile, line,'\n')) { 		// interprete input!
      switch (read_stage){
        case 1:
          set.name = line;
          read_stage++;
        break;
        case 2:
          read_stage--;
          int pos1 = 0;
          int pos2 = 0;
          while(pos2 < (int)line.size()){
            pos2 = line.find(";", pos1); 
            if(pos2 == -1) pos2 = line.size();
            else {
              set.add_ndmap(read_ndmap(rule_file + ".mod" +  "/", line.substr(pos1,pos2 - pos1)));
              pos1 = pos2 + 1;
            }
          }
          group.add_ndmapSet(set);
          set.clear();
        break;
      }
    }
    rFile.close();
  }
  else throw data_error(rule_file, -21);              
  
  return group;			//Error: File couldn't be opened. 
}


