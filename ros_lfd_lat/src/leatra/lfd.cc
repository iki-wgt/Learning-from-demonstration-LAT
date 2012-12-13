#include "lfd.hh"
#include "dtw_leatra.hh"
// #include "kinematics.hh"
#include "draw.hh"
#include <time.h>

/**
 *  Check, if task specified by the string _task_name at the directory "path" exists.
 *  If it exists, the return value is true.
 */
bool lfd::leatra_knows_task(std::string _task_name, std::string path){
  litera lit;
  return lit.folder_or_file_exist(path + _task_name + ".tra");
}

/**
 *  Test, if all by the task required objects are also present in the parameter list _obj.
 *  The task is specified by the variable task_name at the directory "path".
 */
bool lfd::mandatory_objects(std::deque< object >* _obj, std::string task_name, std::string path){

  litera lit;
  
  // Reading trajectories from file
  std::deque< trajectory_lat > JS = lit.read_trajectories( task_name, path );

  for(int i = 0; i < JS.size(); i++){
    if(! JS[i].set_obj_order(_obj)) return false; // not all necessary objects are there
  }
  return true;
}

/**
 *  Saves the specified trajectories to file.
 *  The trajectories must contain all angles of the robot.
 *
 *  parameter:
 *      tra: this are the trajectories, that have to be saved.
 *      task_name: name of the task, being expressed in this trajectories
 *      path: directory, where the tasks are stored to ("path" needs to end with a slash).
 */
bool lfd::save_demo(std::deque< trajectory_lat > tra, std::string task_name, std::string path){
    litera lit;
    return lit.write_trajectories(tra, task_name , path);
}

/**
 *  This method will read the saved trajectories from file, defined by the "task_name", warp them and create a new trajectory based on the objects in "obj".
 *
 *  parameters:
 *
 *  obj: detected objects
 *  task_name: the name of the task (the saved trajectories are found in the folder carrying this name)
 *  path: directory, where all the tasks are saved.
 */
std::deque< std::deque< double > > lfd::reproduce(std::deque< object > obj, std::string task_name, std::string path){

  approximation apx;
  litera lit;
  std::deque< trajectory_lat > JS;
  
  // bringing new objects in Katana Coordinate System:
  for(int i = 0; i < obj.size(); i++){
    obj[i].subtract_kate_offset();
  }

  std::cout << "[Leatra reproduce] reading trajectories from the file: " << path << task_name << std::endl;
  
  // Reading trajectories from file
  JS = lit.read_trajectories( task_name, path );

  // bringing objects in trajectories in Katana Coordinate System:
  for(int i = 0; i < JS.size(); i++){
    JS[i].subtract_kate_offset();
  }
  
  // Sorting objects!
  for(int i = 0; i < JS.size(); i++){
    JS[i].set_obj_order(&obj);
  }
  
  // Shortening all trajectories to the size of the shortest:
  // Getting the size of the shortest trajectory
  int tra_length;
  for(int i = 0; i < JS.size(); i++){
    if(i == 0)
      tra_length = JS[i].map.map_is_consistent();
    else if(JS[i].map.map_is_consistent() < tra_length)
      tra_length = JS[i].map.map_is_consistent();
    if(tra_length < 1){
        std::cout<<"[Leatra Error reproduce] not all trajectories are consistent!"<<std::endl;
    }
  }
  
  // Sample all trajectories down to the same length 1/20 th of the shortest trajectory
  for(int i = 0; i < JS.size(); i++){
    JS[i].map.thinning( tra_length );
  }
    
  std::cout << "[Leatra reproduce] warping the trajectories in joint space." << std::endl;
  
  // Warping trajectory
  warp_leatra W;
  JS = W.warp_in_task_space(JS);
  
  //time_t start,end;
  //char szInput [256];
  //double dif;
  //time (&start);
  
  // Creating task space trajectories
  std::deque< trajectory_lat > TS( JS.begin(), JS.end());
  for(int i=0; i < TS.size(); i++){
    TS[i].joint_to_task_space();
  }
  
  std::cout << "[Leatra reproduce] Calculating mean-trajectory in joint space." << std::endl;
  
  // Calculating mean-trajectory in joint space - containing all 6 angles (5 + gripper)
  ndmapSet set;
  for(int i=0; i < JS.size(); i++){
    set.add_ndmap(JS[i].get_ndmap());
  }
  ndmapSet mean_JS = apx.standard_deviation(set, true); // having all 6 angles

  std::cout << "[Leatra reproduce] Calculating mean-trajectory in task space." << std::endl;
  // Calculating mean-trajectory in task space
  ndmapSetGroup model = apx.make_model(TS);
  model.add_offset(obj);
  ndmapSet mean_TS = apx.constraint_fusion( model, true );

  mean_TS.set_name("TaskSpaceData");
  lit.write_ndmapSet(mean_TS);

  std::cout << "[Leatra reproduce] joining trajectories from task space and joint space." << std::endl;
  
  // Creating new trajectory from mean_TS (in task space) and mean_JS (trajectory in joint space) according to Calinon:
  std::deque< double > theta_1;
  std::deque< std::deque< double > > LAT(6, theta_1);

  optimize_TJ(&LAT, mean_TS.data_pointer(1), mean_TS.data_pointer(3), mean_JS.data_pointer(1), mean_JS.data_pointer(3));

  //time (&end);
  //dif = difftime (end,start);
  //std::cout << "It took " << dif << " Seconds" << std::endl; 
  
  return LAT;
}