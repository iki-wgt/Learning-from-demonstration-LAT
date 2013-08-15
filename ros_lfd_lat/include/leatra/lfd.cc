#include "lfd.hh"
#include "dtw_leatra.hh"
// #include "kinematics.hh"
#include "draw.hh"

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

  for(unsigned int i = 0; i < JS.size(); i++){
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
 *  \brief This method will read the saved trajectories from file, defined by the
 *  "task_name", warp them and create a new trajectory based on the objects
 *  in "obj".
 *
 *  @param obj detected objects
 *  @param task_name the name of the task (the saved trajectories are found in the
 *  folder carrying this name)
 *  @param path directory, where all the tasks are saved.
 *  @param useInterim If this parameter is true the prepared demonstrations
 *  are stored in memory at the first call.
 */
std::deque< std::deque< double > > lfd::reproduce(std::deque< object > obj,
		std::string task_name, std::string path, bool useInterim, bool drawGraph)
{

  approximation apx;
  litera lit;
  std::deque< trajectory_lat > JS;
  ndmapSet mean_JS;
  ndmapSetGroup model;
  
  if(!useInterim || !isInterimPrepared())
  {
	  ROS_INFO("[Leatra reproduce] reading trajectories from the file: %s%s",path.c_str(), task_name.c_str());

	  // Reading trajectories from file
	  JS = lit.read_trajectories( task_name, path );

	  // Shortening all trajectories to the size of the shortest:
	  // Getting the size of the shortest trajectory
	  int tra_length = JS[0].map.map_is_consistent();
	  for(unsigned int i = 0; i < JS.size(); i++){
		if(i == 0)
		  tra_length = JS[i].map.map_is_consistent();
		else if(JS[i].map.map_is_consistent() < tra_length)
		  tra_length = JS[i].map.map_is_consistent();

		if(tra_length < 1){
			ROS_ERROR("[Leatra Error reproduce] not all trajectories are consistent!");
		}
	  }

	  // Sample all trajectories down to the same length 1/20 th of the shortest trajectory
	  for(unsigned int i = 0; i < JS.size(); i++){
		JS[i].map.thinning( tra_length );
	  }


	  //lit.write_trajectories(JS, "JS_no_warp", "DEBUG/");
	  ROS_INFO("[Leatra reproduce] warping the trajectories in joint space.");

	  // Warping trajectory
	  warp_leatra W;
	  JS = W.warp_in_task_space(JS);
	  //lit.write_trajectories(JS, "JS_warp", "DEBUG/");


	  // Sorting objects!
	  for(unsigned int i = 0; i < JS.size(); i++){
		JS[i].set_obj_order(&obj);
	  }



	  // Creating task space trajectories
	  std::deque< trajectory_lat > TS( JS.begin(), JS.end());
	  for(unsigned int i=0; i < TS.size(); i++){
		if(!TS[i].joint_to_task_space())
		{
			ROS_ERROR("joint_to_task_space failed!");
			std::deque< std::deque< double > > emptyLAT;
			return emptyLAT;
		}
	  }

	  //lit.write_trajectories(TS, "TS_no_warp", "DEBUG/");
	  ROS_INFO("[Leatra reproduce] Calculating mean-trajectory in joint space.");
	  // Calculating mean-trajectory in joint space - containing all 6 angles (5 + gripper)
	  ndmapSet set;
	  for(unsigned int i=0; i < JS.size(); i++){
		set.add_ndmap(JS[i].get_ndmap());
	  }
	  mean_JS = apx.standard_deviation(set, true); // having all 6 angles

	  if(drawGraph)
	  {
		  mean_JS.set_name("JointSpace_Mean");
		  lit.write_ndmapSet(mean_JS);
	  }


	  std::cout << "[Leatra reproduce] Calculating mean-trajectory in task space." << std::endl;
	  // Calculating mean-trajectory in task space
	  model = apx.make_model(TS);

	  if(useInterim && !isInterimPrepared())
	  {
		  interimMean_JS = mean_JS;
		  interimModel = model;
	  }
  }
  
  if(useInterim && isInterimPrepared())
  {
	  mean_JS = interimMean_JS;
	  model = interimModel;
  }

  /***************************************************************
   *
   * Here's the place to start looping.
   */

  ROS_INFO("Made model");
  model.add_offset(obj);
  ndmapSet mean_TS = apx.constraint_fusion( model, true );

  ROS_INFO("After constraint fusion");

  if(drawGraph)
  {
	  ROS_INFO("Drawing graph is enabled.");

	  mean_TS.set_name("TaskSpaceData");
	  lit.write_ndmapSet(mean_TS);

	  ndmapSetGroup model2 = model;
	  model2.add_ndmapSet( mean_TS );
	  model2.set_name("COKE");
	  draw graph;
	  int drawReturn = graph.graph_std(model2, path);
	  if(drawReturn != 0)
		  ROS_WARN("Plotting graphs unsuccessful");
  }


  std::cout << "[Leatra reproduce] joining trajectories from task space and joint space." << std::endl;
  
  // Creating new trajectory from mean_TS (in task space) and mean_JS (trajectory in joint space) according to Calinon:
  std::deque< double > theta_1;
  std::deque< std::deque< double > > LAT(7, theta_1);	//TODO: Katana specific!
  std::deque< std::deque<double> >*  tm = mean_TS.data_pointer(1);
  std::deque< std::deque<double> >*  ts = mean_TS.data_pointer(3);
  std::deque< std::deque<double> >*  jm = mean_JS.data_pointer(1);
  std::deque< std::deque<double> >*  js = mean_JS.data_pointer(3);

  ROS_INFO("Will now optimze TJ");
  if(!optimize_TJ(&LAT, tm, ts, jm, js))
  {
	  ROS_ERROR("Optimizing TJ failed!");
	  std::deque< std::deque< double > > emptyLAT;
	  return emptyLAT;
  }

  return LAT;
}

bool lfd::isInterimPrepared()
{
	return ((interimMean_JS.get_dim() > 0) && (interimModel.get_dim() > 0));
}
