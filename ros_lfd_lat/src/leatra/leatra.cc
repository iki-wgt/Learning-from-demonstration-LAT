#include "leatra.hh"
#include <boost/lexical_cast.hpp>

// include these headers to switch from leatra direct kinematics to kdl
#include "ros/ros.h"
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/utilities/utility.h>


/**
 *      The global variables maps_exist, sets_exist and groups_exist are used
 *      for maps, sets and groups. This reduces the risk of repeated equal names -
 *      this could lead to a problem, while saving to file.
 */
int maps_exist;
int sets_exist;
int groups_exist;


/***************************************************************************/

/*                             approximation                                 */

/***************************************************************************/

 /**
  * 	The method mean_value calculates the mean over each element for all the dimensions.
  * 	The parameter set needs to fulfill some criteria: it needs to carry at least two non equal ndmaps.
  *     Those ndmaps have all to be of the same order, viewed as a matrix (see class ndmap and ndmapSet!).
  * 	as a result it returns the mean values as a ndmap.
  */
ndmap approximation::mean_value(ndmapSet set){
  ndmap ret;
  int el = set.set_is_consistent();
  if(el <= 0) return ret;
  int sets = set.set.size();
  int dim = set.set[0].get_dim();

  std::deque<double> x(el,0);					//Preparing the dimensions
  for(int i=0; i < dim; i++){					//of the return ndmap.
    ret.add_deque(x);
  }								//Calculate mean over all ...
  for(int i=0; i < sets; i++){					//...sets
    for(int j=0; j < dim; j++){					//...dimensions
      for(int k=0; k < el; k++){				// and elements.
	ret.map[j][k] = ret.map[j][k] + set.set[i].map[j][k];
      }								//They are simply added all up and ...
    }
  }
  for(int i=0; i < dim; i++){					// ... divided through the
    for(int j=0; j < el; j++){					// number of sets involved.
      ret.map[i][j] = ( ret.map[i][j] / sets );
    }
  }
  return ret;
}

 /**
  * 	The method standard_deviation calculates the mean and the standard deviation of the set (parameter one) at each point.
  *     The ndmapSet returned contains 3 ndmaps:
  *	at position 0 the mean + standard deviation,
  *	at position 1 the mean,
  *	at position 2 the mean - standard deviation.
  *     The second parameter "boold add_s" determines, if at position 3 of the returning ndmapSet the standard deviation is saved.
  */
ndmapSet approximation::standard_deviation(ndmapSet set, bool add_s){
								// The standard deviation is defined as follows:
                                                                // ss = s*s = s^2 = 1/(n-1) Sum: [i=1->n] (xi - x(quer))²
  ndmapSet var;
  int el = set_validation(set);
  if(el <= 0) return var;					// set is inconsistent or has less than 2 ndmaps -> return empty set

  set.correct_nans();

  int num_of_maps = set.get_num_of_maps();
  int dim = set.get_dim();
  ndmap mean;

  mean = mean_value(set); 					// getting all the means over all the ndmaps
  mean.name = set.get_name() + "_mean+stdev";
  var.add_ndmap(mean);						// plus ss
  mean.name = set.get_name() + "_mean";
  var.add_ndmap(mean);						// stays unaltered
  mean.name = set.get_name() + "_mean-stdev";
  var.add_ndmap(mean);						// minus ss
  mean.name = set.get_name() + "_stdev";

  if(add_s) var.add_ndmap(mean);				// will be replaced by ss

  for(int j=0; j < dim; j++){					// for each dimension

    for(int i=0; i < el; i++){

      double sum = 0;
      double ss = 0;

      for(int k=0; k < num_of_maps; k++){			// creating the standard deviation over all data sets of this dimension of this point
	  sum = sum + pow((set.set[k].map[j][i] - mean.map[j][i]),2);
      }
      ss = sqrt( 1.0/ ((double)(num_of_maps - 1.0)) * sum); 	// only if the standard deviation is asked for
   //   ss = ( 1.0/ ((double)(num_of_maps - 1.0)) * sum);		// only if the variance is asked for

      var.set[0].map[j][i] += ss;
      var.set[2].map[j][i] -= ss;
      if(add_s) var.set[3].map[j][i] = ss;
    }
  }
  var.set_name(set.get_name());
  return var;
}

/**
 *  The method set_validation checks if a set is fit for being processed in other functions (standard_deviation()).
 *  It is determined, if the rules for the ndmap (check for consistency) and ndmapSet have been applied and if there are at least 2 ndmaps.
 *
 * 	Return value:	the number of elements, if it is consistent;
 * 			-1 if a ndmap dosen't contain any data [origin: ndmap::map_is_consistent()];
 * 			-2 if a ndmap - matrix is not rectangular (number of elements in each dimension is different) [origin: ndmap::map_is_consistent()];
 *                      -3 if the set is empty [origin: ndmapSet::map_is_consistent()];
 *                      -4 if the ndmaps have a differnt number of elements (matrices have differnent size: collumns) [origin: ndmapSet::map_is_consistent()];
 *                      -5 if the ndmaps have not the same number of dimensions (matrices have different size: lines) [origin: ndmapSet::map_is_consistent()];
 *                      -6 if the underlying sets have different number of elements [origin: ndmapSetGroup::map_is_consistent()];
 *                      -7 if the group is emptly [origin: ndmapSetGroup::map_is_consistent()];
 *                      -8 if the underlying sets have different number of dimensions [origin: ndmapSetGroup::map_is_consistent()];
 *                      -9 if there are less than 2 ndmaps in the set.
 */
int approximation::set_validation(ndmapSet set){
  try{
    int consistent = set.set_is_consistent();
    if(consistent <= 0) return consistent;
    if(set.get_num_of_maps() < 2) throw data_error(set.get_name(), -9);        // There are less than 2 ndmaps!
    return consistent;
  }
  catch(data_error& err){
    return err.print();
  }
  return -42;
}


/**
 *  The method group_validation checks if a group is fit for being processed in other functions (constraint_fusion()).
 *  It is determined, if the rules for the ndmap, ndmapSet and the set_validation have been applied and if there are at least 2 sets.
 *
 * 	Return value:	the number of elements, if it is consistent;
 * 			-1 if a ndmap dosen't contain any data [origin: ndmap::map_is_consistent()];
 * 			-2 if a ndmap - matrix is not rectangular (number of elements in each dimension is different) [origin: ndmap::map_is_consistent()];
 *                      -3 if the set is empty [origin: ndmapSet::map_is_consistent()];
 *                      -4 if the ndmaps have a differnt number of elements (matrices have differnent size: collumns) [origin: ndmapSet::map_is_consistent()];
 *                      -5 if the ndmaps have not the same number of dimensions (matrices have different size: lines) [origin: ndmapSet::map_is_consistent()];
 *                      -6 if the underlying sets have different number of elements [origin: ndmapSetGroup::map_is_consistent()];
 *                      -7 if the group is emptly [origin: ndmapSetGroup::map_is_consistent()];
 *                      -8 if the underlying sets have different number of dimensions [origin: ndmapSetGroup::map_is_consistent()];
 *                      -9 if there are less than 2 ndmaps in the set.
 *                      -10 if there are less than 2 ndmapSets in the group.
 */
int approximation::group_validation(ndmapSetGroup group){
  try{
    int consistent = group.group_is_consistent();
    if(consistent <= 0) return consistent;
    if(group.get_num_of_sets() < 2) throw data_error(group.get_name(), -10);
    for(int i=0; i < group.get_num_of_sets(); i++){
      int set_val = set_validation(group.get_set(i));
      if(set_val <= 0) return set_val;
    }
    return consistent;
  }

  catch(data_error& err){
    return err.print();
  }

  return -42;
}

/**
 *      The method constraint_fusion approximates a trajectory from a group of trajectories.
 *      It accepts as input the Group data type and a smoothing parameter. TODO!!!
 *      The method determines a new ndmap (n dimensional map) and its related trajectories,
 *      once with added standard deviation and once with subtracted standard deviation.
 *      The smoothing parameter, like above, determines, on how much the new function is being straightened.
 *      The method returns a ndmapSet set: set[0] = upper curve, set[1] = trajectory, set[2] = lower curve;
 *      If the second parameter "bool add_s" is set to "true", the returning set will contain at set[3] = standard deviation.
 *      As it is not so frequently needed, the default value is false.
 */
ndmapSet approximation::constraint_fusion(ndmapSetGroup group, bool add_s){
								// The formula for n gaussian distributions:
								// mean: m = sum[i=1,n](m_i/(s_i^2) / sum[i=1,n](1/(s_i^2))
								// s² = 1/( sum[i=1,n](1/(sn²))
  ndmapSet gauss_prod, empty;
  std::deque<ndmapSet> var;
  int dim = 0;
  int el = group_validation(group);
  if(el <= 0) return empty;

  dim = group.get_dim();

  int sets = group.get_num_of_sets();

  std::deque<double> init_deq(el,0);                            // initialisation of the output
  ndmap init_ndmap;						// ndmapSet with 0
  for(int i=0; i < dim; i++) init_ndmap.add_deque(init_deq);	// values
  for(int i=0; i < 3; i++) gauss_prod.add_ndmap(init_ndmap);	// in the right dimensions
  if(add_s) gauss_prod.add_ndmap(init_ndmap);                   // if the standard deviation also should be returned here is its place


  for(int i=0; i < dim; i++){ 				        // for each dimension
    for (int j=0; j < el; j++){				        // of each element

      double mean_num = 0.0;					// numerator (Zaehler)
      double mean_den = 0.0;					// denominator (Nenner)

      double ss_num = 1.0;                                      // numerator of the standard deviation
      double ss_den = 0.0;                                      // denominator of the standard deviation initialized with 0

      for(int k=0; k < sets; k++){
        mean_num += ((group.group[k].set[1].map[i][j])/(group.group[k].set[3].map[i][j]));
        mean_den += ( 1.0 / (group.group[k].set[3].map[i][j]));

        ss_den += (1.0/(group.group[k].set[3].map[i][j]));
      }

      gauss_prod.set[0].map[i][j] = (mean_num / mean_den) + (ss_num / ss_den);
      gauss_prod.set[1].map[i][j] = mean_num / mean_den;
      gauss_prod.set[2].map[i][j] = (mean_num / mean_den) - (ss_num / ss_den);
      if(add_s) gauss_prod.set[3].map[i][j] = ss_num / ss_den;
    }
  }

  std::string name = group.get_name() + "-Trajectory";
  gauss_prod.set_name(name);
  gauss_prod.set[0].set_name(name + "_mean+stdev");
  gauss_prod.set[1].set_name(name + "_mean");
  gauss_prod.set[2].set_name(name + "_mean-stdev");
  if(add_s) gauss_prod.set[0].set_name(name + "_stdev");

  if(! gauss_prod.correct_nans()) return empty;			// If the correction of nans is not successful, an empty set is returned
  return gauss_prod;
}

/**
 * The method make_model(std::deque< trajectory_lat > trajectories) creates a model, on how to behave
 * at a certain situation, regarding speciffic objects.
 * The parameter is a list of recorded trajectories.
 *
*/
ndmapSetGroup approximation::make_model(std::deque< trajectory_lat > trajectories){
  ndmapSetGroup group, tmp;

  int tra_num = trajectories.size();

  if(tra_num <= 0) return group;                                // If the deque parameter is empty
  int obj_num = trajectories[0].get_num_of_objects();           // The amount of objects, carried by each trajectory need to be equal


  for(int i=0; i < tra_num; i++){
    if(trajectories[i].get_num_of_objects() != obj_num) return group;
  }


  for(int i=0; i < tra_num; i++){                               // calculating the distance, between trajectory and object.
    tmp.add_ndmapSet(trajectories[i].return_distance());
  }


  for(int i=0; i < obj_num; i++){

    ndmapSet set;                                               // Fetching all data regarding one object
    for(int j=0; j < tra_num; j++){
      set.add_ndmap(tmp.get_set(j).get_ndmap(i));
    }
    if(set.get_num_of_maps() > 0) set.set_name(set.get_ndmap(0).get_name());
    group.add_ndmapSet(standard_deviation(set,true));                     // Get the mean and standard deviation of them
  }

  return group;
}



/***************************************************************************/

/*                             object                                      */

/***************************************************************************/

/**
 *      The method get_name returns the name of the object.
 */
std::string object::get_name(){
  return name;
}

/**
 *      The method set_name sets/changes the name of the object.
 */
void object::set_name(std::string _name){
  name = _name;
}

/**
 *      The method add_coordinate extends the list of coordinates of this object.
 */
void object::add_coordinate(double coord){
  coordinate.push_back(coord);
}

/**
 *      The method get_num_of_coordinate returns the number of saved coordinates.
 */
int object::get_num_of_coordinate(){
  return coordinate.size();
}

/**
 *      The method get_coordinate returns the i'th coordinate of the deque coordinate.
 *      If there are less than i coordinates an exception is raised.
 */
double object::get_coordinate(unsigned int i){
  if(coordinate.size() < i) throw data_error(name, -31);
  return coordinate[i];
}

/**
 *      The method set_coordinates sets all coordinates of the object at once.
 */
void object::set_coordinates(std::deque< double > _coordinate){
  coordinate = _coordinate;
}
/**
 *      The method print, displays the objects content in the terminal.
 */
void object::print(){
  std::cout<<"* Object name: "<< name << std::endl;
  std::cout<<"* Coordinates: ";
  for(unsigned int i=0; i < coordinate.size(); i++) std::cout<< coordinate[i] <<" ; ";
  std::cout<<std::endl;
}

bool object::subtract_kate_offset(){
  if(coordinate.size() != 3) return false;
  
  coordinate[0] = coordinate[0] - 38;
  coordinate[1] = coordinate[1] - 0;
  coordinate[2] = coordinate[2] - 592;
  
  return true;
}


/***************************************************************************/

/*                          trajectory_lat                                     */

/***************************************************************************/

/**
 *      The method get_ndmap returns the member map.
 */
ndmap trajectory_lat::get_ndmap(){
  return map;
}

void trajectory_lat::set_ndmap(ndmap _map){
  map = _map;
}

/**
 *      The method get_name returns the trajectories name.
 */
std::string trajectory_lat::get_name(){
  return name;
}

void trajectory_lat::set_name(std::string _name){
  name = _name;
}

/**
 *      The method set_ndmap_name sets the name of the member map.
 */
void trajectory_lat::set_ndmap_name(std::string name){
  map.set_name(name);
}

/**
 *      The method get_num_of_objects void object::get_num_of_objects()returns the number of objects.
 */
int trajectory_lat::get_num_of_objects(){
  return obj.size();
}

/**
 *      The method get_object() returns the i'th object of the list, if it is existent.
 *      Otherwise it returns an empty object.s
 */
object trajectory_lat::get_object(unsigned int i){
    if(obj.size() < i){
    object empty;
    return empty;
    }
    return obj[i];
}

/**
 *      The method add_object() adds the object given to the end of the list obj.
 */
void trajectory_lat::add_object(object _obj){
    obj.push_back(_obj);
}
/**
 *      The method clear() empties the trajectory.
 */
void trajectory_lat::clear(){
  name = "";
  obj.clear();
}


/**
 *      The method print() prints the trajectory's content to terminal.
 */
void trajectory_lat::print(){
  std::cout<<"Trajectory: "<< name << std::endl;
  std::cout<<"Involved Objects: "<< obj.size() << std::endl;
  for(unsigned int i=0; i < obj.size(); i++) obj[i].print();
  std::cout<<"Data: "<< std::endl;
  map.print();
  std::cout<< std::endl << std::endl;
}

/**
 *      The method return_distance() calculates the distance of the saved objects to its ndmap,
 *      groups it together in one ndmapSet and returns it as the result.
 *      Semanically: Calculation of the distance to all involved n objects to the recorded trajectory.
 *      This way n trajectories (ndmaps) are produced, put together in one ndmapSet.
 */
ndmapSet trajectory_lat::return_distance(){
  ndmapSet set;
  int num_of_objects = obj.size();
  for(int i=0; i < num_of_objects; i++){
    ndmap tmp;
    tmp.set_deque(map.get_deque());

    tmp.set_name(obj[i].get_name());
    tmp.minus(obj[i]);

    set.add_ndmap(tmp);

  }
  set.set_name( name );
  return set;
}

std::string trajectory_lat::get_obj_name(unsigned int x){
  if(obj.size() <= x) return ""; 
  return obj[x].get_name();
}

bool trajectory_lat::joint_to_task_space(){
  return map.joint_to_task_space();
}

void trajectory_lat::invert(){
  map.invert();
}

std::deque< object > trajectory_lat::get_objects(){
  return obj;
}

std::deque< double > trajectory_lat::get_euclide(){
  return map.get_euclide();
}

bool trajectory_lat::create_this_sequence( std::deque< int > seq ){
  return map.create_this_sequence(seq);
}


/**
 *  This method changes the order of the objects in the trajectory.
 *  For the reproduction of a task, there are some required objects - those have to be present.
 *  If there are more objects present, they will be ignored.
 *  The order of the objects in the trajectory is determined by the order of the required objects in _obj.
 */
bool trajectory_lat::set_obj_order(std::deque< object >* _obj){

  std::deque< int > index(_obj->size(), -1);

  // Check, if all at demonstration time used objects are now present, too.
  for(unsigned int i = 0; i < obj.size(); i++){
    bool found = false;
    for(unsigned int j = 0; j < _obj->size() && found == false; j++){
      if(obj[i].get_name() == (*_obj)[j].get_name()){
        if(index[j] == -1){
          found = true;
          index[j] = i;
        }
      }
    }
    if(! found) return false;
  }

  // All previous used objects have been found. Now bring them in the right order:

  std::deque< object > obj_new_order;

  for(unsigned int i = 0; i < index.size(); i++){
    if(index[i] != -1){
      obj_new_order.push_back(obj[index[i]]);
    }
  }

  obj = obj_new_order;
  return true;
}

void trajectory_lat::info(){
  std::cout << "\n*************************" << std::endl;
  std::cout << "\n* Trajectory Information" << std::endl;
  std::cout << "* Name: " << name << std::endl;
  map.info();
  std::cout << "* Included Objects: " << std::endl;
  std::cout << "*" << std::endl;
  for(unsigned int i=0; i < obj.size(); i++){
    obj[i].print();
  }
  std::cout << "\n*************************" << std::endl;
}

/**
 *      Adapt coordinates to katana-coordinates system
 */
bool trajectory_lat::subtract_kate_offset(){
 
  for(unsigned int i = 0; i < obj.size(); i++){
    if(! obj[i].subtract_kate_offset())
      std::cout<< "TRAJECTORY ERROR: object number " << i << "doesn't contain 3 dimensions as expected!" << std::endl;// TODO: Fehler (return false ) abfangen
  }
  return true;
}

/***************************************************************************/

/*                             ndmap                                       */

/***************************************************************************/

/**
 *      The constructor ndmap sets a default name, contaning a incremented number (global),
 *      in order to make the name unique.
 */
ndmap::ndmap(){
  maps_exist++;
  name = "Map" + intTOstring(maps_exist);
}

int ndmap::get_dim(){
  return map.size();
}

std::string ndmap::get_name(){
  return name;
}

void ndmap::set_name(std::string _name){
  name = _name;
}

/**
 *      This function adds a dimension to the ndmap.
 *      Therefore the standard container std::deque has to be used.
 */
void ndmap::add_deque(std::deque<double> deq){
  map.push_back(deq);
}

/**
 *      This function prints the content of the ndmap to the shell.
 */
void ndmap::print(){
  int dim = get_dim();
  if(dim > 0){
    int el = map[0].size();

    std::cout<<name<<std::endl;

    for(int j=0; j < el; j++){
      for(int i=0; i < dim; i++){
        std::cout<<map[i][j] << "  ";
      }
      std::cout<<std::endl;
    }
  }
}

void ndmap::info(){
  std::cout << "* Map Information" << std::endl;
  std::cout << "* Name: " << name << std::endl;
  std::cout << "* Dimensions: " << get_dim() << std::endl;
  std::cout << "* Elements: " << map_is_consistent() << std::endl;
}


bool ndmap::create_this_sequence( std::deque< int > seq ){

  std::deque<double> tmp;
  std::deque< std::deque<double> > map2( get_dim() , tmp);
  for(unsigned int i=0; i < seq.size(); i++){
    for(int j=0; j < get_dim(); j++){
      map2[j].push_back(map[j][seq[i]]);
    }
  }

  map = map2;

  return true;
}


/**
 *	The method correct_nans goes through all data of this ndmap and checks for faulty entries: nan (not a number).
 *	If it finds a nan the nan is replaced by the mean of it's two neighbours (the next non-nan neighbours).
 *	If it has only one (non-nan) neighbour it gets its value.
 *
 * 	If there are too many nans in the ndmap, the graphs and the approximations will probably be falsified.
 * 	Therefore the correction will only be applied, if the ratio of nans is lower than specified by the parameter ratio.
 * 	By default ratio is set to 0.05 (5%).
 *
 * 	Return value: 	true:  the correction occured
 * 			false: the ratio (nans/all elements) is > than specified by the parameter ratio
 */

bool ndmap::correct_nans(double ratio){

  int dim = get_dim();
  for(int i=0; i < dim; i++){						// count the amount of nans
    int el = map[i].size();
    int nans = 0;
    for(unsigned int j=0; j < map[i].size(); j++){
      if( isnan( map[i][j]) ) nans++;
    	//if(! (std::isnormal( map[i][j] ) ) ) nans++;
    }
    if(((double)nans/(double)el) > ratio) return false;			// If more than ratio (%*100) of all points are nans, it can't be reliably fixed -> return false;
  }
									// else correct the numbers!
  for(int i=0; i < dim; i++){
	//unsigned int el = map[i].size();
    for(unsigned int j=0; j < map[i].size(); j++){
      if( isnan(map[i][j])){
      //if(! (std::isnormal(map[i][j]))){

	    double left = 0.0;						// search for the left non-nan
	    bool left_found = false;
	    for(int k = 1; (int)j - k >= 0; k++){
	      if(!(isnan(map[i][j-k]))){
	      //if(std::isnormal(map[i][j-k])){
		left = map[i][j-k];
		left_found = true;
		break;
	      }
	    }

	    double right = 0.0;						// search for the right non_nan
	    bool right_found = false;
	    for(unsigned int k = 1; j + k < map[i].size(); k++){
	      if(!(isnan(map[i][j+k]))){	//if(std::isnormal(map[i][j+k])){
		right = map[i][j+k];
		right_found = true;
		break;
	      }
	    }

	if(left_found && right_found) map[i][j] = (left + right) / 2.0;	//if both values could be found: calculate mean of the two values
	else if(left_found) map[i][j] = left;				// only one value could be found: take that value
	else if(right_found) map[i][j] = right;				// only one value could be found: take that value

	else map[i][j] = 0.0; 						// no value was found: = 0.0
      }
    }
  }
  return true;
}


/**
 * 	The method map_is_consistent checks if the number of elements
 * 	in each dimension are the same.
 *
 * 	Return value:	the number of elements, if it is consistent;
 * 			-1 if the ndmap dosen't contain any data;
 * 			-2 if the ndmap - matrix is not rectangular (number of elements in each dimension is different);
 */
int ndmap::map_is_consistent(){

  try{
	unsigned int matrix_length;
    if(map.size() == 0) throw data_error(name, -1);                                                             //map is empty - no dimensions there
    matrix_length = map[0].size();

    for(unsigned int i=0; i < map.size(); i++){
      if(map[i].size() != matrix_length) throw data_error((name + " at dimension" + intTOstring(i)), -2);       // Matrix inconsistent
    }
    return matrix_length;
  }

  catch(data_error & err){
    return err.print();
  }

  return -42;
}

std::deque< double > ndmap::get_row(unsigned int row){
  if(map.size() > row) return map[row];
  else{
      std::deque< double > err;
      return err;
  }
}

/**
 *      The method minus takes the coordinates of the as a  parameter transmitted object and
 *      subtracts the first coordinate from all numbers in the first dimension, the second
 *      coordinate and subtracts it from all numbers of the second dimension and so forth.
 *      If the number of dimensions of the object and the ndmao are not equal, a exeption is raised.
 */
void ndmap::minus(object obj){

  int dim = get_dim();
  int el = map_is_consistent();

  if((obj.get_num_of_coordinate() == dim) && (el > 0)){

    for(int i=0; i < dim; i++){
      for(int j=0; j < el; j++){
        map[i][j] = map[i][j] - obj.get_coordinate(i);
      }
    }
  }
  else throw data_error("object " + obj.get_name() + " dim: " + boost::lexical_cast<std::string>( obj.get_num_of_coordinate() ) + " and map " + name  + " dim: " + boost::lexical_cast<std::string>( dim ) + " (in ndmap::minus)" , -32);
}

void ndmap::invert(){
  for(unsigned int i=0; i < map.size(); i++){
    std::deque< double > tmp;
    for(int j= map[i].size() -1; j > -1; j--){
      tmp.push_back(map[i][j]);
    }
    map[i] = tmp;
  }
}


void ndmap::set_deque(std::deque< std::deque<double> > deq){
  map = deq;
}

std::deque< std::deque<double> > ndmap::get_deque(){
  return map;
}

/**
 *      The method add_offset takes the coordinates of the as a  parameter transmitted object and
 *      adds the first coordinate to all numbers in the first dimension, the second
 *      coordinate and adds it to all numbers of the second dimension and so forth.
 *      If the number of dimensions of the object and the ndmao are not equal, a exeption is raised.
 */
void ndmap::add_offset(object obj){
  int dim = get_dim();
  int el = map_is_consistent();

  if((obj.get_num_of_coordinate() == dim) && (el > 0)){

    for(int i=0; i < dim; i++){
      for(unsigned int j=0; j < map[i].size(); j++){
        map[i][j] = map[i][j] + obj.get_coordinate(i);
      }
    }
  }
  else throw data_error("object " + obj.get_name() + " dim: " + boost::lexical_cast<std::string>( obj.get_num_of_coordinate() ) + " and map " + name + " dim: " + boost::lexical_cast<std::string>( dim ) + " (in ndmap::add_offset)", -32);
}

bool ndmap::thinning(int new_size){
  int old_size = map_is_consistent();
  int reduce = old_size - new_size;

  if(reduce == 0) return true;
  if(reduce < 0 ) return false;

  double step_size = ((double) old_size) / ((double) reduce);

  for(int i = reduce; i > 0; i--){
    int index = (int) floor((i * step_size) - 0.5);
    for(int j = 0; j < get_dim(); j++){
      map[j].erase(map[j].begin() + index);
    }
  }

  return true;
}

/**
 *  The function smoothing smootheth all dimensions in the map.
 *  It is beeing smoothened by a normal distribution according to the parameter sigma.
 *  The precision in the width of the smoothening (instead of being infinitesimally broad) is limited to
 *  the width of sigma * 3 in each direction.
 */
void ndmap::smoothing(double sigma){

  int width = sigma + 1;
  width = width * 3;

  std::deque< std::deque<double> > tmp = map; // Creating a  copy of all data

  // running through all dimensions
  for(unsigned int i=0; i < tmp.size(); i++){
    int el = tmp[i].size();
    for(int j=0; j < el; j++){

      int loc_width = width;
      if(width > j) continue; //loc_width = j;
      else if(width > (el-1) - j) continue; //loc_width = (el-1)-j;

      if(loc_width == 0) continue;

      double avg = 0;
      // Calculating the area of the middle point
      avg += tmp[i][j] * (2 * area_gauss(sigma, 0.0, 0.5));

      //std::cout<< "Area Gauss " << 0 << " = " << (2 * area_gauss(sigma, 0.0, 0.5)) << std::endl;

      for(int k=1; k < loc_width; k++){
        avg += tmp[i][j+k] * (area_gauss(sigma, k - 0.5, k + 0.5));
        avg += tmp[i][j-k] * (area_gauss(sigma, k - 0.5, k + 0.5));

        //std::cout<< "Area Gauss " << k << " = " << (area_gauss(sigma, k - 0.5, k + 0.5)) << std::endl;
      }
      std::cout << "Alt: " << map[i][j] << std::endl;
      map[i][j] = avg;
      std::cout << "Neu: " << map[i][j] << std::endl;
    }
  }
}

/**
 *  This method calculates the approximate area under a gaussian normal distribution
 *  considering sigma. x1 and x2 mark the borders of the area under the curve.
 */

double ndmap::area_gauss(double sigma, double x1, double x2){
  double F1, F2;
  F1 = (1/(sigma * sqrt(2* PI ))) * exp(-0.5*((x1/sigma) * (x1/sigma)));
  F2 = (1/(sigma * sqrt(2* PI ))) * exp(-0.5*((x2/sigma) * (x2/sigma)));
  return (((F1-F2)/2) + F2) * fabs(x1-x2);
}

void ndmap::push_front( std::deque< double > point){

  if(point.size() != map.size()) return; // TODO: Throw exception

  for(unsigned int i=0; i < map.size(); i++){
    map[i].push_front(point[i]);
  }
}

void ndmap::push_back( std::deque< double > point){

  if(point.size() != map.size()){
    return; // TODO: Throw exception
  }
  for(unsigned int i=0; i < map.size(); i++){
    map[i].push_back(point[i]);
  }
}

void ndmap::remove_dimension(int dim){

  if(dim < get_dim()){
    map.erase(map.begin() + dim);
  }
  else std::cout<<"[ndmap: Dimension could not be removed!] \n";
}

std::deque< std::deque<double> >* ndmap::data_pointer(){
  return &map;
}


/**
 *  Searches for gaps between two points bigger than max_gap and fills it linear with intermediate steps.
 */

void ndmap::fill_gaps(double max_gap){

  int el = map_is_consistent();
  if(el <= 0) return; //TODO throw exception

  int dim = map.size();

  for(int i=0; i < el-1; i++){

    // calculating the euclidian distance
    double dist = 0;
    for(int j=0; j < dim; j++)
      dist = pow( map[j][i+1] - map[j][i], 2);

    // test if gap is bigger
    if(sqrt(dist) > max_gap){

      // add a point at all dimensions
      for(int j=0; j < dim; j++){
        std::deque< double >::iterator it;
        it = map[j].begin();
        it = it + i + 1;
        map[j].insert (it, ((map[j][i] + map[j][i+1]) / 2) );
      }

      el++;
      i--;

    }
  }
}


void ndmap::init_map(int dimensions, int elements, double value){

  std::deque< double > el(elements, value);
  std::deque< std::deque< double > > _map(dimensions, el);
  map = _map;
}


bool ndmap::joint_to_task_space(){

  int el = map_is_consistent();
  if(el < 1) return false;

  int dim = get_dim();

  std::deque< double > init;
  std::deque< std::deque<double> > task_space(3 ,init); // Three dimensions for x,y,z

  // switch from leatra DK to KDL DK
  /*if( dim < 5 ){
    std::cout<<"[NDMAP] joint_to_task_space: at least 5 dimensions expected!"<<std::endl;
    return false;
  }
  std::deque< double > init;
  std::deque< std::deque<double> > task_space(3 ,init); // Three dimensions for x,y,z

  for(int i=0; i < el; i++){

    std::deque< double > angles;

    for(int j=0; j < 5; j++){  // taking over only the first 5 angles
      angles.push_back(map[j][i]);
    }

    std::deque< double > X;
    X = DK_lat(angles);

    if(X.size() != 3){
      std::cout<<"[NDMAP] joint_to_task_space: 3 dimensions expected!"<<std::endl;
      return false;
    }

    for(unsigned int j=0; j < X.size(); j++){
      task_space[j].push_back(X[j]);
    }
  }*/

  KDL::Tree my_tree;

  // set up robot description
  std::string robot_desc_string;
  ros::param::get("robot_description", robot_desc_string);
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
  	ROS_ERROR("Failed to construct kdl tree");
  	return 1;
  }

  // Create solver based on kinematic tree
  KDL::TreeFkSolverPos_recursive fksolver = KDL::TreeFkSolverPos_recursive(my_tree);

  // for every captured joint state
  for(int i=0; i < el; i++){
	  KDL::JntArray jointpositions = KDL::JntArray(dim);

	  for (int j = 0; j < dim; ++j) {
		jointpositions(j) = map[j][i];
	  }

	  // Create the frame that will contain the results
	  KDL::Frame cartpos;

	  // Calculate forward position kinematics
	  bool kinematics_status;
	  kinematics_status = fksolver.JntToCart(jointpositions,cartpos, "katana_gripper_tool_frame");	// TODO: Katana specific
	  if(kinematics_status>=0){
		  // success now copy the result
		  for(unsigned int j=0; j < 3; j++){
			  task_space[j].push_back(cartpos.p[j]);
		  }
	  }else{
		  ROS_ERROR("Could not calculate forward kinematics!");
	  }
  }

  map = task_space;
  return true;
}

std::deque< double > ndmap::get_euclide(){
  std::deque< double > euclide;

  for(int i=0; i < map_is_consistent(); i++){

    double sum = 0;
    for(int j=0; j < get_dim(); j++){
      sum += (map[j][i] * map[j][i]);
    }
    euclide.push_back( sqrt( sum ));
  }
  return euclide;
}




/***************************************************************************/

/*                             ndmapSet                                    */

/***************************************************************************/

/**
 *      The constructor ndmapSet sets a default name, contaning a incremented number (global),
 *      in order to make the name unique.
 */
ndmapSet::ndmapSet(){
  sets_exist++;
  name = "Set" + intTOstring(sets_exist);
}

/**
 *      This method empties the set and erases the name.
 */
void ndmapSet::clear(){
  set.clear();
  name = "";
}

/**
 *      The method get_dim returns the number of dimensions present in the set.
 *      If the number of dimensions are different in the ndmaps, an error code
 *      is returned.
 */
int ndmapSet::get_dim(){
  int el = set_is_consistent();
  if(el <= 0) return el;

  if(set.size() > 0){
    return set[0].get_dim();
  }
  return 0;
}

/**
 *      The method get_ndmap returns the ndmap in the set at position i.
 */
ndmap ndmapSet::get_ndmap(int i){
  return set[i];
}

int ndmapSet::get_num_of_maps(){
  return set.size();
}

void ndmapSet::set_name(std::string _name){
  name = _name;
}

std::string ndmapSet::get_name(){
  return name;
}

/**
 *      The method add_ndmap adds a ndmap at the end of the deque 'map'.
 */
void ndmapSet::add_ndmap(ndmap x){	// changed from int to void BR
  set.push_back(x);
}

/**
 *      Helpfunction to print the content of a set to the shell.
 */
void ndmapSet::print(){
  std::cout<<name<<std::endl;
  for(unsigned int i=0; i < set.size(); i++){
    set[i].print();
    std::cout<<std::endl;
  }
}

void ndmapSet::info(){
  std::cout << "\n* Set Information" << std::endl;
  std::cout << "* Name: " << name << std::endl;
  std::cout << "* Maps: " << get_num_of_maps() << std::endl;
  std::cout << "* Dimensions: " << get_dim() << std::endl;
  std::cout << "* Elements: " << set_is_consistent() << std::endl << std::endl;
}

/**
 * 	The method set_is_consistent checks the ndmapSet on consistency.
 *      Additionally to the return values of "map_is_consistent()" there are following return values and their meaning:
 *
 * 	Return value:	the number of elements, if it is consistent;
 * 			-1 if a ndmap dosen't contain any data [origin: ndmap::map_is_consistent()];
 * 			-2 if a ndmap - matrix is not rectangular (number of elements in each dimension is different) [origin: ndmap::map_is_consistent()];
 *                      -3 if the set is empty;
 *                      -4 if the ndmaps have a differnt number of elements (matrices have differnent size: collumns);
 *                      -5 if the ndmaps have not the same number of dimensions (matrices have different size: lines);
 *
 */
int ndmapSet::set_is_consistent(){
  try{
    int consistent;
    if(set.size() == 0) throw data_error(name, -3); 		//empty set
    consistent = set[0].map_is_consistent();
    if(consistent < 0) return consistent;			//the map itself is inconsistent

    int dim = set[0].get_dim();
    for(unsigned int i=0; i < set.size(); i++){
      if(dim != set[i].get_dim()) throw data_error(name, -5);	// the underlying maps have a differnent number of dimensions
    }

    for(unsigned int i=0; i < set.size(); i++){
      int cons_test = set[i].map_is_consistent();
      if(consistent != cons_test){                              // test if all ndmaps have the same length (number of elements)
        if(cons_test < 0) return cons_test; 		        //the map itself is inconsistent
        else throw data_error(name, -4); 			// maps are different
      }
    }
    return consistent;					        // the sets are all consistent -> return the number of elements!
  }

  catch(data_error& err){
    return err.print();
  }

  return -42;
}


void ndmapSet::remove_dimension(int dim){
  for(unsigned int i=0; i < set.size(); i++){
    set[i].remove_dimension(dim);
  }
}

/**
 *      This function calls the "ndmap::correct_nans()" for all its elements.
 */
bool ndmapSet::correct_nans(double ratio){
  for(unsigned int i=0; i<set.size(); i++){
    if(! set[i].correct_nans(ratio)) return false;

  }
  return true;
}

/**
 *      The method add_offset takes the coordinates of the as a  parameter transmitted object and
 *      calls the ndmap::add_offset method for all its containing ndmaps.
 */
void ndmapSet::add_offset(object obj){
  int num_of_maps = get_num_of_maps();
  for(int i=0; (i < num_of_maps) && (i < 3); i++){              // Explanation: (i < 3) at Position 3, there is the standard deviation, here it doesn't need to be added.
    set[i].add_offset(obj);
  }
}

void ndmapSet::smoothing(double sigma){
  for(unsigned int i=0; i < set.size(); i++)
    set[i].smoothing(sigma);
}

/**
 *  Returns the pointer to the data of the ndmap number i.
 */
std::deque< std::deque<double> >* ndmapSet::data_pointer(int i){
  return set[i].data_pointer();
}

/*
// Implementation until now only works for exactly two ndmaps in the set
void ndmapSet::time_warp(){

  std::deque< std::deque<double> > x = set[0].get_deque();
  std::deque< std::deque<double> > y = set[1].get_deque();
  int dim = get_dim();

  warper warp;
  ndmap map[2];

  for(int i=0; i < dim; i++){

    warp.set_vector(x[i], y[i]);
    warp.warping();
    map[0].add_deque(warp.get_x_vector());
    map[1].add_deque(warp.get_y_vector());
  }

  clear();
  add_ndmap(map[0]);
  add_ndmap(map[1]);
}
*/
/***************************************************************************/

/*                             ndmapSetGroup                               */

/***************************************************************************/

/**
 *      The constructor ndmapSetGroup sets a default name, contaning a incremented number (global),
 *      in order to make the name unique.
 */
ndmapSetGroup::ndmapSetGroup(){
  groups_exist++;
  name = "Group" + intTOstring(groups_exist);
}

int ndmapSetGroup::get_num_of_sets(){
  return group.size();
}

/**
 *      This method returns the number of dimensions.
 *      If the underlying data is inconsistent it returns the return value of "ndmapSetGroup::group_is_consistent()".
 */
int ndmapSetGroup::get_dim(){
  int el = group_is_consistent();
  if(el <= 0) return el;

  if(group.size() > 0){
    return group[0].get_dim();
  }
  return 0;
}

void ndmapSetGroup::set_name(std::string _name){
  name = _name;
}

std::string ndmapSetGroup::get_name(){
  return name;
}

/**
 *      The method get_set_name returns the name of the i'th set.
 *      If there are less than i sets in the group, a question mark is returned.
 */
std::string ndmapSetGroup::get_set_name(unsigned int i){
    if(group.size() < i) return "?";
    return group[i].get_name();
}


/**
 *      This function returns the set at position i.
 */
ndmapSet ndmapSetGroup::get_set(int i){
  return group[i];
}

void ndmapSetGroup::add_ndmapSet(ndmapSet set){
  group.push_back(set);
}

/**
 *      This help function prints the data to the shell.
 */
void ndmapSetGroup::print(){
  for(unsigned int i=0; i < group.size(); i++){
    group[i].print();
    std::cout<<std::endl;
  }
}


/**
 * 	The method group_is_consistent checks the ndmapSetGroup on consistency.
 *      There are following return values and their meaning:
 *
 * 	Return value:	the number of elements, if it is consistent;
 * 			-1 if a ndmap dosen't contain any data [origin: ndmap::map_is_consistent()];
 * 			-2 if a ndmap - matrix is not rectangular (number of elements in each dimension is different) [origin: ndmap::map_is_consistent()];
 *                      -3 if the set is empty [origin: ndmapSet::map_is_consistent()];
 *                      -4 if the ndmaps have a differnt number of elements (matrices have differnent size: collumns) [origin: ndmapSet::map_is_consistent()];
 *                      -5 if the ndmaps have not the same number of dimensions (matrices have different size: lines) [origin: ndmapSet::map_is_consistent()];
 *                      -6 if the underlying sets have different number of elements.
 *                      -7 if the group is emptly
 *                      -8 if the underlying sets have different number of dimensions.
 */
int ndmapSetGroup::group_is_consistent(){
  try{
    int el;
    int dim;

    if(group.size() == 0) throw data_error(name, -7); 		// empty group
    el = group[0].set_is_consistent();
    if(el < 0) return el; 			                // the underlying map or set is inconsistent
    dim = group[0].get_dim();

    for(unsigned int i=0; i < group.size(); i++){
      int el_test = group[i].set_is_consistent();
      if(el != el_test){
        if(el_test < 0) return el_test; 			// the underlying map or set is inconsistent
        else throw data_error(name, -6); 			//the underlying sets have different number of elements.
      }

      if(dim != group[i].get_dim()) throw data_error(name, -8); //the underlying sets have different number of dimensions.
    }

    return el;
  }

  catch(data_error& err){
    return err.print();
  }

  return -42;
}


/**
 *      This function calls the "ndmapSet::correct_nans()" for all its elements.
 */
bool ndmapSetGroup::correct_nans(double ratio){
  for(unsigned int i=0; i < group.size(); i++){
    if(! group[i].correct_nans(ratio)) return false;
  }
  return true;
}

/**
 *      The method info prints statistical information about the group to terminal.
 */
void ndmapSetGroup::info(){

  std::cout << "\n* Group Information" << std::endl;
  std::cout << "* Name: " << name << std::endl;
  std::cout << "* Sets: " << get_num_of_sets() << std::endl;
  std::cout << "* Dimensions: " << get_dim() << std::endl;
  std::cout << "* Elements: " << group_is_consistent() << std::endl << std::endl;
}

/**
 *      The method add_offset takes the coordinates of the as a  parameter transmitted object and
 *      calls the ndmapSet::add_offset method for all its containing ndmaps.
 */
void ndmapSetGroup::add_offset( std::deque< object > obj){
  int num_of_sets = get_num_of_sets();
  for(int i=0; i < num_of_sets; i++){
    //int obj_num = -1;
    for(unsigned int j=0; j < obj.size(); j++){
      if(obj[j].get_name() == group[i].name){
        group[i].add_offset(obj[j]);
        break;
      }
    }
  }
}



/***************************************************************************/

/*                             data_error                                  */

/***************************************************************************/

/**
 *      The constructor of the class data_error initializes its members.
 *      The first parameter allowes to transmit a speciffic message and the second
 *      parameter a code. By the code it is determined, on how to react on this particular error.
 */
data_error::data_error(std::string _msg, int _code): msg(_msg), code(_code){ E = "Leatra Error: ";}

/**
 *  The method print() prints the data related error messages to the terminal.
 *  The return parameter is always the private member "code". It is defined as follows:
 *
 * 	Return value:
 * 			-1 if a ndmap dosen't contain any data [origin: ndmap::map_is_consistent()];
 * 			-2 if a ndmap - matrix is not rectangular (number of elements in each dimension is different) [origin: ndmap::map_is_consistent()];
 *                      -3 if the set is empty [origin: ndmapSet::map_is_consistent()];
 *                      -4 if the ndmaps have a differnt number of elements (matrices have differnent size: collumns) [origin: ndmapSet::map_is_consistent()];
 *                      -5 if the ndmaps have not the same number of dimensions (matrices have different size: lines) [origin: ndmapSet::map_is_consistent()];
 *                      -6 if the underlying sets have different number of elements [origin: ndmapSetGroup::map_is_consistent()];
 *                      -7 if the group is emptly [origin: ndmapSetGroup::map_is_consistent()];
 *                      -8 if the underlying sets have different number of dimensions [origin: ndmapSetGroup::map_is_consistent()];
 *                      -9 if there are less than 2 ndmaps in the set.
 *                      -10 if there are less than 2 ndmapSets in the group.
 */
int data_error::print(){
  std::cout << E;
  switch(code){
    case -1: std::cout << "The ndmap "          << msg << " is empty!";                                                                       break;
    case -2: std::cout << "The ndmap "          << msg << " has varying number of elements!";                                                 break;
    case -3: std::cout << "The ndmapSet "       << msg << " is empty!";                                                                       break;
    case -4: std::cout << "The ndmapSet "       << msg << " carrys ndmaps with a varying number of elements!";                                break;
    case -5: std::cout << "The ndmapSet "       << msg << " carrys ndmaps with a varying number of dimensions!";                              break;
    case -6: std::cout << "The ndmapSetGroup "  << msg << " carrys ndmaps with a varying number of elements!";                                break;
    case -7: std::cout << "The ndmapSetGroup "  << msg << " is empty!";                                                                       break;
    case -8: std::cout << "The ndmapSetGroup "  << msg << " carrys ndmaps with a varying number of dimensions!";                              break;
    case -9: std::cout << "The set_validation for the ndmapSet "  << msg << " failed. There are less than 2 ndmaps present!";                 break;
    case -10: std::cout << "The group_validation for the ndmapSetGroup "  << msg << " failed. There are less than 2 ndmapSets present!";      break;
    case -21: std::cout << "The file "          << msg << " couldn't be opened!";                                                             break;
    case -31: std::cout << "Object "            << msg << " there was an attempt of illegal access on the coordinates!";                      break;
    case -32: std::cout << "The number of dimensions of " << msg << " are not equal!";                                                        break;
    case -41: std::cout << "The file " << msg << " couldn't be created! Does it exist already?";                                              break;
    case -42: std::cout << "Cannot compute this command: " << msg;                                                                            break;
  }
  std::cout<<std::endl;
  return code;
}


/**
 *  The init function has to be called in the beginning when fastad is running.
 *  It initializes three variables, that are needed in naming new maps, sets and groups, when the user doesn't name then himself.
 */
void init(){

  maps_exist = 0;
  sets_exist = 0;
  groups_exist = 0;
}
