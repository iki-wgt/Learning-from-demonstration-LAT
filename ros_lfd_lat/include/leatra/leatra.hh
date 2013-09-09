#ifndef LEATRA_H
#define LEATRA_H

#include <vector>
#include <deque>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <cmath>


#include "stringhelp.hh"
#include "kinematics.hh"

#define PI 3.1415926535898

class ndmap;
class ndmapSet;
class ndmapSetGroup;
class approximation;
class trajectory_lat;
class object;

/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class approximation holds the functions necessary for approximation.
 */

class approximation{

private:
  int set_validation(ndmapSet set);
  int group_validation(ndmapSetGroup group);

public:
  ndmap mean_value(ndmapSet set);
  ndmapSet standard_deviation(ndmapSet set, bool add_s = false);
  ndmapSet constraint_fusion(ndmapSetGroup group, bool add_s = false);

  ndmapSetGroup make_model(std::deque< trajectory_lat > trajectories);

};

/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The ndmap class represents a matrix.
 * The class contains n-dimensions of a trajectory, described in discrete steps.
 * Each dimension can contain as many elements as needed.
 * The ndmap's dimenstions need all to have the same number of elements. The matrix needs to be rectangular.
 */

class ndmap{

private:

  /** @param map holds a list of lists of doubles. It can be looked at as a matrix. */
  std::deque< std::deque<double> > map;
  std::string name;

public:

  ndmap();
  int get_dim();
  std::string get_name();
  void set_name(std::string _name);
  void add_deque(std::deque<double> deq);
  bool correct_nans(double ratio = 0.05);
  int map_is_consistent();
  void print();
  void minus(object obj);
  void add_offset(object obj);
  void set_deque(std::deque< std::deque<double> > deq);
  std::deque< std::deque<double> > get_deque();
  std::deque< double > get_row(unsigned int row);
  void smoothing(double sigma);
  double area_gauss(double sigma, double x1, double x2);
  bool thinning(int new_size);
  void push_front( std::deque< double > point);
  void push_back(  std::deque< double > point);
  void fill_gaps(double max_gap);
  void init_map(int dimensions, int elements, double value = 0);
  bool joint_to_task_space();
  void info();
  void remove_dimension(int dim);
  void invert();
  std::deque< double > get_euclide();
  bool create_this_sequence( std::deque< int > seq );
  std::deque< std::deque<double> >* data_pointer();
  void getMinMax(unsigned int row, double& min, double& max);
  unsigned int getDimWithMaxDev();

  friend class approximation;
  friend class litera;
  friend class warp_master;
  friend class lfd;
  friend class warp_leatra;

};

/**(*tra)[i].
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class trajectory_lat holds the data and functions necessary for processing a trajectory.
 */

class trajectory_lat{
private:
  ndmap map;
  std::string name;
  std::deque< object > obj;

public:
  ndmap get_ndmap();
  void set_ndmap(ndmap _map);
  std::string get_name();
  void set_name(std::string _name);
  void set_ndmap_name(std::string name);
  int get_num_of_objects();
  object get_object(unsigned int i);
  std::string get_obj_name(unsigned int x);
  void add_object(object _obj);
  void clear();
  void print();
  void info();
  ndmapSet return_distance();
  bool joint_to_task_space();
  void invert();
  std::deque< object > get_objects();
  std::deque< double > get_euclide();
  bool create_this_sequence( std::deque< int > seq );
  bool set_obj_order(std::deque< object >* obj);
  int get_tra_length(){
	  return map.map_is_consistent();
  }
  bool subtract_kate_offset();

  friend class warp_leatra;
  friend class lfd;
};



/**
 * @author  Heiko Posenauer
 * @date 02.02.2012Group
 * @version 2.0
 *
 * The ndmapSet class holds a set of ndmaps.
 * The smallest ndmapSet contains one ndmap.
 * Again this data type can carry as many ndmaps as the user wishes.
 * The Set has to hold only ndmaps that are similar:
 * They need all to be rectangular, and of the same dimensions i.e. the number of the dimensions and elements need to be equal.
 * -> All matrices need to be of the same order.
  */

class ndmapSet{

private:
public:
    /** @param set holds a list of ndmaps. It can be looked at as a three dimensional matrix. */
  std::deque< ndmap > set;
  std::string name;
  void clear();
  std::deque<double> get_min_value();
  std::deque<double> get_max_value();

//public:
  ndmapSet();
  int get_dim();
  int get_num_of_maps();
  ndmap get_ndmap(int i);
  void set_name(std::string _name);
  std::string get_name();
  bool correct_nans(double ratio = 0.05);
  int set_is_consistent();
  void add_ndmap(ndmap x);
  void print();
  void add_offset(object obj);
  void remove_dimension(int dim);
  void smoothing(double sigma);
  bool thinning(int new_size);
  void info();
  std::deque< std::deque< double > >* data_pointer(int i);
  std::deque< bool > getConstraints(double threshold);

  friend class approximation;
  friend class litera;
  friend class warp_master;
};



/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *map
 * The ndmapSetGroup data type is a list of ndmapSets.
 * Each describing the same thing from a different perspective.
 * The ndmapSetGroup data type can carry as many ndmapSets as needed.
 * The Group has to hold only ndmapSets that are similar:
 * They need all to be rectangular, and of the same dimensions i.e. the number of the dimensions and elements need to be equal.
 * -> All matrices need to be of the same order.
 */

class ndmapSetGroup{

private:
  /** @param group groups a list of ndmapSets. */
  std::deque< ndmapSet > group;
  std::string name;
  std::deque<double> get_min_value();
  std::deque<double> get_max_value();

public:
  ndmapSetGroup();
  int get_num_of_sets();
  int get_dim();
  void set_name(std::string _name);
  std::string get_name();
  std::string get_set_name(unsigned int i);
  int group_is_consistent();
  bool correct_nans(double ratio = 0.05);
  ndmapSet get_set(int i);
  void add_ndmapSet(ndmapSet set);
  void print();
  void info();
  void add_offset( std::deque< object > obj);

  friend class approximation;
  friend class litera;
};






/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class object holds the coordinates of a speciffic object, defined by its private member "name"..
 */

class object{
private:
  std::string name;
  std::deque< double > coordinate;

public:
  std::string get_name();
  void set_name(std::string _name);
  void add_coordinate(double coord);
  void set_coordinates(std::deque< double > _coordinate);
  int get_num_of_coordinate();
  double get_coordinate(unsigned int i);
  bool subtract_kate_offset();
  void print();
};



/**
 * @author  Heiko Posenauer
 * @date 02.02.2012
 * @version 2.0
 *
 * The class data_error is managing the exeption handling for the data related errors.
 * The member "code" holds the error type and the string "msg" some more detailed information, most of the time the name of the data-type the error occured.
 */

class data_error{

private:
  std::string E;
  std::string msg;
  int code;

public:
  data_error(std::string _msg, int _code);
  int print();
};

void init();


#endif
