#ifndef DTW_H
#define DTW_H

#include <deque>
#include <math.h>
#include <limits>


struct tuple{

  int x;
  int y;
  
  tuple(int _x, int _y){
    x = _x;
    y = _y;
  }
};

class warp{
  
private:
  std::deque< double > x;
  std::deque< double > y;
  
  std::deque< std::deque< double > > D;
  std::deque< std::deque< double > > D_acc;
  
  std::deque< tuple > path;
  
  double path_costs;

public:
  void set_x(std::deque< double > _x);
  void set_y(std::deque< double > _y);
  void make_distance_matrix_slope();
  int get_D_rows();
  int get_D_cols();
  int get_D_acc_rows();
  int get_D_acc_cols();
  void accumulate_distance_matrix();
  void accumulate_distance_matrix_step_size_cond2();
  void find_warping_path();
  void find_warping_path_classic();
  void find_warping_path_classic_adapted();
  void find_warping_path_Sakoe_Chiba();
  void add_point_to_path_front(int x, int y);
  std::deque< tuple > get_path();
  void normalize_path_for_x();
  void normalize_path_for_y();
  std::deque< int > get_path_x();
  std::deque< int > get_path_y();
  void print_path();
  double path_cost();
  double get_path_costs();
  std::deque< std::deque< double > > get_D();
  std::deque< std::deque< double > > get_D_acc();
  bool D_consistency_check();
  bool D_acc_consistency_check();
  
  void info();
};

#endif