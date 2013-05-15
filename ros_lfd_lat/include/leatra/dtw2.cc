#include <iostream>
#include "dtw2.hh"

#define DELAY 5


/**
 *  The path must exist already and needs to have a match for all elements on both sides
 *  This function will reduce double matchings for x.
 */
void warp::normalize_path_for_x(){
  if(path.size() > 0){
   // std::cout<<" [Normal] path size = " << path.size() << std::endl;
    int size = path.size() - 1;
    for(int i = size; i > 0; i--){
       //   std::cout<<" [Normal] point " << path[i].x << std::endl;
      if(path[i].x <= path[i-1].x){
        path.erase(path.begin()+i);
	// std::cout<<" [Normal] remove point " << path[i].x << std::endl;
      }
    }
  }
  // std::cout<<" [Normal] path size = " << path.size() << std::endl;
}



/**
 *  The path must exist already and needs to have a match for all elements on both sides
 *  This function will reduce double matchings for y.
 */
void warp::normalize_path_for_y(){
  if(path.size() > 0){
   // std::cout<<" [Normal] path size = " << path.size() << std::endl;
    int size = path.size() - 1;
    for(int i = size; i > 0; i--){
       //   std::cout<<" [Normal] point " << path[i].x << std::endl;
      if(path[i].y <= path[i-1].y){
        path.erase(path.begin()+i);
	// std::cout<<" [Normal] remove point " << path[i].x << std::endl;
      }
    }
  }
  // std::cout<<" [Normal] path size = " << path.size() << std::endl;
}


/**
 *  returns the path only for the x vector
 */
std::deque< int > warp::get_path_x(){
  std::deque< int > _x;
  for(unsigned int i=0; i < path.size(); i++){
    _x.push_back(path[i].x);
  }
  return _x;
}

/**
 *  returns the path only for the y vector
 */
std::deque< int > warp::get_path_y(){
  std::deque< int > _y;
  for(unsigned int i=0; i < path.size(); i++){
    _y.push_back(path[i].y);
  }
  return _y;
}

void warp::info(){
  std::cout<<"\n* Warp Information" << std::endl;
  std::cout<<"* Elements in x: "<< x.size() << std::endl;
  std::cout<<"* Elements in y: "<< y.size() << std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"* Matrix D: "<< std::endl;
  if(D_consistency_check()) std::cout<< "* D is consistent" << std::endl;
  else std::cout<< "* D is inconsistent!"<< std::endl;
  if(D.size() == 0) std::cout<<"* Elements in x: "<< 0 << std::endl;
  else   std::cout<<"* Elements in x: "<< D[0].size() << std::endl;
  std::cout<<"* Elements in y: "<< D.size() << std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"* Matrix D_acc: "<< std::endl;
  if(D_acc_consistency_check()) std::cout<< "* D_acc is consistent" << std::endl;
  else std::cout<< "* D_acc is inconsistent!"<< std::endl;
  if(D_acc.size() == 0) std::cout<<"* Elements in x: "<< 0 << std::endl;
  else   std::cout<<"* Elements in x: "<< D_acc[0].size() << std::endl;
  std::cout<<"* Elements in y: "<< D_acc.size() << std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"* Path length: " << path.size() << std::endl;
}


bool warp::D_acc_consistency_check(){

  if(D_acc.size() != y.size()) return false;

  for(unsigned int i=0; i < D_acc.size(); i ++){
    if(D_acc[i].size() != x.size()) return false;
  }
  return true;
}

bool warp::D_consistency_check(){

  if(D.size() != y.size()) return false;

  for(unsigned int i=0; i < D.size(); i ++){
    if(D[i].size() != x.size()) return false;
  }
  return true;
}

void warp::set_x(std::deque< double > _x){
  x = _x;
}

void warp::set_y(std::deque< double > _y){
  y = _y;
}

void warp::make_distance_matrix_slope(){
  int n = x.size();
  int m = y.size();
  if(n == 0 || m == 0) return;

  for(int i=0; i < DELAY ; i++){
    x.push_back(x[n-1]);
    y.push_back(y[m-1]);
  }
  for(int i=0; i < DELAY ; i++){
    x.push_front(x[1]);
    y.push_front(y[1]);
  }

  n = x.size();
  m = y.size();

  std::deque< double > row(n,0.0);
  std::deque< std::deque< double > > _D(m, row);

  for(int i= DELAY ; i < m; i++){
    for(int j= DELAY ; j < n; j++){

      double delta[2];
      delta[0] = 0;
      delta[1] = 0;

      for(int k = 0; k < DELAY ; k ++){
	delta[0] += (x[j + (DELAY - k)] - x[j - k]) * (1.0 / DELAY );
	delta[1] += (y[i + (DELAY - k)] - y[i - k]) * (1.0 / DELAY );
      }

      _D[i][j] = fabs(delta[0] - delta[1]);
    }
  }

  for(int i=0; i < DELAY ; i++){
    x.pop_back();
    y.pop_back();
    x.pop_front();
    y.pop_front();
    _D.pop_front();
    _D.pop_back();
    for(unsigned int j=0; j < _D.size(); j++){
      _D[j].pop_front();
      _D[j].pop_back();
    }
  }

  D = _D;
}

int warp::get_D_acc_rows(){
  return D_acc.size();
}

int warp::get_D_rows(){
  return D.size();
}

int warp::get_D_acc_cols(){
  if(D_acc.size() == 0) return 0;
  else return D_acc[0].size();
}

int warp::get_D_cols(){
  if(D.size() == 0) return 0;
  else return D[0].size();
}

void warp::accumulate_distance_matrix(){

  int rows = get_D_rows();
  int cols = get_D_cols();

  D_acc = D;

  // Accumulate the first column:
    for(int i=1; i < rows; i++){
    D_acc[i][1] += D_acc[i-1][1];
  }

  // Accumulate the first row:
  for(int i=1; i < cols; i++){
    D_acc[1][i] += D_acc[1][i-1];
  }

  // Accumulate diagonal (left top to right down)
  for(int i = 1; i < rows; i++){
    for(int j = 1; j < cols ; j++){

      if(D_acc[i-1][j-1] < D_acc[i-1][j]){

        if(D_acc[i-1][j-1] < D_acc[i][j-1])
          D_acc[i][j] += D_acc[i-1][j-1];
        else
          D_acc[i][j] += D_acc[i][j-1];
      }
      else{

        if(D_acc[i-1][j] < D_acc[i][j-1])
          D_acc[i][j] += D_acc[i-1][j];
        else
          D_acc[i][j] += D_acc[i][j-1];
      }
    }
  }
}

void warp::accumulate_distance_matrix_step_size_cond2(){

  int rows = get_D_rows();
  int cols = get_D_cols();

  D_acc = D;

  // Accumulate the first column:
  for(int i=1; i < cols; i++){
    D_acc[i][1] += D_acc[i-1][1];
  }

  // Accumulate the first row:
  for(int i=1; i < rows; i++){
    D_acc[1][i] += D_acc[1][i-1];
  }

  // Add a first row
  std::deque< double > first_row(cols + 1, std::numeric_limits< double >::infinity());
  first_row[0] = 0;
  //  std::numeric_limits< double >::infinity();
  D_acc.push_front(first_row);

  //Add a first column
  for(int i = 1; i < rows; i++){
    D_acc[i].push_front(std::numeric_limits< double >::infinity());
  }

  rows++;
  cols++;

  // Accumulate diagonal (left top to right down)
  for(int i = 2; i < cols; i++){
    for(int j = 2; j < rows ; j++){

      if(D_acc[i-1][j-1] < D_acc[i-2][i-1]){

        if(D_acc[i-1][j-1] < D_acc[i-1][i-2])
          D_acc[i][j] += D_acc[i-1][j-1];
        else
          D_acc[i][j] += D_acc[i-1][i-2];
      }
      else{

        if(D_acc[i-2][i-1] < D_acc[i-1][i-2])
          D_acc[i][j] += D_acc[i-2][i-1];
        else
          D_acc[i][j] += D_acc[i-1][i-2];
      }
    }
  }

  // Remove first row
  D_acc.pop_front();
  // Remove first column
  for(unsigned int i=0; i < D_acc.size(); i++){
    D_acc[i].pop_front();
  }
}

void warp::add_point_to_path_front(int x, int y){
    struct tuple tmp(x,y);
    path.push_front(tmp);
}



void warp::find_warping_path_classic(){

  int path_x = get_D_acc_cols() -1;
  int path_y = get_D_acc_rows() -1;       // Starting position of the warping path

  add_point_to_path_front(path_x , path_y);

  while((path_x > 0) || (path_y > 0)){  // End and target position is x;y  0;0

    // choosing from 3 close neighbours the cheapest way

    if(path_x == 0){ // new position of path is clear
      path_y--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(path_y == 0){ // new position of path is clear
      path_x--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    // find the cheapest of all three:
    if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 0][path_x - 1]){

      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 0]){
        path_y--;
        path_x--;
      }
      else{
        path_y--;
      }
    }
    else{
      if(D_acc[path_y - 0][path_x - 1] <= D_acc[path_y - 1][path_x - 0]){
        path_x--;
      }
      else{
        path_y--;
      }
    }

    add_point_to_path_front(path_x, path_y);
  }
}

void warp::find_warping_path_classic_adapted(){

  int path_x = get_D_acc_cols() -1;
  int path_y = get_D_acc_rows() -1;       // Starting position of the warping path

  bool x_repeat = false;
  bool y_repeat = false;

  add_point_to_path_front(path_x , path_y);

  while((path_x > 0) || (path_y > 0)){  // End and target position is x;y  0;0

    // choosing from 3 close neighbours the cheapest way

    if(path_x == 0){ // new position of path is clear
      path_y--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(path_y == 0){ // new position of path is clear
      path_x--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(x_repeat){
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 0][path_x - 1]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
      }
      x_repeat = false;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(y_repeat){
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 0]){
        path_y--;
        path_x--;
      }
      else{
        path_y--;
      }
      y_repeat = false;
      add_point_to_path_front(path_x , path_y);
      continue;
    }


    // find the cheapest of all three:
    if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 0][path_x - 1]){

      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 0]){
        path_y--;
        path_x--;
      }
      else{
        path_y--;
        x_repeat = true;
      }
    }
    else{
      if(D_acc[path_y - 0][path_x - 1] <= D_acc[path_y - 1][path_x - 0]){
        path_x--;
        y_repeat = true;
      }
      else{
        path_y--;
        x_repeat = true;
      }
    }

    add_point_to_path_front(path_x, path_y);
  }
}


void warp::find_warping_path(){

  int path_x = get_D_acc_cols() -1;
  int path_y = get_D_acc_rows() -1;       // Starting position of the warping path

  add_point_to_path_front(path_x , path_y);

  while((path_x > 0) || (path_y > 0)){  // End and target position is x;y  0;0

    // choosing from 3 close neighbours the cheapest way

    if(path_x == 0){ // new position of path is clear
      path_y--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(path_y == 0){ // new position of path is clear
      path_x--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(1 == path_x && 1 == path_y){
      path_x--;
      path_y--;
      add_point_to_path_front(path_x, path_y);
      continue;
    }

    if(1 == path_x){

      // find the cheapest of two:
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 2][path_x - 1]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
        path_y--;
        path_y--;
      }
      add_point_to_path_front(path_x, path_y);
      continue;
    }

    if(1 == path_y){

      // find the cheapest of two:
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 2]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
        path_x--;
        path_y--;
      }
      add_point_to_path_front(path_x, path_y);
      continue;
    }

    // find the cheapest of all three:
    if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 2]){

      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 2][path_x - 1]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
        path_y--;
        path_y--;
      }
    }
    else{
      if(D_acc[path_y - 1][path_x - 2] <= D_acc[path_y - 2][path_x - 1]){
        path_x--;
        path_x--;
        path_y--;
      }
      else{
        path_x--;
        path_y--;
        path_y--;
      }
    }

    add_point_to_path_front(path_x, path_y);
  }
}

std::deque< tuple > warp::get_path(){
  return path;
}

void warp::print_path(){
  std::cout<<"* Warping path: "<<std::endl;
  for(unsigned int i=0; i < path.size(); i++){
    std::cout << "* x: " << path[i].x << "* y: " << path[i].y << std::endl;
  }
}


// Walkes through the accumulated cost matrix M from bottom right to top left, finds the best matching way through the two vectors (deletes not fitting entries in the vectors)
// This algorithm uses the Sakoe-Chiba band.

void warp::find_warping_path_Sakoe_Chiba(){

  int path_x = get_D_acc_cols() -1;
  int path_y = get_D_acc_rows() -1;       // Starting position of the warping path

  int T = (int) ( 0.5 + ( path_x / 5.0));      // Defining the Sakoe Chiba band as a 5th of the matrix size (asuming it to be quadratic)

  add_point_to_path_front(path_x , path_y);


  while((path_x > 0) || (path_y > 0)){  // End and target position is x;y  0;0

    // choosing from 3 close neighbours the cheapest way

    if(path_x == 0){ // new position of path is clear
      path_y--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    if(path_y == 0){ // new position of path is clear
      path_x--;
      add_point_to_path_front(path_x , path_y);
      continue;
    }

    // being at the upper border of the Sakoe Chiba band, walking in the x direction is not any more possible

    if(path_x -1  <=  path_y - T){
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 2][path_x - 1]){
        path_y--;
        path_x--;
      }
      else{
        path_y--;
        path_y--;
        path_x--;
      }
           add_point_to_path_front(path_x, path_y);
      continue;
    }


    // being at the lower border of the Sakoe Chiba band, walking in the y direction is not any more possible

    if(path_y -1  <= path_x - T){
      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 2]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
        path_x--;
        path_y--;
      }
      add_point_to_path_front(path_x, path_y);
      continue;
    }


    // find the cheapest of all three:

    if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 1][path_x - 2]){

      if(D_acc[path_y - 1][path_x - 1] <= D_acc[path_y - 2][path_x - 1]){
        path_y--;
        path_x--;
      }
      else{
        path_x--;
        path_y--;
        path_y--;
      }
    }
    else{
      if(D_acc[path_y - 1][path_x - 2] <= D_acc[path_y - 2][path_x - 1]){
        path_x--;
        path_x--;
        path_y--;
      }
      else{
        path_x--;
        path_y--;
        path_y--;
      }
    }

    add_point_to_path_front(path_x, path_y);
  }
}

double warp::get_path_costs(){
  return path_costs;
}

double warp::path_cost(){
  double cost = 0;
  for(unsigned int i=0; i < path.size(); i++){
    cost += D[path[i].y][path[i].x];
  }
  path_costs = cost;
  return cost;
}

std::deque< std::deque< double > > warp::get_D(){
  return D;
}

std::deque< std::deque< double > > warp::get_D_acc(){
  return D_acc;
}
