#include <iostream>
#include "dtw_leatra.hh"
#include "litera.hh"
#include "kinematics.hh"

/**
 *      The method warp_in_task_space takes a trajectory in joint space and applies to it
 *      a warping in task space.
 */
std::deque< trajectory_lat > warp_leatra::warp_in_task_space(std::deque< trajectory_lat > JS){ 
 
  boost::timer timer;
  // Create a list of trajectories in task space
  std::deque< trajectory_lat > TS( JS.begin(), JS.end());
  for(unsigned int i=0; i < TS.size(); i++){
    TS[i].joint_to_task_space();
  }
  
  ROS_INFO("Create a list of trajectories in task space %f", timer.elapsed());
  timer.restart();

  // warp trajectories in task space
  // warping all permutations of trajectories and looking for the cheapest warp

  std::deque< std::deque< warp > > W;

  // preparing the sequences, that should be warped (for each trajectory one sequence):
  std::deque< std::deque< double > > xy;

  for(unsigned int i = 0; i < TS.size(); i++){
    xy.push_back( TS[i].get_euclide() );
  }
  
  ROS_INFO("Filling xy %f", timer.elapsed());
  timer.restart();

  // warping all possible pairs -> creating a triangular matrix, for example with 6 trajectories:
  //
  //     0-1  1-2  2-3  3-4  4-5
  //     0-2  1-3  2-4  3-5
  //     0-3  1-4  2-5
  //     0-4  1-5
  //     0-5
  //
  for(unsigned int i = 0; i < TS.size() - 1; i++){
    std::deque< warp > w_row;
    boost::timer timer2;
    for(unsigned int j = i + 1; j < TS.size(); j++){
      warp tmp;
      ROS_INFO("warp tmp %f", timer2.elapsed());
      timer2.restart();
      tmp.set_x(xy[i]);
      ROS_INFO("set x %f", timer2.elapsed());
      timer2.restart();
      tmp.set_y(xy[j]);
      ROS_INFO("set y %f", timer2.elapsed());
            timer2.restart();
      tmp.make_distance_matrix_slope();
      ROS_INFO("make distance matrix %f", timer2.elapsed());
            timer2.restart();
      tmp.accumulate_distance_matrix();
      ROS_INFO("accumulate distance matrix %f", timer2.elapsed());
            timer2.restart();
      tmp.find_warping_path_classic_adapted();
      ROS_INFO("find warping path %f", timer2.elapsed());
            timer2.restart();
      tmp.path_cost();
      ROS_INFO("path cost %f", timer2.elapsed());
            timer2.restart();
      w_row.push_back(tmp);
      ROS_INFO("push back %f", timer2.elapsed());
            timer2.restart();
    }
    W.push_back(w_row);
  }

  ROS_INFO("Filling W %f", timer.elapsed());
  timer.restart();

  // searching for the cheapest warp
  int first_i = 0;
  int first_j = 0;

  int last_i = 0;  // also looking for the most expensive path
  int last_j = 0;

  for(unsigned int i=0; i < W.size(); i++){
    for(unsigned int j=0; j < W[i].size(); j++){
      if(W[first_i][first_j].get_path_costs() > W[i][j].get_path_costs()){
        first_i = i;
        first_j = j;
      }
      if(W[last_i][last_j].get_path_costs() < W[i][j].get_path_costs()){
        last_i = i;
        last_j = j;
      }
    }
  }

  ROS_INFO("Searching the cheapest path %f", timer.elapsed());
  timer.restart();

  // first_i and first_j point to a warp made out of two trajectories and those are:
  int t1 = first_i;
  int t2 = first_j + (first_i + 1);

  // Initializing the second best with the worst values
  int second_i = last_i;
  int second_j = last_j;

  // Looking for the second cheapest among the trajectory t1
  if((int)(W.size()) > t1){
    for(int i=0; i < (int)(W[t1].size()); i++){
      if(W[t1][i].get_path_costs() <= W[second_i][second_j].get_path_costs()){
        if(! (t1 == first_i && i == first_j)){
          second_i = t1;
          second_j = i;
        }
      }
    }
  }

  for(int i=0; i < (int)(W.size()); i++){
    if(t1 - (i+1) > -1){
      if(W[i][t1 - (i + 1)].get_path_costs() <= W[second_i][second_j].get_path_costs()){
        if(! (i == first_i && (t1-(i + 1)) == first_j)){
          second_i = i;
          second_j = t1 - (i + 1);
        }
      }
    }
  }

  ROS_INFO("Looking for the second cheapest t1 %f", timer.elapsed());
    timer.restart();

  // Looking for the second cheapest among the trajectory t2
  if((int)(W.size()) > t2){
    for(int i=0; i < (int)(W[t2].size()); i++){
      if(W[t2][i].get_path_costs() <= W[second_i][second_j].get_path_costs()){
        if(! (t2 == first_i && i == first_j)){
          second_i = t2;
          second_j = i;
        }
      }
    }
  }

  for(int i=0; i < (int)(W.size()); i++){
    if(t2 - (i+1) > -1){
      if(W[i][t2 - (i + 1)].get_path_costs() <= W[second_i][second_j].get_path_costs()){
        if(! (i == first_i && (t2-(i + 1)) == first_j)){
          second_i = i;
          second_j = t2 - (i + 1);
        }
      }
    }
  }

  int t3 = second_i;
  int t4 = second_j + (second_i + 1);

  ROS_INFO("Looking for the second cheapest t2 %f", timer.elapsed());
  timer.restart();

  // choosing the strongest trajectory and aligning all paths according to it:
  // t is the reference trajectory
  int t;

  if(t1 == t3 || t1 == t4){
    t = t1;
  }
  else{
    t = t2;
  }

  // warping all possible pairs -> creating a triangular matrix, for example with 6 trajectories:
  //
  //     0-1  1-2  2-3  3-4  4-5
  //     0-2  1-3  2-4  3-5
  //     0-3  1-4  2-5
  //     0-4  1-5
  //     0-5
  //
  
 
 //std::cout << "Normalizing for Trajectory " << t << std::endl << std::endl;
 
  // Normalizing all paths on t:
  if((int)(W.size()) > t){
    for(unsigned int i=0; i < W[t].size(); i++){
      W[t][i].normalize_path_for_x();
      //std::cout<< "NORMALIZING for x: W[" << t << "][" << i << "]" << std::endl;
    }
  }
  for(int i=0; i < (int)(W.size()); i++){
    if(t - (i+1) > -1){
      W[i][t - (i + 1)].normalize_path_for_y();
      //std::cout<< "NORMALIZING for y: W[" << i << "][" << t - (i + 1) << "]" << std::endl;
    }
  }
  
  ROS_INFO("Warping all possible pairs %f", timer.elapsed());
  timer.restart();

  // making the master warping path:
  std::deque< std::deque< int > > master_path;

  for(int i = 0; i < t; i++){
    master_path.push_back(W[i][t-(i+1)].get_path_x());  
    //std::cout<< "path number " << i << " from W[" << i << "][" << t - (i + 1) << "]" << " size = " << W[i][t-(i+1)].get_path_x().size() << std::endl;
  }
  if(t != 0){
    master_path.push_back(W[t-1][0].get_path_y());
    //std::cout<< "path number " << t << " from W[" << t-1 << "][" << 0 << "]" << " size = " << W[t-1][0].get_path_y().size() << std::endl;
  } else{
    master_path.push_back(W[t][0].get_path_x());   
    //std::cout<< "path number " << t << " from W[" << t << "][" << 0 << "]" << " size = " << W[t][0].get_path_x().size() << std::endl;
  }
  if(t < (int)(W.size())){
    for(unsigned int i = 0; i < W[t].size(); i++){
      master_path.push_back(W[t][i].get_path_y());
      //std::cout<< "path number " << i+t+1 << " from W[" << t << "][" << i << "]" << " size = " << W[t][i].get_path_y().size() << std::endl;
    }
  }

  ROS_INFO("Making the master warping path %f", timer.elapsed());
  timer.restart();
   
  // apply warping path to all trajectories in JS
  
  for(unsigned int i=0; i < JS.size(); i++){
    JS[i].create_this_sequence( master_path[i] );
  }
  
  ROS_INFO("Apply warping path %f", timer.elapsed());

  return JS;
}






bool warp_leatra::warp_trajectories(std::deque< trajectory_lat > *tra){

  std::deque< double > x = (*tra)[0].get_ndmap().get_row(0);
  std::deque< double > y = (*tra)[1].get_ndmap().get_row(0);

  warp W;
  W.set_x(x);
  W.set_y(y);
  W.make_distance_matrix_slope();
 // W.accumulate_distance_matrix_step_size_cond2();
  W.accumulate_distance_matrix();
  W.info();

 // W.find_warping_path_Sakoe_Chiba();
  W.find_warping_path_classic_adapted();

    std::cout<<"WARP 2" << std::endl;

  W.print_path();

  std::deque< tuple > path = W.get_path();

  std::deque< double > rowA, rowB ;
  for(unsigned int i=0; i < path.size(); i++){
    rowA.push_back(x[path[i].x]);
    rowB.push_back(y[path[i].y]);
  }

  (*tra)[0].map.map[0] = rowA;
  (*tra)[1].map.map[0] = rowB;

  std::cout<<"Path costs = "<< W.path_cost() << std::endl;

  ndmap map;
  map.set_name("D5");
  std::deque< std::deque< double> > D = W.get_D();
  for(unsigned int i = 0; i < path.size(); i++){
    D[path[i].y][path[i].x] = 0.3;
  /* if((path[i].y + 1  <  D.size()) && (path[i].x + 1  <  D[path[i].x].size()) && (path[i].x - 1 > 0) && (path[i].y - 1 > 0)){
      D[path[i].y - 1][path[i].x - 1] = 0.3;
      D[path[i].y - 1][path[i].x]= 0.3;
      D[path[i].y][path[i].x - 1]= 0.3;
      D[path[i].y + 1][path[i].x + 1]= 0.3;
      D[path[i].y + 1][path[i].x]= 0.3;
      D[path[i].y][path[i].x + 1]= 0.3;
    }
 */
  }
  map.set_deque(D);

  litera lit;
  lit.write_ndmap(map);

  map.set_name("D_acc5");

  std::deque< std::deque< double> > D_acc = W.get_D_acc();
  for(unsigned int i = 0; i < path.size(); i++){
    D_acc[path[i].y][path[i].x] = 12.0;
 /*   if((path[i].y + 1  <  D_acc.size()) && (path[i].x + 1  <  D_acc[path[i].x].size()) && (path[i].x - 1 > 0) && (path[i].y - 1 > 0)){
      D_acc[path[i].y - 1][path[i].x - 1] = 12.0;
      D_acc[path[i].y - 1][path[i].x]= 12.0;
      D_acc[path[i].y][path[i].x - 1]= 12.0;
      D_acc[path[i].y + 1][path[i].x + 1]= 12.0;
      D_acc[path[i].y + 1][path[i].x]= 12.0;
      D_acc[path[i].y][path[i].x + 1]= 12.0;
    }
  */
  }
  map.set_deque(D_acc);

  lit.write_ndmap(map);



  return true;
}
