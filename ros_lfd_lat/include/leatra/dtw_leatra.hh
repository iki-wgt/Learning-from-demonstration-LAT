#ifndef DTW_LEATRA_H
#define DTW_LEATRA_H

#include <deque>
#include <math.h>
#include <limits>
#include "leatra.hh"
#include "dtw2.hh"

class warp_leatra{

public:

  std::deque< trajectory_lat > warp_in_task_space(std::deque< trajectory_lat > JS);
  bool warp_trajectories(std::deque< trajectory_lat > *tra);
};

#endif
