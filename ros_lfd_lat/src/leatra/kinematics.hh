#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <deque>
#include "math.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Eigen>

// include these headers to switch from leatra direct kinematics to kdl
#include "ros/ros.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/utilities/utility.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

#define Pi 3.1415926535898


/**
 *  Inverse Kinematics
 *
 *  parameters:
 *     theta_t0 = current joint space angles
 *     x_t0     = current task space coordinates
 *     y_t0     = desired task space coordinates
 *  return value:
 *     theta_t1 = calculated (desired) joint space angles
 */
std::deque< double > IK_lat(std::deque< double > theta_t0, std::deque< double > x_t0, std::deque< double > x_t1);

/**
 *  Direct Kinematics
 *
 *  parameters:
 *     angles = current joint space angles (5 angles)
 *  return value:
 *     X = task space coordinates (x,y,z) [without orientation]
 */
std::deque< double > DK_lat(std::deque< double > angles);

/**
 *  Optimisation Kinematics
 *
 *  Optimisation of a trajectory in joint space and a trajectory in task space
 *  
 *  parameters:
 *      All parameters are pointers:
 *      LAT: here the result must be stored in
 *      TM: mean of the task space trajectory
 *      TS: standard deviation of the task space trajectory
 *      JM: mean of the joint space trajectory
 *      JS: standard deviation of the joint space trajectory 
 */
bool optimize_TJ(std::deque< std::deque<double> >* LAT, 
                std::deque< std::deque<double> >*  TM, std::deque< std::deque<double> >* TS , 
                std::deque< std::deque<double> >*  JM, std::deque< std::deque<double> >* JS);


#endif
