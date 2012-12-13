#include "kinematics.hh"

// katana 6M180 configuration
const double L1=190.0;
const double L2=139.0;
const double L3=147.3;
const double L4=166.0;

int nan_count;


using namespace Eigen;



Matrix<double, 3, 5> Jacobi_lat(std::deque< double > theta){

  const double t1=theta[0];
  const double t2=theta[1];
  const double t3=theta[2];
  const double t4=theta[3];

  Matrix<double, 3, 5> J;

  J(0,0) = -sin(t1) * (-190.0 * cos(t2) + 139.0 * cos(t2 + t3) - 3133.0 / 10.0 * cos(t2 + t3 - t4));
  J(0,1) = cos(t1) * (190.0 * sin(t2) - 139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(0,2) = cos(t1) * (-139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(0,3) = -3133.0 / 10.0 * cos(t1) * sin(t2 + t3 - t4);
  J(0,4) = 0;

  J(1,0) = cos(t1) * (-190.0 * cos(t2) + 139.0 * cos(t2 + t3) - 3133.0 / 10.0 * cos(t2 + t3 - t4));
  J(1,1) = sin(t1) * (190.0 * sin(t2) - 139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(1,2) = sin(t1) * (-139.0 * sin(t2 + t3) + 3133. / 10.0 * sin(t2 + t3 - t4));
  J(1,3) = -3133.0 / 10.0 * sin(t1) * sin(t2 + t3 - t4);
  J(1,4) = 0;

  J(2,0) = 0;
  J(2,1) = 190.0 * cos(t2) - 139.0 * cos(t2 + t3) + 3133.0 / 10.0 * cos(t2 + t3 - t4);
  J(2,2) = -139.0 * cos(t2 + t3) + 3133.0 / 10.0 * cos(t2 + t3 - t4);
  J(2,3) = -3133.0 / 10.0 * cos(t2 + t3 - t4);
  J(2,4) = 0;

  return J;
}




Matrix<double, 3, 4> Jacobi_lat_s(Matrix<double, 4, 1> theta){

  const double t1=theta(0);
  const double t2=theta(1);
  const double t3=theta(2);
  const double t4=theta(3);

  Matrix<double, 3, 4> J;

  J(0,0) = -sin(t1) * (-190.0 * cos(t2) + 139.0 * cos(t2 + t3) - 3133.0 / 10.0 * cos(t2 + t3 - t4));
  J(0,1) = cos(t1) * (190.0 * sin(t2) - 139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(0,2) = cos(t1) * (-139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(0,3) = -3133.0 / 10.0 * cos(t1) * sin(t2 + t3 - t4);

  J(1,0) = cos(t1) * (-190.0 * cos(t2) + 139.0 * cos(t2 + t3) - 3133.0 / 10.0 * cos(t2 + t3 - t4));
  J(1,1) = sin(t1) * (190.0 * sin(t2) - 139.0 * sin(t2 + t3) + 3133.0 / 10.0 * sin(t2 + t3 - t4));
  J(1,2) = sin(t1) * (-139.0 * sin(t2 + t3) + 3133. / 10.0 * sin(t2 + t3 - t4));
  J(1,3) = -3133.0 / 10.0 * sin(t1) * sin(t2 + t3 - t4);

  J(2,0) = 0;
  J(2,1) = 190.0 * cos(t2) - 139.0 * cos(t2 + t3) + 3133.0 / 10.0 * cos(t2 + t3 - t4);
  J(2,2) = -139.0 * cos(t2 + t3) + 3133.0 / 10.0 * cos(t2 + t3 - t4);
  J(2,3) = -3133.0 / 10.0 * cos(t2 + t3 - t4);

  return J;
}

Matrix<double, 4, 3> Jpinv_s(Matrix<double, 3, 4> J){
    
  JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
  svd.computeU();	
  svd.computeV();	

  Matrix<double, 3, 3> U;
  Matrix<double, 4, 4> V;
  Matrix<double, 1, 3> _S;
  Matrix<double, 4, 3> S = Matrix<double, 4, 3>::Zero();
  
  U = svd.matrixU();
  V = svd.matrixV();
  _S = svd.singularValues();
  S(0,0) = 1 / _S(0);
  S(1,1) = 1 / _S(1);
  S(2,2) = 1 / _S(2);

  return V * S * U.transpose();
}
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

std::deque< double > IK_lat(std::deque< double > theta_t0, std::deque< double > x_t0, std::deque< double > x_t1){

  MatrixXf m(3,5);
  m = Jacobi_lat(theta_t0).cast<float>();

  //std::cout<< std::endl << "Matrix J = \n" << m << std::endl;

  //MatrixXf m = MatrixXf::Random(3,2);
  //std::cout << "Here is the matrix m:" << std::endl << m << std::endl;
  JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
  //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
  //std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
  //std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;
  Vector3f rhs(x_t0[0] - x_t1[0], x_t0[1] - x_t1[1], x_t0[2] - x_t1[2]);
  //std::cout << "Now consider this rhsmath.h vector:" << std::endl << rhs << std::endl;
  VectorXf Dtheta(5);
  Dtheta = svd.solve(rhs);
  //std::cout << "A least-squares solution of m*x = rhs is:" << std::endl << Dtheta << std::endl;

  std::deque< double > theta_t1;

//  std::cout<< "Dtheta.size() = " << Dtheta.size() <<std::endl;
//  std::cout<< "theta_t0.size() = " << theta_t0.size() <<std::endl;

  for(unsigned int i=0; i < theta_t0.size(); i++)
    theta_t1.push_back(theta_t0[i] - (double)Dtheta(i));

  return theta_t1;
}



/**
 *  Direct Kinematics
 *
 *  parameters:
 *     angles = current joint space angles (5 angles)
 *  return value:
 *     X = task space coordinates (x,y,z) [without orientation]
 */
std::deque< double > DK_lat(std::deque< double > angles){

  // correct encoder values
  std::deque< double > theta = angles;
  theta[1] = theta[1] - Pi / 2.0;
  theta[2] = theta[2] - Pi;
  theta[3] = Pi - theta[3];
  theta[4] = -theta[4];

  theta[2] = theta[1] + theta[2];
  theta[3] = theta[2] + theta[3];

  // calculation
  double factor = L1 * sin(theta[1]) + L2 * sin(theta[2]) + ( L3 + L4 ) * sin(theta[3]);

  // compare homogenous transformation matrix
  std::deque< double > X(3,0);
  X[0] = cos(theta[0]) * factor;
  X[1] = sin(theta[0]) * factor;
  X[2] = L1 * cos(theta[1]) + L2 * cos(theta[2]) + ( L3 + L4 ) * cos(theta[3]);

  return X;
}
/**
 *  Direct Kinematics
 *
 *  parameters:
 *     angles = current joint space angles (5 angles)
 *  return value:
 *     X = task space coordinates (x,y,z) [without orientation]
 */
Matrix<double, 3, 1>  DK_lat(Matrix<double, 4, 1>  angles){

  // correct encoder values
  Matrix<double, 4, 1> theta = angles;
  theta(1) = theta(1) - Pi / 2.0;
  theta(2) = theta(2) - Pi;
  theta(3) = Pi - theta(3);

  theta(2) = theta(1) + theta(2);
  theta(3) = theta(2) + theta(3);

  // calculation
  double factor = L1 * sin((double)theta(1)) + L2 * sin((double)theta(2)) + ( L3 + L4 ) * sin((double)theta(3));

  // compare homogenous transformation matrix
  Matrix<double, 3, 1> X;
  X(0) = cos((double)theta(0)) * factor;
  X(1) = sin((double)theta(0)) * factor;
  X(2) = L1 * cos((double)theta(1)) + L2 * cos((double)theta(2)) + ( L3 + L4 ) * cos((double)theta(3));

  return X;
}



/**
 *  Optimisation Kinematics 
 *
 *  Optimisation of a trajectory in joint space and a trajectory in task space:
 *  Creating an optimized joint space trajectory and writing it to LAT.
 *
 *  parameters:
 *      All parameters are pointers:
 *      LAT: here the result must be stored in
 *      ANGLE: current angles of the robot
 *      TM: mean of the task space trajectory
 *      TS: standard deviation of the task space trajectory
 *      JM: mean of the joint space trajectory
 *      JS: standard deviation of the joint space trajectory
 */
bool optimize_TJ(std::deque< std::deque<double> >* LAT,
                std::deque< std::deque<double> >*  TM, std::deque< std::deque<double> >* TS ,
                std::deque< std::deque<double> >*  JM, std::deque< std::deque<double> >* JS){

  nan_count = 0;

  // test if all measures are as expected:
  if((TM->size() != 3 || TS->size() != 3) ||(JM->size() != 6 || JS->size() != 6)) return false;
  if(LAT->size() != 6) return false;
  
  unsigned int tra_size = (*TM)[0].size();
  for(unsigned int i = 0; i < TM->size(); i++){
    if((*TM)[i].size() != tra_size)return false;
    if((*TS)[i].size() != tra_size)return false;
  }

  for(unsigned int i = 0; i < JM->size(); i++){
    if((*JM)[i].size() != tra_size)return false;
    if((*JS)[i].size() != tra_size)return false;
  }

  
  int limit_violation = 0;
  Matrix<double, 4,2> limit;
  limit <<   0.12, 6.15,
	    -0.20, 2.15,
             0.93, 5.34,
             1.12, 5.15;
  
  // from here on, all trajectories have the correct dimensions.
	     
  // the first point in the new trajectory is the first mean of JM
  for(int i = 0; i < 6; i++){
    (*LAT)[i].push_back( (*JM)[i][0] );
  }

  // Having the first point of the new trajectory set above, now all the other points
  // are calculated in the loop below:
  for(unsigned int i = 1; i < tra_size; i++){

    // initializing all joint space variables:
    Matrix<double, 4, 1> theta_old;
    Matrix<double, 4, 1> d_theta;

    for(int j = 0; j < 4; j++){
      theta_old(j) = (*LAT)[j][i-1];  
      d_theta(j) = (*JM)[j][i] - (*LAT)[j][i-1];
    }
    
    // initializing all task space variables:
    Matrix<double, 3, 1> d_x;

   Matrix<double, 3, 1> x_old;
   x_old = DK_lat(theta_old);
    
    for(int j = 0; j < 3; j++){    
      d_x(j) = (*TM)[j][i] - x_old(j);
    }

    Matrix<double, 3, 4> J;
    J = Jacobi_lat_s(theta_old);
    
    Matrix<double, 4, 3> Jinv;
    Jinv = Jpinv_s(J);
    
    Matrix<double, 4, 4> I = Matrix<double, 4, 4>::Identity();
    
    Matrix<double, 4, 1> theta_new;
    
    Matrix<double, 4, 4> alpha = Matrix<double, 4, 4>::Identity();
    
    //std::cout << alpha <<std::endl;

    theta_new =  theta_old + Jinv * d_x + alpha * ((I - (Jinv * J)) * d_theta);

    for(int j = 0; j < 4; j++){
      (*LAT)[j].push_back( (double)theta_new(j) );
       if((*LAT)[j][i] > limit(j,1)){ 
	 (*LAT)[j][i] = limit(j,1);
	 limit_violation++;
       }
       if((*LAT)[j][i] < limit(j,0)){  
	 (*LAT)[j][i] = limit(j,0);
	 limit_violation++;
       } 
    }

    // Adding the 5th and 6th angle: the 6th angle from the mean of JM[5][i]
    (*LAT)[4].push_back( (*JM)[4][i] );
    (*LAT)[5].push_back( (*JM)[5][i] );
  }
  
  MatrixXd LAT_tmp(4,tra_size - 1);
  for(int i = 0; i < 4; i++){
    LAT_tmp(i,0) = (*LAT)[i][0];
  }
  // flatening LAT the first four angles
  for(int i = 0; i < 4; i++){
    for(unsigned int j = 1; j < tra_size - 1; j++){
        LAT_tmp(i,j) = (*LAT)[i][j];
       (*LAT)[i][j] = (LAT_tmp(i,j) + (((*LAT)[i][j+1]+LAT_tmp(i,j-1)) / 2) )/ 2;
    }
  }

  std::cout << "From "<< tra_size << " points there have been " << limit_violation << " limit violations! And " << nan_count << " nans." << std::endl;
  
  return true;
}
