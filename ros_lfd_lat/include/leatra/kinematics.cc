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

Matrix<double, Eigen::Dynamic, 3> Jpinv_s(Matrix<double, 3, Eigen::Dynamic> J){
    
  JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
  svd.computeU();	
  svd.computeV();

  int m = -1;

  if(J.rows() < J.cols())
  {
	  m = J.rows();
  }
  else
  {
	  m = J.cols();
  }


  Matrix<double, 3, 3> U;
  Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
  Matrix<double, 1, Eigen::Dynamic> _S(m);
  Matrix<double, Eigen::Dynamic, 3> S = Matrix<double, Eigen::Dynamic, 3>::Zero(J.cols(), 3);
  
  U = svd.matrixU();
  V = svd.matrixV();
  _S = svd.singularValues();

  for (int i = 0; i < m; ++i) {
	  S(i, i) = 1 / _S(i);
  }


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
	unsigned int tra_size = (*TM)[0].size();
	unsigned int dofs = JM->size();
	unsigned int dofJac = dofs - 2;		// exclude the two angles from the gripper	//TODO: Katana specific
	int limit_violation = 0;
	Matrix<double, 5,2> limit;	//TODO: these limits have to be read out of the robot description
  limit <<   -3.025528, 2.891097,
		  -0.135228, 2.168572,
		  -2.221804, 2.054223,
		  -2.033309, 1.87613,
		  -2.993240, 2.870985;

	// from here on, all trajectories have the correct dimensions.

	// the first point in the new trajectory is the first mean of JM
	for(unsigned int i = 0; i < dofs; i++){		//TODO: Katana specific
		(*LAT)[i].push_back( (*JM)[i][0] );
	}

	// Get the root and tip link names from parameter server.
	  std::string root_name = "katana_base_link";
	  std::string tip_name = "katana_gripper_tool_frame";
	  /*if (!ros::param::get("/root_name", root_name))
	  {
		ROS_ERROR("No root_name specified!");
		return false;
	  }
	  if (!ros::param::get("/tip_name", tip_name))
	  {
		ROS_ERROR("No tip name specified!");
		return false;
	  }*/

	// create KDL stuff for Jacobian and forward kinematics
	KDL::Tree my_tree;

	// set up robot description
	std::string robot_desc_string;
	if(!ros::param::get("robot_description", robot_desc_string))
	{
		ROS_ERROR("Failed to retrieve robot description!");
		return false;
	}
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	// Create solver based on kinematic tree
	KDL::TreeFkSolverPos_recursive fksolver = KDL::TreeFkSolverPos_recursive(my_tree);

	KDL::Chain chain;		// create chain
	my_tree.getChain(root_name, tip_name, chain);

	KDL::ChainJntToJacSolver jacSolver = KDL::ChainJntToJacSolver(chain);

	// Having the first point of the new trajectory set above, now all the other points
	// are calculated in the loop below:
	for(unsigned int pointNo = 1; pointNo < tra_size; pointNo++){

		// initializing all joint space variables:
		VectorXd theta_old(dofJac);
		VectorXd d_theta(dofJac);
		KDL::JntArray jntPositionsJac = KDL::JntArray(dofJac);

		for(unsigned int jointNo = 0; jointNo < dofJac; jointNo++){
			theta_old(jointNo) = (*LAT)[jointNo][pointNo-1];
			jntPositionsJac(jointNo) = (*LAT)[jointNo][pointNo-1];
			d_theta(jointNo) = (*JM)[jointNo][pointNo] - (*LAT)[jointNo][pointNo-1];
		}

		KDL::JntArray jntPositions = KDL::JntArray(dofs);

		//copy joint positions from matrix to JntArray
		for (unsigned int k = 0; k < dofs; ++k) {
			jntPositions(k) = (*LAT)[k][pointNo-1];
		}

		// Create the frame that will contain the results
		KDL::Frame cartpos;

		// initializing all task space variables:
		Matrix<double, 3, 1> d_x;

		Matrix<double, 3, 1> x_old;

		// Calculate forward position kinematics
		int kinematics_status;
		kinematics_status = fksolver.JntToCart(jntPositions, cartpos, tip_name);
		if(kinematics_status>=0){
			// success now copy the result
			for (unsigned int i = 0; i < 3; ++i) {
				x_old(i) = cartpos.p[i];
			}

		}
		else
		{
			ROS_ERROR("Could not calculate forward kinematics! Error code: %d, no of joints: %d, jointpositions: %d", kinematics_status, my_tree.getNrOfJoints(), jntPositions.rows());
		}

		for(int j = 0; j < d_x.rows(); j++){
			d_x(j) = (*TM)[j][pointNo] - x_old(j);
		}

		// create Jacobian

		KDL::Jacobian kdlJacobian(dofJac);

		jacSolver.JntToJac(jntPositionsJac, kdlJacobian);

		Matrix<double, 3, Eigen::Dynamic> J = kdlJacobian.data.topRows(3);

		Matrix<double, Eigen::Dynamic, 3> Jinv;
		Jinv = Jpinv_s(J);

		MatrixXd I = MatrixXd::Identity(dofJac, dofJac);

		VectorXd theta_new(dofJac);

		MatrixXd alpha = MatrixXd::Identity(dofJac, dofJac);
		//alpha = alpha * 1.0;

		theta_new =  theta_old + Jinv * d_x + alpha * ((I - (Jinv * J)) * d_theta);

		/*std::cout << "theta_old\n" << theta_old << "\n" << std::endl;
		std::cout << "d_x\n" << d_x << "\n" << std::endl;
		std::cout << "J\n" << J << "\n" << std::endl;
		std::cout << "Jinv\n" << Jinv << "\n" << std::endl;
		std::cout << "Leatra Jacobi\n" << Jacobi_lat_s(theta_old.topRows(4)) << "\n" << std::endl;
		VectorXd theta_tmp(dofJac);
		theta_tmp = Jinv * d_x;
		std::cout << "Jinv * d_x\n" << theta_tmp << "\n" << std::endl;
		theta_tmp = alpha * ((I - (Jinv * J)) * d_theta);
		std::cout << "alpha * ((I - (Jinv * J)) * d_theta)\n" << theta_tmp << "\n" << std::endl;*/

		for(unsigned int jointNo = 0; jointNo < dofJac; jointNo++){
			(*LAT)[jointNo].push_back( (double)theta_new(jointNo) );
			if((*LAT)[jointNo][pointNo] > limit(jointNo,1)){
				ROS_WARN("Limit violation value: %f, limit: %f, joint: %d, step: %d",
						(*LAT)[jointNo][pointNo],
						limit(jointNo,1),
						jointNo,
						pointNo);
				(*LAT)[jointNo][pointNo] = limit(jointNo,1);
				limit_violation++;
       }
       if((*LAT)[jointNo][pointNo] < limit(jointNo,0)){
    	   ROS_WARN("Limit violation value: %f, limit: %f, joint: %d, step: %d",
    			   (*LAT)[jointNo][pointNo],
    			   limit(jointNo,0),
    			   jointNo,
    			   pointNo);

    	   (*LAT)[jointNo][pointNo] = limit(jointNo,0);
    	   limit_violation++;
       }
	  }

		// Adding the 6th and 7th angle: the 6th angle from the mean of JM[5][i]
		(*LAT)[5].push_back( (*JM)[5][pointNo] );	//TODO: Katana specific!
		(*LAT)[6].push_back( (*JM)[6][pointNo] );
	}

	MatrixXd LAT_tmp(dofJac,tra_size - 1);
	for(unsigned int jointNo = 0; jointNo < dofJac; jointNo++){
		LAT_tmp(jointNo,0) = (*LAT)[jointNo][0];
	}

	// flatening LAT the angles that were computed with the Jacobian
	for(unsigned int jointNo = 0; jointNo < dofJac; jointNo++){
		for(unsigned int pointNo = 1; pointNo < tra_size - 1; pointNo++){
			LAT_tmp(jointNo,pointNo) = (*LAT)[jointNo][pointNo];
			(*LAT)[jointNo][pointNo] = (LAT_tmp(jointNo,pointNo)
					+ (((*LAT)[jointNo][pointNo+1]+LAT_tmp(jointNo,pointNo-1)) / 2) )/ 2;
		}
	}

	const unsigned int THINNING_FACTOR = 10; //use only every 10th value
	size_t newSize = tra_size / THINNING_FACTOR;

	// thinning
	for (size_t jointNo = 0; jointNo < dofs; ++jointNo)
	{
		std::deque<double> thinnedPoints(newSize);

		for (size_t pointNo = 0; pointNo < newSize; ++pointNo)
		{
			thinnedPoints[pointNo] = LAT->at(jointNo)[pointNo * THINNING_FACTOR];
			// pointNo * THINNING_FACTOR is never bigger than tra_size because
			// tra_size / THINNING_FACTOR rounds down
		}
		// replace old LAT by the thinned one
		(*LAT)[jointNo] = thinnedPoints;
	}


	std::cout << "From "<< tra_size << " points there have been " << limit_violation << " limit violations! And " << nan_count << " nans." << std::endl;

	return true;
}
