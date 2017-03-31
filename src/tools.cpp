#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size() )
  {
    std::cout << "Invalid estimation or ground_truth data" << std::endl; 
    return rmse;
  }
  for (int i=0; i< estimations.size();++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
     
    //Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
	
  }
  
  //calulate the mean
  rmse = rmse/estimations.size();
  
  //calculate the sqrt
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameteres
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px_2 = px*px;
  float py_2 = py*py;
  float p2sum = px_2 + py_2;
  float psqrt = sqrt(p2sum);
  float p3_2 = p2sum * psqrt;

  //check division by zero
  if (fabs(p2sum) < 0.0001)
  {
    std::cout << "Denomination cannot be zero" << std::endl;
    Hj << 0, 0, 0, 0,
         -0, 0, 0, 0,
          0, 0, 0, 0;
    return Hj;
  }
  Hj << (px/psqrt), (py/psqrt),0,0,
       -(py/p2sum), (px/psqrt),0,0,
        py*(vx*py-vy*px)/p3_2,px*(vy*px-vx*py)/p3_2,px/psqrt,py/psqrt;

  return Hj;
}

