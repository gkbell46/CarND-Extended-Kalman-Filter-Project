#include "kalman_filter.h"
#include <iostream>


MatrixXd I; // Identity matrix

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  
  x_ = (F_ * x_)  ;  
  P_ = (F_ * P_ * F_.transpose()) + Q_;
  //std::cout << "x_:" << std::endl; std::cout << x_ << std::endl;
  //std::cout << "P_:" << std::endl; std::cout << P_ << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  MatrixXd y = z - (H_ * x_);
  //std::cout << "y:" << std::endl; std::cout << y << std::endl;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S =  (H_ * PHt)  + R_;
  //std::cout << "S:" << std::endl; std::cout << S << std::endl;
  MatrixXd k = PHt * S.inverse();
  //std::cout << "k:" << std::endl; std::cout << k << std::endl;
  
  
  I = MatrixXd::Identity(x_.size(),x_.size());
  //std::cout << "I:" << std::endl; std::cout << I << std::endl;
  // new state
  x_ = x_ + (k * y);
  //std::cout << "x_:" << std::endl; std::cout << x_ << std::endl;
  P_ = (I - (k * H_)) * P_;
  //std::cout << "P_:" << std::endl; std::cout << P_ << std::endl;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  //std::cout << "z(0)" << std::endl; std::cout << x_ << std::endl;
  float hx_1  = sqrt((px * px) + (py * py));
  float hx_2 = atan2(py,px);
  float hx_3 = ((px * vx) + (py * vy)) / hx_1; 
  VectorXd hx_ = VectorXd(3);
  if (hx_1 !=0 )
  {
  	hx_ << hx_1, 	
  	       hx_2, 
  	       hx_3;
  
  	MatrixXd y = z - hx_;
  	MatrixXd PHt = P_ * H_.transpose();
  	MatrixXd S = (H_ * PHt) + R_;
  	MatrixXd k = PHt * S.inverse();
  	I = MatrixXd::Identity(x_.size(),x_.size());
  	//std::cout << "I:" << std::endl; std::cout << I << std::endl;
	// new state
	x_ = x_ + (k * y);
	P_ = (I - (k * H_)) * P_;
  }
  
}
