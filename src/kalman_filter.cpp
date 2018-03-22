#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

  x_= F_* x_;
//   std::cout<<"====after predict====="<<std::endl;
//   std::cout<<P_<<std::endl;
//   std::cout<< x_<<std::endl;
//   std::cout<<x_<<std::endl;
  P_ = F_* P_ * F_.transpose() + Q_;
//   std::cout<<"========="<<std::endl;
//   std::cout<<P_<<std::endl;
//   std::cout<< x_<<std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd predicted_= H_* x_;
  MatrixXd y_= z- predicted_;
//   MatrixXd HT_= H_.transpose();
  MatrixXd HT_= H_laser_transpose_;
  MatrixXd PHT_= P_ * HT_;
  MatrixXd K_= PHT_ * ( H_ * PHT_ + R_ ).inverse();// Kalman Gain
  x_= x_ + (K_ * y_);
  
  
  
  P_= (I_- K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  VectorXd z_predicted_polar = VectorXd(3);  
  float px_= x_(0);
  float py_= x_(1);
  float vx_= x_(2);
  float vy_= x_(3);
  
  
  float rho_= sqrt(px_*px_+ py_*py_);
  float theta= atan2(py_,px_);
  float rho_dot;
  if(fabs(rho_)>0.0001)
    rho_dot = (px_*vx_+ py_*vy_)/rho_;
  else
  {

    px_ += .001;
    py_ += .001;
    rho_dot = sqrt(px_ * px_ + py_ * py_);
  }
    
  z_predicted_polar<< rho_,theta,rho_dot;
  
  VectorXd y_= z- z_predicted_polar;
  y_[1]= tools.NormalizePhi(y_[1]);
//   std::cout<<"====theta====="<<std::endl;
//   std::cout<<y_[1]<<std::endl;
  MatrixXd HT_= H_.transpose(); 
  MatrixXd K_= P_ * HT_ * ( H_ * P_* HT_ + R_ ).inverse();// Kalman Gain
  x_= x_ + (K_ * y_);
  
 
  MatrixXd I_ = MatrixXd::Identity(4, 4);
  P_= (I_- K_ * H_) * P_;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}


  

