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
  x_= F_* x_;
  P_ = F_* P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

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
  if(fabs(rho_)<0.0001)
  {
    px_ += .001;
    py_ += .001;
    rho_= sqrt(px_*px_+ py_*py_);
  }
  rho_dot = (px_*vx_+ py_*vy_)/rho_;  
  z_predicted_polar<< rho_,theta,rho_dot;
  
  VectorXd y_= z- z_predicted_polar;
  y_[1]= tools.NormalizePhi(y_[1]);
  MatrixXd HT_= H_.transpose(); 
  MatrixXd K_= P_ * HT_ * ( H_ * P_* HT_ + R_ ).inverse();// Kalman Gain
  x_= x_ + (K_ * y_);
 
  MatrixXd I_ = MatrixXd::Identity(4, 4);
  P_= (I_- K_ * H_) * P_;

}


  

