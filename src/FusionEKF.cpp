#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.P_ = MatrixXd(4, 4);
	
  H_laser_ << 1, 0, 0, 0,
			        0, 1, 0, 0;
   
	Hj_ << 1,1,0,0,
				 1,1,0,0,
				 1,1,1,1;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  ekf_.H_laser_transpose_= H_laser_.transpose();
	ekf_.I_ = MatrixXd::Identity(4, 4);
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  ekf_.F_ <<  1,0,0,0,
							0,1,0,0,
							0,0,1,0,
							0,0,0,1;
  
  
  ekf_.P_ <<  1.0,0,0,0,
							0,1.0,0,0,
							0,0,1.0,0,
							0,0,0,1.0;
	
	float dt=0.05;
	float dt_2= dt*dt;
	float dt_3= dt_2*dt;
	float dt_4= dt_3*dt;
  float noise_ax= 9;
  float noise_ay= 9;
	ekf_.dt=dt;
	
	ekf_.F_(0,2)=dt;
  ekf_.F_(1,3)=dt;
	
  ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax,0,
	       0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
	       dt_3/2 * noise_ax, 0, dt_2 * noise_ax,0,
	       0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      float px= rho* cos(theta);
      float py= rho* sin(theta);
      float vx= rho_dot * cos(theta);
      float vy= rho_dot * sin(theta);
      ekf_.x_(0)= px;
			ekf_.x_(1)= py;
			ekf_.x_(2)= vx;
			ekf_.x_(3)= vy; 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {  
      ekf_.x_(0)= measurement_pack.raw_measurements_(0);
      ekf_.x_(1)= measurement_pack.raw_measurements_(1);
			ekf_.x_(2)= 5.0;
			ekf_.x_(3)= 0.0;
    }
		previous_timestamp_= measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt_curr = (measurement_pack.timestamp_- previous_timestamp_) /1000000.0;

	
	if(ekf_.dt!=dt_curr)  // modification to reduce unnecessary calculation of Q
	{
		ekf_.dt=dt_curr;
		ekf_.F_(0,2)=ekf_.dt;
		ekf_.F_(1,3)=ekf_.dt;

		float dt_2= ekf_.dt*ekf_.dt;
		float dt_3= dt_2*ekf_.dt;
		float dt_4= dt_3*ekf_.dt;
		float noise_ax= 9;
		float noise_ay= 9;

		ekf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax,0,
					 0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
					 dt_3/2 * noise_ax, 0, dt_2 * noise_ax,0,
					 0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;

	}
// 	cout<<"before"<<endl;
// 	cout << "x_ = " << ekf_.x_ << endl;
//   cout << "P_ = " << ekf_.P_ << endl;
// 	cout << "Q_ = " << ekf_.Q_ << endl;
  ekf_.Predict();
// 	cout<<"after"<<endl;
// 	cout << "x_ = " << ekf_.x_ << endl;
//   cout << "P_ = " << ekf_.P_ << endl;
// 	cout << "Q_ = " << ekf_.Q_ << endl;
	
	

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
		ekf_.R_ =  R_radar_; // Radar measurement error matrix
	  Hj_= tools.CalculateJacobian(ekf_.x_); // Jacobian matrix
		ekf_.H_= Hj_; 
		ekf_.UpdateEKF(measurement_pack.raw_measurements_); 
    
  } else {
    // Laser updates
		ekf_.R_ = R_laser_; // lidar measurement error matrix
	  ekf_.H_ = H_laser_; 
    ekf_.Update(measurement_pack.raw_measurements_);
  }
	previous_timestamp_= measurement_pack.timestamp_ ; 
	

  // print the output
//   cout << "x_ = " << ekf_.x_ << endl;
//   cout << "P_ = " << ekf_.P_ << endl;
}
