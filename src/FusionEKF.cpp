#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

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

  //State
  VectorXd x(4);
  x << 0, 0, 0, 0;

  //measurement covariance matrix - laser
  //MatrixXd R_laser_(2, 2);
  R_laser_ << 0.0225, 0     ,
              0     , 0.0225;

  //measurement covariance matrix - radar
  //MatrixXd R_radar_(3, 3);
  R_radar_ << 0.09, 0     , 0   ,
              0   , 0.0009, 0   ,
              0   , 0     , 0.09;

  //projection matrix
  //atrixXd H_laser_(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //MatrixXd Hj_(3, 4);
  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;

  //State covariance
  //adjust to see effect on accuracy
  MatrixXd P(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  //state transition
  //no time-related element initially
  MatrixXd F(4, 4);
  F << 1, 0, .05, 0,
       0, 1, 0, .05,
       0, 0, 1, 0,
       0, 0, 0, 1;

  //process covariance matrix
  MatrixXd Q(4, 4);
  Q << 0, 0, 0, 0,
       0, 0, 0, 0, 
       0, 0, 0, 0,
       0, 0, 0, 0;

  //invoke and initialize a KalmanFilter instance
  ekf_ = KalmanFilter();
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);   

  //acceleration noise
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  Tools tools;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "Initialization - RADAR\n";

      cout << "Measurements: " << endl
           << measurement_pack.raw_measurements_[0] << endl
           << measurement_pack.raw_measurements_[1] << endl
           << measurement_pack.raw_measurements_[3] << endl;


      //Compute position in cartesian coordinates from polar
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);

      cout << "INITIAL STATE - RADAR: rho, phi, x_(0), x_(1)\n" 
           << rho << endl
           << phi << endl
           << ekf_.x_(0) << endl
           << ekf_.x_(1) << endl;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      /**
      Initialize state.
      */
      cout << "Initialization - LIDAR\n";

      cout << "Measurements: " << endl
           << measurement_pack.raw_measurements_[0] << endl
           << measurement_pack.raw_measurements_[1] << endl;

      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = px, 
      ekf_.x_(1) = py;

      cout << "INITIAL STATE - LIDAR: px, py, x_(0), x_(1)\n" 
           << px << endl
           << py << endl
           << ekf_.x_(0) << endl
           << ekf_.x_(1) << endl;
    } 

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    cout << "INITIAL TIMESTAMP: " << measurement_pack.timestamp_ << endl;

    is_initialized_ = true;
    cout << "is_initialized_ == true\n";
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  cout << "PREDICT\n";
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;  //convert from micros to s
  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "dt = " << dt << endl;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0.             , dt_3/2*noise_ax, 0.             ,
              0.             , dt_4/4*noise_ay, 0.             , dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0              , dt_2*noise_ax  , 0              ,
              0.             , dt_3/2*noise_ay, 0.             , dt_2*noise_ay  ;            
  
  cout << "dt powers: " << dt_2 << "\t" << dt_3 << "\t" << dt_4 << endl;
  cout << "noise_ax, noise_ay = " << noise_ax << "\t" << noise_ay << endl; 
  cout << "Q = " << ekf_.Q_ << endl;
  cout << "Q(0,0) = " << ekf_.Q_(0,0) << endl;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "************* UPDATE - Radar **************\n";
    cout << "z = " << measurement_pack.raw_measurements_ << endl;

    // Radar updates

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    cout << "Hj = " << ekf_.H_ << endl;
    cout << "R = " << ekf_.R_ << endl;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    cout << "************* UPDATE - Laser **************\n";
    cout << "z = " << measurement_pack.raw_measurements_ << endl;
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    cout << "H = " << ekf_.H_ << endl;
    cout << "R = " << ekf_.R_ << endl;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "*************** AFTER PREDICT + UPDATE *****************\n";
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
