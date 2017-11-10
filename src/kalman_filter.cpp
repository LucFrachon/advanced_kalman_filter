#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

const float EPSILON = 0.0001;

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

  x_ = F_ * x_;  //state prediction
  P_ = F_ * P_ * F_.transpose() + Q_;  //covariance matrix update
}

void KalmanFilter::Update(const VectorXd &z) {

  //equations for LIDAR measurements
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  //equations for RADAR measurements
  
  Tools tools;

  //define variables for easier reading and saving cycles
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px * px + py * py);
  //measurement function
  VectorXd h_of_x(3);
  //measurement error
  VectorXd y(3);
  
  if (px < EPSILON)
  {
    cout << "Px close to zero\n";
    float px = EPSILON;
  }

  if (rho < EPSILON) 
  { //if division by zero, keep the previous value for y
    cout << "Division by zero in UpdateEFK()\n";
    float rho = EPSILON;
    //float rho_dot = 0;
  } else
  { //otherwise calculate y = measurement error
    float rho_dot = (px * vx + py * vy) / rho;
    h_of_x << rho,
              atan2(py, px),
              rho_dot;
    
    y = z - h_of_x;  
    //normalize the angle element of y
    y(1) = atan2(sin(y(1)), cos(y(1)));

    cout << "Measured angle: " << z_norm(1) << endl;
    cout << "Computed angle: " << h_of_x(1) << endl;
    cout << "Error: " << y(1) << endl;

  }
  
  MatrixXd Hjt = H_.transpose();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd S = H_ * PHjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHjt * Si;

  //new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
